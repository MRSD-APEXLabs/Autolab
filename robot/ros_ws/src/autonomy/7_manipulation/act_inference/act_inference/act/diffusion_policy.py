"""Conditional DDPM policy for a fixed-length chunk of normalized TCP actions.

The image encoder conditions a temporal convolutional denoiser. Inference uses
deterministic DDIM steps so repeated queries with the same observation agree.
"""
import math

import torch
from torch import nn
import torch.nn.functional as F
from torchvision.models import resnet18

from .image_inputs import DEPTH_ENCODING


DIFFUSION_SCHEDULES = ('linear', 'cosine')
DIFFUSION_PREDICTIONS = ('epsilon', 'v')


def diffusion_alpha_bar(schedule, steps):
    """Cumulative signal fraction alpha_bar[t] for t = 0 .. steps-1."""
    if schedule == 'linear':
        # Legacy (checkpoints without 'diffusion_schedule'). DDPM's linear betas are meant for
        # ~1000 steps: with 100 the noisiest training input still holds 0.6 x the action
        # (alpha_bar = 0.36), while sampling starts from pure noise, so the sampler's random
        # start decides the output instead of the observation.
        betas = torch.linspace(1e-4, .02, steps)
    elif schedule == 'cosine':
        # Squared-cosine schedule (Nichol & Dhariwal 2021, Diffusion Policy's squaredcos_cap_v2):
        # the noisiest training input is pure noise, exactly where sampling starts.
        s = .008
        f = torch.cos((torch.linspace(0, 1, steps + 1, dtype=torch.float64) + s) / (1 + s) * math.pi / 2) ** 2
        betas = (1 - f[1:] / f[:-1]).clamp(max=.999).float()
    else:
        raise ValueError(f'Unknown diffusion schedule: {schedule}')
    return torch.cumprod(1 - betas, dim=0)


def _time_embedding(timestep, width):
    half = width // 2
    frequency = torch.exp(-math.log(10000) * torch.arange(half, device=timestep.device) / max(half - 1, 1))
    angles = timestep.float()[:, None] * frequency[None]
    embedding = torch.cat((angles.sin(), angles.cos()), dim=-1)
    return F.pad(embedding, (0, width - embedding.shape[-1]))


class _ResidualTemporalBlock(nn.Module):
    def __init__(self, width, dilation):
        super().__init__()
        self.conv1 = nn.Conv1d(width, width, 3, padding=dilation, dilation=dilation)
        self.conv2 = nn.Conv1d(width, width, 3, padding=1)
        self.norm1 = nn.GroupNorm(8, width)
        self.norm2 = nn.GroupNorm(8, width)
        self.condition = nn.Linear(width, width * 2)

    def forward(self, sample, condition):
        scale, shift = self.condition(condition).chunk(2, dim=-1)
        hidden = self.conv1(F.silu(self.norm1(sample)))
        hidden = self.norm2(hidden) * (1 + scale[..., None]) + shift[..., None]
        return sample + self.conv2(F.silu(hidden))


class DiffusionPolicy(nn.Module):
    def __init__(self, config, pretrained_backbone=False):
        super().__init__()
        self.use_depth = bool(config.get('use_depth', False))
        if self.use_depth and config.get('depth_encoding') != DEPTH_ENCODING:
            raise ValueError('Unsupported or missing checkpoint depth encoding')
        self.num_queries = int(config['num_queries'])
        self.train_steps = int(config.get('diffusion_train_steps', 100))
        self.inference_steps = int(config.get('diffusion_inference_steps', 16))
        width = int(config.get('diffusion_hidden_dim', 256))
        if self.num_queries < 1 or self.train_steps < 2 or not 1 <= self.inference_steps <= self.train_steps:
            raise ValueError('Invalid diffusion horizon or step count')
        if width < 8 or width % 8:
            raise ValueError('Diffusion hidden dimension must be a multiple of eight')
        self.inference_seed = int(config.get('diffusion_inference_seed', 0))
        # Older checkpoints predate both keys and keep their linear/epsilon behaviour. With the
        # cosine schedule the noisiest step has alpha_bar ~ 0, where recovering the action from a
        # noise estimate divides by ~0, so new checkpoints predict v (Salimans & Ho 2022) instead.
        self.schedule = config.get('diffusion_schedule', 'linear')
        self.prediction = config.get('diffusion_prediction', 'epsilon')
        if self.schedule not in DIFFUSION_SCHEDULES or self.prediction not in DIFFUSION_PREDICTIONS:
            raise ValueError('Unsupported diffusion schedule or prediction type')
        backbone = resnet18(pretrained=pretrained_backbone)
        if self.use_depth:
            old = backbone.conv1
            expanded = nn.Conv2d(5, old.out_channels, old.kernel_size, old.stride, old.padding, bias=False)
            with torch.no_grad():
                expanded.weight[:, :3].copy_(old.weight)
                expanded.weight[:, 3:4].copy_(old.weight.mean(dim=1, keepdim=True))
                expanded.weight[:, 4:].zero_()
            backbone.conv1 = expanded
        backbone.fc = nn.Identity()
        self.image_encoder = backbone
        self.condition = nn.Sequential(nn.Linear(512 + int(config.get('observation_dim', 6)) + width, width), nn.SiLU(), nn.Linear(width, width))
        self.input = nn.Conv1d(6, width, 1)
        self.blocks = nn.ModuleList(_ResidualTemporalBlock(width, dilation) for dilation in (1, 2, 4, 8, 16))
        self.output = nn.Sequential(nn.GroupNorm(8, width), nn.SiLU(), nn.Conv1d(width, 6, 1))
        self.register_buffer('alpha_bar', diffusion_alpha_bar(self.schedule, self.train_steps))
        self.lr = float(config['lr'])

    def _context(self, qpos, image):
        expected = 5 if self.use_depth else 3
        if image.ndim != 5 or image.shape[1] != 1 or image.shape[2] != expected:
            raise ValueError(f'Diffusion expects one camera with {expected} image channels')
        rgb = image[:, 0, :3]
        mean = rgb.new_tensor((.485, .456, .406))[None, :, None, None]
        std = rgb.new_tensor((.229, .224, .225))[None, :, None, None]
        rgb = (rgb - mean) / std
        pixels = torch.cat((rgb, image[:, 0, 3:]), dim=1) if self.use_depth else rgb
        return torch.cat((self.image_encoder(pixels), qpos), dim=-1)

    def _predict_noise(self, sample, timestep, context):
        condition = self.condition(torch.cat((context, _time_embedding(timestep, self.input.out_channels)), dim=-1))
        hidden = self.input(sample.transpose(1, 2))
        for block in self.blocks:
            hidden = block(hidden, condition)
        return self.output(hidden).transpose(1, 2)

    def forward(self, qpos, image, actions=None, is_pad=None):
        context = self._context(qpos, image)
        if actions is not None:
            actions = actions[:, :self.num_queries]
            is_pad = is_pad[:, :self.num_queries]
            if actions.shape[1] != self.num_queries:
                raise ValueError('Diffusion training requires a full padded action chunk')
            # Extend the final valid waypoint through padded positions; the loss
            # still counts valid positions only.
            last = (~is_pad).sum(dim=1).clamp(min=1) - 1
            terminal = actions[torch.arange(len(actions), device=actions.device), last][:, None]
            actions = torch.where(is_pad[..., None], terminal, actions)
            timestep = torch.randint(self.train_steps, (len(actions),), device=actions.device)
            noise = torch.randn_like(actions)
            alpha = self.alpha_bar[timestep][:, None, None]
            noisy = alpha.sqrt() * actions + (1 - alpha).sqrt() * noise
            estimate = self._predict_noise(noisy, timestep, context)
            target = noise if self.prediction == 'epsilon' else alpha.sqrt() * noise - (1 - alpha).sqrt() * actions
            valid = (~is_pad)[..., None]
            loss = ((estimate - target).square() * valid).sum() / (valid.sum().clamp(min=1) * 6)
            return {'loss': loss, 'noise_mse': loss}
        generator = torch.Generator(device=qpos.device).manual_seed(self.inference_seed)
        sample = torch.randn((len(qpos), self.num_queries, 6), device=qpos.device, generator=generator)
        schedule = torch.linspace(self.train_steps - 1, 0, self.inference_steps, device=qpos.device).round().long()
        schedule = torch.unique_consecutive(schedule)
        for index, t in enumerate(schedule):
            alpha = self.alpha_bar[t]
            output = self._predict_noise(sample, t.expand(len(qpos)), context)
            if self.prediction == 'epsilon':
                estimate = output
                clean = ((sample - (1 - alpha).sqrt() * estimate) / alpha.sqrt()).clamp(-10, 10)
            else:
                clean = (alpha.sqrt() * sample - (1 - alpha).sqrt() * output).clamp(-10, 10)
                estimate = (1 - alpha).sqrt() * sample + alpha.sqrt() * output
            next_alpha = self.alpha_bar[schedule[index + 1]] if index + 1 < len(schedule) else alpha.new_tensor(1.0)
            sample = next_alpha.sqrt() * clean + (1 - next_alpha).sqrt() * estimate
        return sample

    def configure_optimizers(self):
        return torch.optim.AdamW(self.parameters(), lr=self.lr)
