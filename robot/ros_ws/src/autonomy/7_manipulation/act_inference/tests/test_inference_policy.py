import pickle

import numpy as np
import pytest

torch = pytest.importorskip('torch')

from act_inference.act.image_inputs import encode_image  # noqa: E402
from act_inference.rollout import load_policy, predict_chunk  # noqa: E402
from inference_fakes import write_tiny_checkpoint  # noqa: E402


@pytest.mark.parametrize('use_depth', [False, True])
def test_checkpoint_loads_and_predicts_a_chunk(tmp_path, use_depth):
    ckpt = write_tiny_checkpoint(tmp_path / 'ckpt', use_depth=use_depth, chunk=10)
    policy, stats, device = load_policy(ckpt)
    assert policy.use_depth == use_depth
    rng = np.random.RandomState(1)
    rgb = rng.randint(0, 255, (600, 960, 3)).astype(np.uint8)
    depth = rng.randint(0, 3000, (600, 960)).astype(np.uint16)
    pose = np.array([.3, 0, .2, 3.1, 0, 0])
    actions = predict_chunk(policy, stats, device, pose, rgb, 480, depth if use_depth else None)
    assert actions.shape == (10, 6) and np.isfinite(actions).all()
    # deterministic at inference (latent from the prior mean), and de-normalized with the dataset statistics
    np.testing.assert_array_equal(actions, predict_chunk(policy, stats, device, pose, rgb, 480, depth if use_depth else None))
    shifted = dict(stats, action_mean=stats['action_mean'] + 1.0)
    np.testing.assert_allclose(predict_chunk(policy, shifted, device, pose, rgb, 480, depth if use_depth else None),
                               actions + 1.0, atol=1e-6)


def test_depth_policy_requires_valid_depth(tmp_path):
    policy, stats, device = load_policy(write_tiny_checkpoint(tmp_path / 'ckpt', use_depth=True))
    rgb = np.zeros((600, 960, 3), np.uint8)
    with pytest.raises(ValueError, match='aligned uint16 depth'):
        predict_chunk(policy, stats, device, np.zeros(6), rgb, 480, None)
    with pytest.raises(ValueError, match='entirely invalid'):
        predict_chunk(policy, stats, device, np.zeros(6), rgb, 480, np.zeros((600, 960), np.uint16))


def test_encode_image_channels():
    rgb = np.full((4, 4, 3), 255, np.uint8)
    depth = np.array([[0, 1000, 2000, 5000]] * 4, np.uint16)
    encoded = encode_image(rgb, depth, use_depth=True)
    assert encoded.shape == (4, 4, 5) and encoded[..., :3].max() == 1.0
    np.testing.assert_allclose(encoded[0, :, 3], [0, .5, 1, 1])
    np.testing.assert_array_equal(encoded[0, :, 4], [0, 1, 1, 1])
    assert encode_image(rgb).shape == (4, 4, 3)


@pytest.mark.parametrize('change', [{'robot': 'ur5e'}, {'camera_names': ['a', 'b']}, {'state_dim': 7},
                                    {'state_space': 'joints'}])
def test_foreign_checkpoints_are_refused(tmp_path, change):
    ckpt = write_tiny_checkpoint(tmp_path / 'ckpt')
    with open(ckpt / 'config.pkl', 'rb') as f:
        config = pickle.load(f)
    with open(ckpt / 'config.pkl', 'wb') as f:
        pickle.dump(dict(config, **change), f)
    with pytest.raises(ValueError, match='Checkpoint must be'):
        load_policy(ckpt)


def test_invalid_statistics_are_refused(tmp_path):
    ckpt = write_tiny_checkpoint(tmp_path / 'ckpt')
    with open(ckpt / 'dataset_stats.pkl', 'rb') as f:
        stats = pickle.load(f)
    with open(ckpt / 'dataset_stats.pkl', 'wb') as f:
        pickle.dump(dict(stats, action_std=np.zeros(6)), f)
    with pytest.raises(ValueError, match='action_std'):
        load_policy(ckpt)
