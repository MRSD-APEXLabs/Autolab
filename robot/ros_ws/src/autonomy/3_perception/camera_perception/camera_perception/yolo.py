"""Ultralytics YOLO detection (oriented boxes, boxes or segmentation) located in 3D with the stereo depth."""
from __future__ import annotations

from dataclasses import dataclass
import os

import cv2
import numpy as np

from .geometry import Pose, planar_pose


@dataclass
class Detection:
    cls_id: int
    name: str
    conf: float
    center: np.ndarray            # (2,) px
    size: np.ndarray              # (2,) px: width along theta, height across it
    theta: float                  # rad, image angle of the width (x right, y down); 0 for plain boxes
    polygon: np.ndarray           # (N, 2) px outline: OBB corners, box corners or mask contour
    pose: Pose | None = None      # x along the width, z the surface normal towards the camera
    metric_size: tuple | None = None   # (width, height) in metres on the fitted plane
    pose_source: str | None = None     # 'plane' or 'center_depth'

    def as_dict(self):
        """Camera-Edge `wellplates` / servo `detections` entry (center, size, theta, conf, cls_id, name, pose)
        plus metric_size and the outline."""
        return {
            'center': [float(v) for v in self.center],
            'size': [float(v) for v in self.size],
            'theta': float(self.theta),
            'conf': float(self.conf),
            'cls_id': int(self.cls_id),
            'name': self.name,
            'pose': self.pose.as_dict() if self.pose is not None else None,
            'pose_source': self.pose_source,
            'metric_size': None if self.metric_size is None else [float(v) for v in self.metric_size],
            'polygon': [[float(x), float(y)] for x, y in self.polygon],
        }


def _numpy(tensor):
    return tensor.detach().cpu().numpy() if hasattr(tensor, 'detach') else np.asarray(tensor)


def detections_from_result(result):
    """Detections of one ultralytics Results, whatever the task (obb, detect or segment)."""
    names = getattr(result, 'names', None) or {}
    detections = []
    obb = getattr(result, 'obb', None)
    if obb is not None and len(obb):
        xywhr = _numpy(obb.xywhr).reshape(-1, 5)
        corners = _numpy(obb.xyxyxyxy).reshape(-1, 4, 2)
        for i, (conf, cls_id) in enumerate(zip(_numpy(obb.conf).ravel(), _numpy(obb.cls).ravel())):
            x, y, w, h, theta = (float(v) for v in xywhr[i])
            detections.append(Detection(int(cls_id), str(names.get(int(cls_id), int(cls_id))), float(conf),
                                        np.array([x, y]), np.array([w, h]), theta, corners[i].astype(np.float64)))
        return detections
    boxes = getattr(result, 'boxes', None)
    if boxes is None or not len(boxes):
        return detections
    masks = getattr(result, 'masks', None)
    outlines = list(masks.xy) if masks is not None else []
    xyxy = _numpy(boxes.xyxy).reshape(-1, 4)
    for i, (conf, cls_id) in enumerate(zip(_numpy(boxes.conf).ravel(), _numpy(boxes.cls).ravel())):
        x0, y0, x1, y1 = (float(v) for v in xyxy[i])
        center, size, theta = np.array([(x0 + x1) / 2, (y0 + y1) / 2]), np.array([x1 - x0, y1 - y0]), 0.0
        polygon = np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]])
        outline = np.asarray(outlines[i], np.float64).reshape(-1, 2) if i < len(outlines) else np.empty((0, 2))
        if len(outline) >= 3:
            # the mask's minimum-area rectangle: its orientation, and the mask itself for the depth
            (cx, cy), (w, h), angle = cv2.minAreaRect(outline.astype(np.float32))
            center, size, theta, polygon = np.array([cx, cy]), np.array([w, h]), float(np.deg2rad(angle)), outline
        detections.append(Detection(int(cls_id), str(names.get(int(cls_id), int(cls_id))), float(conf),
                                    center, size, theta, polygon))
    return detections


def parse_classes(text, names):
    """Class ids from 'name, name' or 'id id' (names as the model spells them); empty keeps all."""
    ids = []
    by_name = {str(v): int(k) for k, v in names.items()}
    for item in text.replace(',', ' ').split():
        if item.lstrip('-').isdigit():
            ids.append(int(item))
        elif item in by_name:
            ids.append(by_name[item])
        else:
            raise ValueError(f'class {item!r} is not one of the model classes {sorted(by_name)}')
    return ids


def clahe(bgr, clip, tiles):
    """Contrast-limited histogram equalization of the lightness (LAB L), colours kept."""
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    lab[..., 0] = cv2.createCLAHE(clipLimit=float(clip), tileGridSize=(int(tiles), int(tiles))).apply(lab[..., 0])
    return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)


class YoloDetector:
    """An ultralytics model (task from the weights) with the thresholds applied at inference."""

    def __init__(self, model_path, device='cuda:0', conf=0.25, iou=0.7, imgsz=0, classes='', max_detections=0,
                 half=False, clahe_clip=0.0, clahe_tiles=4):
        os.environ.setdefault('YOLO_OFFLINE', '1')   # no downloads or telemetry from a robot
        if not os.path.isfile(model_path):
            raise FileNotFoundError(f'YOLO model {model_path} not found')
        from ultralytics import YOLO   # heavy (torch): import where it is needed
        import torch
        if str(device).startswith('cuda') and not torch.cuda.is_available():
            device = 'cpu'
        self.device = device
        self.model = YOLO(model_path)
        self.task = self.model.task
        self.names = dict(self.model.names)
        self.classes = parse_classes(classes, self.names) or None
        self.conf, self.iou = float(conf), float(iou)
        self.imgsz = int(imgsz) or None
        self.max_detections = int(max_detections)
        self.half = bool(half) and device != 'cpu'
        self.clahe_clip, self.clahe_tiles = float(clahe_clip), int(clahe_tiles)   # clip 0 = no CLAHE
        if self.clahe_clip > 0.0 and self.clahe_tiles < 1:
            raise ValueError(f'clahe_tiles must be >= 1, got {clahe_tiles}')

    def describe(self):
        return {'task': self.task, 'classes': self.names, 'device': self.device,
                'clahe': f'clip {self.clahe_clip}, {self.clahe_tiles}x{self.clahe_tiles} tiles' if self.clahe_clip > 0.0 else 'off'}

    def predict(self, bgr):
        if self.clahe_clip > 0.0:   # the model's input only; outputs keep the camera image's pixel coordinates
            bgr = clahe(bgr, self.clahe_clip, self.clahe_tiles)
        kwargs = {'conf': self.conf, 'iou': self.iou, 'device': self.device, 'classes': self.classes, 'verbose': False}
        if self.imgsz:
            kwargs['imgsz'] = self.imgsz
        if self.half:   # only when asked: recent ultralytics warns about the argument on every call
            kwargs['half'] = True
        return self.model.predict(bgr, **kwargs)[0]

    def warmup(self, shape=(600, 960, 3)):
        self.predict(np.zeros(shape, np.uint8))

    def detect(self, bgr, depth_mm=None, intrinsics=None):
        """Detections, most confident first (at most max_detections if > 0), located if depth is given."""
        detections = sorted(detections_from_result(self.predict(bgr)), key=lambda d: -d.conf)
        if self.max_detections > 0:
            detections = detections[:self.max_detections]
        if depth_mm is not None and intrinsics is not None:
            for det in detections:
                locate(det, depth_mm, intrinsics)
        return detections


def locate(det, depth_mm, intrinsics):
    """Pose of the detection's (roughly flat) visible face from the depth inside its outline."""
    found = planar_pose(depth_mm, intrinsics, det.center, det.size, det.theta, polygon=det.polygon)
    if found is not None:
        det.pose, det.metric_size, det.pose_source = found
    return det
