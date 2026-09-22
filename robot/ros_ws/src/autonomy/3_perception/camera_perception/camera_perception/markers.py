"""Outputs in the pc3.py formats: RViz MarkerArrays, an annotated image and a JSON summary.

The marker layouts match what ~/coding/Autolab/pc3.py published and move_to_pose_node reads:

- AprilTags: DELETEALL, then one SPHERE per tag (ns `apriltags`, id = tag id, 2 cm, red). The planner stores
  every ADD marker's position under its id, so this array holds nothing else.
- Detections: DELETEALL, then per detection a CUBE (ns e.g. `wellplates`, id i, metric size) and a
  TEXT_VIEW_FACING label (ns `<ns>_text`, id 1000 + i). Cubes are ordered by ascending confidence, so the
  last one of the namespace (the one the planner keeps) is the most confident.

Only objects with a 3D pose get markers: pc3.py drew pose-less ones at pixel/500 "positions", which a
consumer could not tell from real ones.
"""
from __future__ import annotations

import json

import cv2
import numpy as np
from geometry_msgs.msg import Point, Pose as PoseMsg, Quaternion
from visualization_msgs.msg import Marker, MarkerArray

from .geometry import rotation_to_quaternion

TAG_COLOR = (0, 0, 255)          # BGR, annotations
DETECTION_COLOR = (0, 255, 0)
AXIS_COLORS = ((0, 0, 255), (0, 255, 0), (255, 0, 0))   # x red, y green, z blue


def pose_msg(pose):
    msg = PoseMsg()
    msg.position = Point(x=float(pose.position[0]), y=float(pose.position[1]), z=float(pose.position[2]))
    q = rotation_to_quaternion(pose.rotation)
    msg.orientation = Quaternion(x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3]))
    return msg


def _delete_all(header):
    marker = Marker()
    marker.header = header
    marker.action = Marker.DELETEALL
    return marker


def apriltag_markers(header, tags, ns='apriltags'):
    array = MarkerArray()
    array.markers.append(_delete_all(header))
    for tag in tags:
        if tag.pose is None:
            continue
        m = Marker()
        m.header, m.ns, m.id = header, ns, int(tag.id)
        m.type, m.action = Marker.SPHERE, Marker.ADD
        m.pose = pose_msg(tag.pose)
        m.scale.x = m.scale.y = m.scale.z = 0.02
        m.color.r, m.color.a = 1.0, 1.0
        array.markers.append(m)
    return array


def detection_markers(header, detections, ns='detections'):
    array = MarkerArray()
    array.markers.append(_delete_all(header))
    located = sorted((d for d in detections if d.pose is not None), key=lambda d: d.conf)
    for i, det in enumerate(located):
        m = Marker()
        m.header, m.ns, m.id = header, ns, i
        m.type, m.action = Marker.CUBE, Marker.ADD
        m.pose = pose_msg(det.pose)
        width, height = det.metric_size if det.metric_size is not None else (0.1, 0.06)
        m.scale.x, m.scale.y, m.scale.z = max(float(width), 0.01), max(float(height), 0.01), 0.01
        m.color.g, m.color.a = 1.0, 0.8
        array.markers.append(m)
        text = Marker()
        text.header, text.ns, text.id = header, f'{ns}_text', 1000 + i
        text.type, text.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        text.pose.position = Point(x=m.pose.position.x, y=m.pose.position.y, z=m.pose.position.z + 0.03)
        text.pose.orientation.w = 1.0
        text.scale.z = 0.06
        text.color.g, text.color.a = 1.0, 1.0
        text.text = f'{det.name} {det.conf:.2f}'
        array.markers.append(text)
    return array


def _project(intrinsics, point):
    return (int(round(intrinsics.fx * point[0] / point[2] + intrinsics.cx)),
            int(round(intrinsics.fy * point[1] / point[2] + intrinsics.cy)))


def _draw_axes(image, intrinsics, pose, length):
    if pose is None or pose.position[2] <= 0.0:
        return
    origin = _project(intrinsics, pose.position)
    for axis, color in zip(pose.rotation.T, AXIS_COLORS):
        tip = pose.position + length * axis
        if tip[2] > 0.0:
            cv2.line(image, origin, _project(intrinsics, tip), color, 2, cv2.LINE_AA)


def annotate(bgr, tags=(), detections=(), intrinsics=None):
    """A copy of the image with the detections' outlines, labels and (given intrinsics) pose axes."""
    image = bgr.copy()
    font = cv2.FONT_HERSHEY_SIMPLEX
    for det in detections:
        cv2.polylines(image, [np.round(det.polygon).astype(np.int32).reshape(-1, 1, 2)], True, DETECTION_COLOR, 2)
        u, v = int(det.center[0]), int(det.center[1])
        cv2.circle(image, (u, v), 4, DETECTION_COLOR, -1)
        label = f'{det.name} {det.conf:.2f}'
        if det.pose is not None:
            label += f' z={det.pose.position[2]:.3f}m'
        cv2.putText(image, label, (u + 6, v - 6), font, 0.55, DETECTION_COLOR, 2, cv2.LINE_AA)
        if intrinsics is not None and det.metric_size is not None:
            _draw_axes(image, intrinsics, det.pose, 0.5 * min(det.metric_size))
    for tag in tags:
        cv2.polylines(image, [np.round(tag.corners).astype(np.int32).reshape(-1, 1, 2)], True, TAG_COLOR, 2)
        u, v = int(tag.center[0]), int(tag.center[1])
        label = f'ID {tag.id}'
        if tag.pose is not None:
            label += f' z={tag.pose.position[2]:.3f}m'
        cv2.putText(image, label, (u + 6, v + 18), font, 0.55, TAG_COLOR, 2, cv2.LINE_AA)
        if intrinsics is not None:
            _draw_axes(image, intrinsics, tag.pose, tag.size_m)
    return image


def summary_json(header, tags=(), detections=(), detections_key='detections', extra=None):
    """The frame's results as JSON: Camera-Edge's `apriltags` and detection lists, plus the stamp/frame."""
    payload = {
        'stamp': {'sec': int(header.stamp.sec), 'nanosec': int(header.stamp.nanosec)},
        'frame_id': header.frame_id,
        'apriltags': [t.as_dict() for t in tags],
        detections_key: [d.as_dict() for d in detections],
    }
    payload.update(extra or {})
    return json.dumps(payload, separators=(',', ':'))
