"""YOLO results to located detections: fake ultralytics results, plus the real models when available."""
import os
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from camera_perception.yolo import clahe, detections_from_result, locate, parse_classes
from scene import INTRINSICS, Scene, rx

MODELS = Path(os.environ.get('CAMERA_PERCEPTION_TEST_MODELS', '/home/labx/apple_server_manip/data/models/yolo'))
DATA = Path(__file__).parent / 'data'


class Sized(SimpleNamespace):
    """An ultralytics Boxes/OBB stand-in: attributes plus a length."""

    def __init__(self, n, **fields):
        super().__init__(**fields)
        self._n = n

    def __len__(self):
        return self._n


def fake_obb(xywhr, conf, cls, names={0: 'wellplate'}):
    xywhr = np.asarray(xywhr, np.float32).reshape(-1, 5)
    corners = []
    for x, y, w, h, r in xywhr:   # ultralytics xywhr2xyxyxyxy
        c, s = np.cos(r), np.sin(r)
        v1, v2 = np.array([w / 2 * c, w / 2 * s]), np.array([-h / 2 * s, h / 2 * c])
        ctr = np.array([x, y])
        corners.append([ctr + v1 + v2, ctr + v1 - v2, ctr - v1 - v2, ctr - v1 + v2])
    obb = Sized(len(xywhr), xywhr=xywhr, xyxyxyxy=np.array(corners, np.float32), conf=np.asarray(conf, np.float32),
                cls=np.asarray(cls, np.float32))
    return SimpleNamespace(names=names, obb=obb, boxes=None, masks=None)


def test_obb_results():
    result = fake_obb([[500, 300, 120, 80, 0.3], [100, 100, 20, 10, 1.0]], [0.9, 0.4], [0, 0])
    dets = detections_from_result(result)
    assert [d.name for d in dets] == ['wellplate', 'wellplate']
    first = dets[0]
    assert first.conf == pytest.approx(0.9) and first.theta == pytest.approx(0.3)
    np.testing.assert_allclose(first.center, [500, 300])
    np.testing.assert_allclose(first.size, [120, 80])
    assert first.polygon.shape == (4, 2)
    np.testing.assert_allclose(first.polygon.mean(axis=0), [500, 300], atol=1e-3)
    empty = SimpleNamespace(names={}, obb=Sized(0), boxes=None, masks=None)
    assert detections_from_result(empty) == []


def test_box_and_segmentation_results():
    names = {0: 'Opentron', 4: 'wellplate'}
    boxes = Sized(2, xyxy=np.array([[10, 20, 50, 40], [100, 100, 200, 160]], np.float32),
                  conf=np.array([0.5, 0.8], np.float32), cls=np.array([0, 4], np.float32))
    plain = detections_from_result(SimpleNamespace(names=names, obb=None, boxes=boxes, masks=None))
    assert [d.name for d in plain] == ['Opentron', 'wellplate']
    np.testing.assert_allclose(plain[0].center, [30, 30])
    np.testing.assert_allclose(plain[0].size, [40, 20])
    assert plain[0].theta == 0.0 and plain[0].polygon.shape == (4, 2)
    # a mask outline turned 30 degrees: the detection takes its minimum-area rectangle and keeps the outline
    c, s = np.cos(np.radians(30)), np.sin(np.radians(30))
    outline = np.array([[-40, -20], [40, -20], [40, 20], [-40, 20]]) @ np.array([[c, s], [-s, c]]) + [150, 130]
    masks = SimpleNamespace(xy=[np.zeros((0, 2), np.float32), outline.astype(np.float32)])
    segmented = detections_from_result(SimpleNamespace(names=names, obb=None, boxes=boxes, masks=masks))
    np.testing.assert_allclose(segmented[0].center, [30, 30])            # empty outline: the box
    mask_det = segmented[1]
    np.testing.assert_allclose(mask_det.center, [150, 130], atol=0.5)
    assert sorted(mask_det.size) == pytest.approx([40, 80], abs=0.5)
    direction = np.array([np.cos(mask_det.theta), np.sin(mask_det.theta)])
    long_axis = np.array([c, s]) if mask_det.size[0] > mask_det.size[1] else np.array([-s, c])
    assert abs(direction @ long_axis) == pytest.approx(1.0, abs=1e-3)
    assert len(mask_det.polygon) == 4
    assert detections_from_result(SimpleNamespace(names=names, obb=None, boxes=None, masks=None)) == []


def test_clahe_raises_local_contrast_and_keeps_the_image_layout():
    gray = np.random.default_rng(0).integers(30, 50, (60, 80)).astype(np.uint8)   # dim, low-contrast texture
    image = np.dstack([gray] * 3)
    out = clahe(image, 3.0, 4)
    assert out.shape == image.shape and out.dtype == np.uint8
    assert out.std() > 2 * image.std()
    assert np.corrcoef(gray.ravel(), out[..., 1].ravel())[0, 1] > 0.9     # same pattern, stretched


def test_parse_classes():
    names = {0: 'Opentron', 3: 'pipette_holder', 4: 'wellplate'}
    assert parse_classes('', names) == []
    assert parse_classes('wellplate, 3 Opentron', names) == [4, 3, 0]
    with pytest.raises(ValueError, match='not one of'):
        parse_classes('plate', names)


def test_locate_a_detection_on_a_tilted_plane():
    scene = Scene(rotation=rx(0.5), origin=(0.0, 0.05, 0.8))
    det = detections_from_result(fake_obb([[480, 320, 110, 70, 0.2]], [0.95], [0]))[0]
    locate(det, scene.depth_mm(), INTRINSICS)
    assert det.pose_source == 'plane'
    np.testing.assert_allclose(det.pose.rotation[:, 2], -scene.rotation[:, 2], atol=0.01)
    ray = INTRINSICS.ray(480, 320)
    np.testing.assert_allclose(det.pose.position / det.pose.position[2], ray, atol=1e-6)   # on the centre's ray
    assert abs((det.pose.position - scene.origin) @ scene.rotation[:, 2]) < 0.002          # and on the plane
    assert det.metric_size[0] > det.metric_size[1] > 0.1
    d = det.as_dict()
    assert set(d) >= {'center', 'size', 'theta', 'conf', 'cls_id', 'name', 'pose', 'metric_size'}
    nothing = detections_from_result(fake_obb([[480, 320, 110, 70, 0.2]], [0.95], [0]))[0]
    locate(nothing, np.zeros((600, 960), np.uint16), INTRINSICS)
    assert nothing.pose is None and nothing.as_dict()['pose'] is None


@pytest.mark.parametrize('model, task', [('best_top.pt', 'obb'), ('best_seg.pt', 'segment')])
def test_real_model(model, task):
    pytest.importorskip('ultralytics')
    if not (MODELS / model).is_file():
        pytest.skip(f'{MODELS / model} not found (CAMERA_PERCEPTION_TEST_MODELS)')
    from camera_perception.yolo import YoloDetector
    detector = YoloDetector(str(MODELS / model), conf=0.5)
    assert detector.task == task and 'wellplate' in detector.names.values()
    scene = Scene(origin=(0.0, 0.0, 0.9))
    assert detector.detect(scene.image, scene.depth_mm(), INTRINSICS) == []    # a blank gray plane
    only = YoloDetector(str(MODELS / model), classes='wellplate', max_detections=1)
    assert only.classes == [next(k for k, v in only.names.items() if v == 'wellplate')]


def test_clahe_lets_the_wrist_model_find_the_clear_plate_on_the_zedx_deck():
    """A 2026-09-22 ZED X frame (JPEG): a clear 96-well plate on the dim Opentrons deck, below AprilTag 1."""
    pytest.importorskip('ultralytics')
    if not (MODELS / 'best_wrist.pt').is_file():
        pytest.skip(f'{MODELS / "best_wrist.pt"} not found (CAMERA_PERCEPTION_TEST_MODELS)')
    import cv2
    from camera_perception.yolo import YoloDetector
    image = cv2.imread(str(DATA / 'zedx_clear_plate.jpg'))
    plain = YoloDetector(str(MODELS / 'best_wrist.pt'), conf=0.8)
    assert plain.detect(image) == []
    with_clahe = YoloDetector(str(MODELS / 'best_wrist.pt'), conf=0.8, clahe_clip=3.0, clahe_tiles=4)
    found = with_clahe.detect(image)
    assert len(found) == 1 and found[0].name == 'wellplate'
    np.testing.assert_allclose(found[0].center, [554, 475], atol=5)
    assert with_clahe.describe()['clahe'] == 'clip 3.0, 4x4 tiles'
