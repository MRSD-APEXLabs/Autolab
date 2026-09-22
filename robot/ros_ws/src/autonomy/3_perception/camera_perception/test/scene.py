"""Synthetic scenes for the tests: a textured plane with AprilTags seen by a ZED X-like rectified camera,
rendered with its exact depth, so every pose, size and distance is known."""
import cv2
import numpy as np

from camera_perception.geometry import Intrinsics

W, H = 960, 600
INTRINSICS = Intrinsics(365.0, 365.0, 479.5, 299.5, W, H)
CELLS = 8          # a 36h11 tag is 8 x 8 cells from black edge to black edge (6 x 6 data bits)
QUIET = 2          # white cells rendered around it


def rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def angle_between(r1, r2):
    """Angle (deg) of the rotation taking r1 to r2."""
    cos = (np.trace(r1.T @ r2) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(cos, -1.0, 1.0))))


def tag_texture(tag_id, cell_px=24):
    """The tag as OpenCV draws DICT_APRILTAG_36h11 markers, with a white quiet zone, black edge = CELLS cells."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    draw = getattr(cv2.aruco, 'generateImageMarker', None) or cv2.aruco.drawMarker
    marker = draw(dictionary, int(tag_id), CELLS * cell_px)
    pad = QUIET * cell_px
    return cv2.copyMakeBorder(marker, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=255)


class Scene:
    """A plane (rotation `rotation`: its x, y in-plane, z away from the camera; origin `origin` in metres)
    textured gray, with tags pasted on it at plane coordinates."""

    def __init__(self, rotation=np.eye(3), origin=(0.0, 0.0, 1.0), intrinsics=INTRINSICS, gray=150):
        self.rotation = np.asarray(rotation, np.float64)
        self.origin = np.asarray(origin, np.float64)
        self.intrinsics = intrinsics
        self.image = np.full((intrinsics.height, intrinsics.width, 3), gray, np.uint8)
        self.tags = {}

    def tag_pose(self, xy=(0.0, 0.0), angle=0.0):
        """(rotation, centre) of a texture pasted at plane point `xy`, turned by `angle` in the plane."""
        rotation = self.rotation @ rz(angle)
        return rotation, self.origin + self.rotation @ np.array([xy[0], xy[1], 0.0])

    def add_tag(self, tag_id, size, xy=(0.0, 0.0), angle=0.0):
        texture = tag_texture(tag_id)
        rotation, centre = self.tag_pose(xy, angle)
        n = texture.shape[0]
        full = size * (CELLS + 2 * QUIET) / CELLS          # texture edge in metres, quiet zone included
        texture_to_tag = np.array([[full / n, 0.0, -full / 2], [0.0, full / n, -full / 2], [0.0, 0.0, 1.0]])
        homography = self.intrinsics.matrix @ np.column_stack([rotation[:, 0], rotation[:, 1], centre]) @ texture_to_tag
        # half-pixel shift: warpPerspective maps pixel centres
        shift = np.array([[1.0, 0.0, 0.5], [0.0, 1.0, 0.5], [0.0, 0.0, 1.0]])
        warp = np.array([[1.0, 0.0, -0.5], [0.0, 1.0, -0.5], [0.0, 0.0, 1.0]]) @ homography @ shift
        size_wh = (self.intrinsics.width, self.intrinsics.height)
        rendered = cv2.warpPerspective(texture, warp, size_wh, flags=cv2.INTER_AREA, borderValue=0)
        mask = cv2.warpPerspective(np.full_like(texture, 255), warp, size_wh, flags=cv2.INTER_NEAREST, borderValue=0)
        self.image[mask > 0] = rendered[mask > 0][:, None]
        self.tags[tag_id] = (rotation, centre, size)
        return rotation, centre

    def depth_mm(self):
        """Exact depth of the plane at every pixel, uint16 millimetres."""
        k = self.intrinsics
        us, vs = np.meshgrid(np.arange(k.width, dtype=np.float64), np.arange(k.height, dtype=np.float64))
        rays = np.stack([(us - k.cx) / k.fx, (vs - k.cy) / k.fy, np.ones_like(us)], axis=-1)
        normal = self.rotation[:, 2]
        z = float(normal @ self.origin) / (rays @ normal)
        return np.clip(np.round(z * 1000.0), 0, 65535).astype(np.uint16)
