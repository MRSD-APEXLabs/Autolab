"""ZED X Nano and ZED X feeds from the Xavier's camera hub (or the older Nano-only stream server).

`stream` (HTTP clients) and `calibration` (Stereolabs factory calibration, stereo rectification,
CameraInfo fields) are plain Python and importable without ROS; `camera_node` (one camera's
feed) and `hub_node` (which camera the hub streams) are the ROS 2 nodes.
"""
