"""Browser UI for the Xavier camera hub: switch between the ZED X and the ZED X Nano, see RGB and depth.

`frames` (newest frame per stream, rendering, depth probe) and `web` (HTTP server and page) are
plain Python and importable without ROS; `ui_node` is the ROS 2 node that feeds them.
"""
