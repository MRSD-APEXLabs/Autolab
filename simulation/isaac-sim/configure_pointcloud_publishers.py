#!/usr/bin/env python3
"""Add idempotent ROS 2 point-cloud publishers to the scene action graph.

Run this with Isaac Sim's USD Python environment while Isaac Sim is stopped.
Each point-cloud helper reuses the corresponding left camera render product.
"""

import argparse

from pxr import Sdf, Usd


PUBLISHERS = (
    (
        "/World/ActionGraph/publish_zed_x_left_depth",
        "/World/ActionGraph/publish_zed_x_left_pointcloud",
    ),
    (
        "/World/ActionGraph/publish_zed_x_nano_left_depth",
        "/World/ActionGraph/publish_zed_x_nano_left_pointcloud",
    ),
)


def configure(stage_path: str) -> None:
    stage = Usd.Stage.Open(stage_path)
    if stage is None:
        raise RuntimeError(f"Could not open USD stage: {stage_path}")

    root_layer = stage.GetRootLayer()
    for source_path, destination_path in PUBLISHERS:
        source = stage.GetPrimAtPath(source_path)
        if not source.IsValid():
            raise RuntimeError(f"Missing source camera helper: {source_path}")

        if stage.GetPrimAtPath(destination_path).IsValid():
            stage.RemovePrim(destination_path)

        if not Sdf.CopySpec(root_layer, source_path, root_layer, destination_path):
            raise RuntimeError(f"Could not copy {source_path} to {destination_path}")

        publisher = stage.GetPrimAtPath(destination_path)
        publisher.GetAttribute("inputs:type").Set("depth_pcl")
        publisher.GetAttribute("inputs:topicName").Set("pointcloud")

    root_layer.Save()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("stage", help="USD stage to update")
    configure(parser.parse_args().stage)
