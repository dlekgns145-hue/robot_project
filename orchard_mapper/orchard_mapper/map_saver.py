"""Atomic visual-map storage and an optional periodic ROS service client."""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from .coordinate_transform import MapGeometry


def save_visual_map(
    output_path: str,
    image: np.ndarray,
    weight: np.ndarray,
    geometry: MapGeometry,
    *,
    metadata: dict[str, Any] | None = None,
) -> dict[str, str]:
    base = Path(output_path)
    base.parent.mkdir(parents=True, exist_ok=True)
    png_path = base.with_suffix(".png")
    yaml_path = base.with_suffix(".yaml")
    weight_path = base.parent / f"{base.name}_weight.npy"
    metadata_path = base.parent / f"{base.name}_metadata.json"

    ok, encoded = cv2.imencode(".png", image)
    if not ok:
        raise OSError("visual map PNG encoding failed")
    png_tmp = png_path.with_suffix(".png.tmp")
    with open(png_tmp, "wb") as output:
        output.write(encoded.tobytes())
        output.flush()
        os.fsync(output.fileno())
    os.replace(png_tmp, png_path)

    weight_tmp = weight_path.with_suffix(".npy.tmp")
    with open(weight_tmp, "wb") as output:
        np.save(output, weight.astype(np.float32), allow_pickle=False)
        output.flush()
        os.fsync(output.fileno())
    os.replace(weight_tmp, weight_path)

    yaml_text = (
        f"image: {png_path.name}\n"
        f"resolution: {geometry.resolution:.12g}\n"
        f"origin: [{geometry.origin_x:.12g}, {geometry.origin_y:.12g}, 0.0]\n"
        f"width: {geometry.width}\n"
        f"height: {geometry.height}\n"
        "frame_id: map\n"
        "pixel_convention: north_up\n"
    )
    yaml_tmp = yaml_path.with_suffix(".yaml.tmp")
    yaml_tmp.write_text(yaml_text, encoding="utf-8")
    os.replace(yaml_tmp, yaml_path)

    payload = dict(metadata or {})
    payload.update(
        {
            "resolution": geometry.resolution,
            "origin": [geometry.origin_x, geometry.origin_y, 0.0],
            "width": geometry.width,
            "height": geometry.height,
            "observed_pixels": int(np.count_nonzero(weight > 0.0)),
        }
    )
    metadata_tmp = metadata_path.with_suffix(".json.tmp")
    metadata_tmp.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    os.replace(metadata_tmp, metadata_path)
    return {
        "image": str(png_path),
        "yaml": str(yaml_path),
        "weight": str(weight_path),
        "metadata": str(metadata_path),
    }


def main() -> None:
    import rclpy
    from rclpy.node import Node
    from std_srvs.srv import Trigger

    class VisualMapSaverNode(Node):
        def __init__(self) -> None:
            super().__init__("visual_map_saver")
            self.declare_parameter("save_service", "/orchard_visual_mapper/save")
            self.declare_parameter("save_interval", 0.0)
            self.client = self.create_client(
                Trigger, str(self.get_parameter("save_service").value)
            )
            interval = float(self.get_parameter("save_interval").value)
            if interval > 0.0:
                self.create_timer(interval, self._request_save)
            self.create_service(Trigger, "/orchard_visual_mapper/save_now", self._save)

        def _request_save(self) -> None:
            if self.client.service_is_ready():
                self.client.call_async(Trigger.Request())

        def _save(self, _request, response):
            if not self.client.wait_for_service(timeout_sec=1.0):
                response.success = False
                response.message = "visual mapper save service is unavailable"
                return response
            self.client.call_async(Trigger.Request())
            response.success = True
            response.message = "visual map save requested"
            return response

    rclpy.init()
    node = VisualMapSaverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
