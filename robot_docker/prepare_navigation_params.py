#!/usr/bin/env python3
"""Inject the last localized AMCL pose into a Nav2 parameter template."""

from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path


POSE_FIELDS = ("x", "y", "yaw")


def validated_pose(payload: dict) -> dict[str, float]:
    pose = {field: float(payload[field]) for field in POSE_FIELDS}
    if not all(math.isfinite(value) for value in pose.values()):
        raise ValueError("last pose contains a non-finite value")
    return pose


def render_params(template: str, pose: dict[str, float]) -> str:
    block_pattern = re.compile(
        r"(?P<prefix>^    initial_pose:\n)"
        r"(?P<body>(?:^      [^\n]*\n)+)",
        re.MULTILINE,
    )
    match = block_pattern.search(template)
    if match is None:
        raise ValueError("AMCL initial_pose block not found")
    body = match.group("body")
    for field in POSE_FIELDS:
        body, count = re.subn(
            rf"(?m)^(      {field}:)\s*[^\n]+$",
            rf"\1 {pose[field]:.6f}",
            body,
            count=1,
        )
        if count != 1:
            raise ValueError(f"AMCL initial_pose.{field} not found")
    return template[: match.start("body")] + body + template[match.end("body") :]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--template", required=True)
    parser.add_argument("--pose", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    template_path = Path(args.template)
    pose_path = Path(args.pose)
    output_path = Path(args.output)
    template = template_path.read_text(encoding="utf-8")
    if pose_path.is_file():
        pose = validated_pose(json.loads(pose_path.read_text(encoding="utf-8")))
        rendered = render_params(template, pose)
        print(
            "using persisted AMCL pose: "
            f"x={pose['x']:.3f}, y={pose['y']:.3f}, yaw={pose['yaw']:.3f}",
            flush=True,
        )
    else:
        rendered = template
        print("last AMCL pose not found; using template fallback", flush=True)
    output_path.write_text(rendered, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
