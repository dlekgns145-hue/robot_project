from pathlib import Path

import cv2
import numpy as np

from orchard_mapper.coordinate_transform import BevGeometry, MapGeometry, Pose2D
from orchard_mapper.frame_database import FrameDatabase
from orchard_mapper.image_blender import WeightedCanvas
from orchard_mapper.map_saver import save_visual_map


def test_blend_and_preserve_when_canvas_origin_expands():
    canvas = WeightedCanvas(MapGeometry(0.1, -2.0, -2.0, 40, 40))
    image = np.full((10, 10, 3), (20, 80, 140), dtype=np.uint8)
    mask = np.full((10, 10), 255, dtype=np.uint8)
    count = canvas.blend(
        image,
        mask,
        Pose2D(0.0, 0.0, 0.0),
        BevGeometry(10, 10, 1.0, 0.5),
        feather_pixels=0,
    )
    assert count > 0
    before_image, before_weight = canvas.snapshot()
    assert np.count_nonzero(before_weight) > 0
    canvas.reconfigure(MapGeometry(0.1, -3.0, -3.0, 60, 60))
    after_image, after_weight = canvas.snapshot()
    assert np.count_nonzero(after_weight) == np.count_nonzero(before_weight)
    assert np.count_nonzero(after_image) == np.count_nonzero(before_image)


def test_frame_database_round_trip(tmp_path: Path):
    database = FrameDatabase(str(tmp_path / "database"))
    image = np.full((8, 9, 3), 73, np.uint8)
    mask = np.full((8, 9), 255, np.uint8)
    frame_id = database.add(
        1234, image, mask, Pose2D(1.0, 2.0, 0.3), Pose2D(0.5, 0.7, 0.1)
    )
    assert frame_id == 1
    records = database.records()
    assert len(records) == 1
    restored_image, restored_mask = database.load(records[0])
    np.testing.assert_array_equal(restored_image, image)
    np.testing.assert_array_equal(restored_mask, mask)
    database.close()


def test_visual_map_files(tmp_path: Path):
    image = np.full((4, 5, 3), 128, np.uint8)
    weight = np.ones((4, 5), np.float32)
    geometry = MapGeometry(0.02, -1.0, -2.0, 5, 4)
    paths = save_visual_map(
        str(tmp_path / "orchard_visual_map"), image, weight, geometry
    )
    assert cv2.imread(paths["image"]).shape == image.shape
    yaml_text = Path(paths["yaml"]).read_text(encoding="utf-8")
    assert "resolution: 0.02" in yaml_text
    assert "origin: [-1, -2, 0.0]" in yaml_text
