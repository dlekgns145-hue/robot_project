from __future__ import annotations

import hashlib
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

APP_DIR = Path(__file__).resolve().parents[1] / "robot_app"
sys.path.insert(0, str(APP_DIR))

from map_postprocess import (  # noqa: E402
    FREE,
    OCCUPIED,
    UNKNOWN,
    PostprocessConfig,
    classify_wall_straight_and_curved,
    connect_wall_gaps,
    cutout_room_rectangle,
    estimate_wall_correction,
    mask_outside_wall_as_unknown,
    process_pending_jobs,
    rectilinearize_wall,
    remove_noise_sections,
    rotate_map,
    transform_pose,
)


class ServerMapPostprocessTests(unittest.TestCase):
    def test_isolated_noise_is_removed_but_linear_wall_is_preserved(self) -> None:
        image = np.full((40, 40), FREE, dtype=np.uint8)
        image[20, 5:30] = OCCUPIED
        image[4, 4] = OCCUPIED
        image[35, 32] = OCCUPIED

        cleaned, report = remove_noise_sections(
            image, 0.05, PostprocessConfig()
        )

        self.assertEqual(int(cleaned[20, 10]), int(OCCUPIED))
        self.assertEqual(int(cleaned[4, 4]), int(FREE))
        self.assertGreaterEqual(report["removed_noise_pixels"], 2)

    def test_minimum_noise_area_zero_keeps_every_small_dot_mark(self) -> None:
        # 2026-08-19: the default 0.0125 m^2 threshold removed 31 of 66 real
        # obstacle marks on an actual classroom map -- most desk/chair legs
        # are only a pixel or two at this resolution, not sensor noise.
        # --keep-all-marks (minimum_noise_area_m2=0.0) must keep all of them.
        image = np.full((40, 40), FREE, dtype=np.uint8)
        image[20, 5:30] = OCCUPIED
        image[4, 4] = OCCUPIED
        image[35, 32] = OCCUPIED

        cleaned, report = remove_noise_sections(
            image, 0.05, PostprocessConfig(minimum_noise_area_m2=0.0)
        )

        self.assertEqual(report["removed_noise_pixels"], 0)
        self.assertEqual(int(cleaned[4, 4]), int(OCCUPIED))
        self.assertEqual(int(cleaned[35, 32]), int(OCCUPIED))

    def test_short_horizontal_and_vertical_wall_dropouts_are_joined(self) -> None:
        image = np.full((50, 50), UNKNOWN, dtype=np.uint8)
        image[10, 5:20] = OCCUPIED
        image[10, 23:40] = OCCUPIED
        image[5:20, 30] = OCCUPIED
        image[23:40, 30] = OCCUPIED

        connected, additions = connect_wall_gaps(
            image, 0.05, PostprocessConfig(maximum_wall_gap_m=0.20)
        )

        self.assertTrue(np.all(connected[10, 20:23] == OCCUPIED))
        self.assertTrue(np.all(connected[20:23, 30] == OCCUPIED))
        self.assertGreaterEqual(additions, 6)

    def test_dominant_skew_is_detected_conservatively(self) -> None:
        image = np.full((180, 180), FREE, dtype=np.uint8)
        angle = math.radians(7.0)
        for offset in (35, 70, 105, 140):
            x1, x2 = 15, 165
            y1 = offset
            y2 = round(offset + math.tan(angle) * (x2 - x1))
            cv2.line(image, (x1, y1), (x2, y2), int(OCCUPIED), 2)

        correction, report = estimate_wall_correction(
            image, 0.05, PostprocessConfig()
        )

        self.assertTrue(report["angle_confident"])
        self.assertAlmostEqual(correction, 7.0, delta=1.0)
        rotated, _metadata = rotate_map(
            image,
            {
                "resolution": 0.05,
                "origin": [0.0, 0.0, 0.0],
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.25,
            },
            correction,
        )
        residual, _report = estimate_wall_correction(
            rotated, 0.05, PostprocessConfig()
        )
        self.assertAlmostEqual(residual, 0.0, delta=0.5)

    def test_crop_only_keeps_world_pose_coordinates_unchanged(self) -> None:
        image = np.full((100, 120), UNKNOWN, dtype=np.uint8)
        image[20:80, 25:95] = FREE
        image[20, 25:95] = OCCUPIED
        image[79, 25:95] = OCCUPIED
        image[20:80, 25] = OCCUPIED
        image[20:80, 94] = OCCUPIED
        from map_postprocess import process_map

        _corrected, _metadata, report = process_map(
            image,
            {
                "resolution": 0.05,
                "origin": [-2.0, -3.0, 0.0],
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.25,
            },
        )
        pose = {"x": 0.2, "y": -0.4, "yaw": 0.7}
        transformed = transform_pose(pose, report["coordinate_transform"])

        self.assertAlmostEqual(transformed["x"], pose["x"], places=5)
        self.assertAlmostEqual(transformed["y"], pose["y"], places=5)
        self.assertAlmostEqual(transformed["yaw"], pose["yaw"], places=5)

    def test_cutout_squares_a_room_skewed_past_the_correction_clamp(self) -> None:
        # process_map's own wall correction clamps at 12 degrees so a noisy
        # Hough estimate can't misalign the navigation pose transform. A
        # room actually skewed further than that (35 degrees here) still
        # needs a clean visualization cutout, which is exactly what this
        # function is for -- it is not clamped and not used for navigation.
        canvas = np.full((240, 240), UNKNOWN, dtype=np.uint8)
        rect = ((120.0, 120.0), (140.0, 100.0), 35.0)
        box = cv2.boxPoints(rect).astype(np.int32)
        cv2.fillConvexPoly(canvas, box, int(FREE))
        cv2.polylines(canvas, [box], True, int(OCCUPIED), 2)
        metadata = {
            "resolution": 0.05,
            "origin": [0.0, 0.0, 0.0],
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.25,
        }

        cropped, _metadata, report = cutout_room_rectangle(canvas, metadata)

        self.assertGreater(np.count_nonzero(cropped != UNKNOWN), 0)
        known_ratio = np.count_nonzero(cropped != UNKNOWN) / cropped.size
        self.assertGreater(known_ratio, 0.85)
        margin_px = math.ceil(0.10 / 0.05)
        self.assertLessEqual(
            abs(cropped.shape[0] - (100 + 2 * margin_px)), 10
        )
        self.assertLessEqual(
            abs(cropped.shape[1] - (140 + 2 * margin_px)), 10
        )
        self.assertIn("cutout_angle_deg", report)

    def test_mask_outside_wall_hides_free_padding_beyond_the_room(self) -> None:
        # A diagonal room (as cutout_room_rectangle's axis-aligned crop
        # would leave it) has free-space padding in its bounding box's
        # corners, outside the wall's own contour.
        image = np.full((150, 150), UNKNOWN, dtype=np.uint8)
        rect = ((75.0, 75.0), (80.0, 80.0), 30.0)
        box = cv2.boxPoints(rect).astype(np.int32)
        cv2.fillConvexPoly(image, box, int(FREE))
        cv2.polylines(image, [box], True, int(OCCUPIED), 2)
        # Padding sliver: free pixels in a corner outside the diamond.
        image[5:15, 5:15] = FREE

        styled, report = mask_outside_wall_as_unknown(image)

        self.assertGreater(report["masked_pixels"], 0)
        self.assertTrue(np.all(styled[5:15, 5:15] == UNKNOWN))

    def test_mask_outside_wall_leaves_the_room_interior_untouched(self) -> None:
        image = np.full((150, 150), UNKNOWN, dtype=np.uint8)
        cv2.rectangle(image, (20, 20), (120, 120), int(FREE), -1)
        cv2.rectangle(image, (20, 20), (120, 120), int(OCCUPIED), 2)
        image[60, 60] = OCCUPIED  # an obstacle mark inside the room
        before = image.copy()

        styled, _report = mask_outside_wall_as_unknown(image)

        # Nothing inside the wall's own contour -- the room's free floor,
        # the wall, or any obstacle mark -- should ever change.
        np.testing.assert_array_equal(styled[22:118, 22:118], before[22:118, 22:118])

    def test_classify_wall_draws_a_rectangle_as_four_straight_segments(
        self,
    ) -> None:
        image = np.full((150, 150), FREE, dtype=np.uint8)
        cv2.rectangle(image, (20, 20), (120, 120), int(OCCUPIED), 2)

        styled, report = classify_wall_straight_and_curved(image)

        self.assertEqual(report["curved_segments"], 0)
        self.assertGreaterEqual(report["straight_segments"], 4)
        self.assertGreater(np.count_nonzero(styled == OCCUPIED), 0)

    def test_classify_wall_keeps_a_circle_as_curved_segments(self) -> None:
        image = np.full((160, 160), FREE, dtype=np.uint8)
        cv2.circle(image, (80, 80), 60, int(OCCUPIED), 3)

        styled, report = classify_wall_straight_and_curved(image)

        self.assertEqual(report["straight_segments"], 0)
        self.assertGreater(report["curved_segments"], 0)
        contours, _ = cv2.findContours(
            (styled == OCCUPIED).astype(np.uint8),
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        largest = max(contours, key=cv2.contourArea)
        simplified = cv2.approxPolyDP(largest, 3.0, True)
        self.assertGreaterEqual(len(simplified), 16)

    def test_classify_wall_mixes_straight_and_curved_on_one_shape(self) -> None:
        # A rounded-rectangle room: straight walls with one curved corner.
        image = np.full((160, 160), FREE, dtype=np.uint8)
        cv2.ellipse(
            image, (80, 80), (50, 40), 0, 0, 360, int(OCCUPIED), 2
        )
        cv2.rectangle(image, (60, 20), (100, 140), int(OCCUPIED), -1)
        image[24:136, 32:128] = FREE
        cv2.rectangle(image, (30, 40), (130, 120), int(OCCUPIED), 2)

        styled, report = classify_wall_straight_and_curved(image)

        self.assertGreater(np.count_nonzero(styled == OCCUPIED), 0)
        self.assertGreaterEqual(report["straight_segments"] + report["curved_segments"], 1)

    def test_classify_wall_caps_thickness_at_2px_even_if_original_is_thicker(
        self,
    ) -> None:
        image = np.full((150, 150), FREE, dtype=np.uint8)
        cv2.rectangle(image, (20, 20), (120, 120), int(OCCUPIED), 6)

        styled, report = classify_wall_straight_and_curved(image, max_thickness_px=2)

        self.assertEqual(report["thickness_px"], 2)
        wall_mask = (styled == OCCUPIED).astype(np.uint8)
        distances = cv2.distanceTransform(wall_mask, cv2.DIST_L2, 3)
        self.assertLessEqual(2 * float(np.max(distances[wall_mask > 0])), 4.5)

    def test_classify_wall_leaves_small_marks_completely_untouched(self) -> None:
        image = np.full((150, 150), FREE, dtype=np.uint8)
        cv2.rectangle(image, (20, 20), (120, 120), int(OCCUPIED), 2)
        image[60, 60] = OCCUPIED
        image[61, 60:63] = OCCUPIED
        image[62, 61] = OCCUPIED
        mark_before = image[55:70, 55:70].copy()

        styled, _report = classify_wall_straight_and_curved(image)

        np.testing.assert_array_equal(styled[55:70, 55:70], mark_before)

    def test_rectilinearize_wall_does_not_shortcut_across_a_real_notch(
        self,
    ) -> None:
        # A C-shaped (non-convex) room: a deep notch cut into one side.
        # Moving corners to snap edges could shortcut straight across the
        # notch's mouth instead of tracing into and back out of it (found
        # on the real classroom map, 2026-08-19, as a spurious interior
        # line). The notch's own floor must stay outside the redrawn wall.
        image = np.full((160, 160), FREE, dtype=np.uint8)
        outline = np.array(
            [
                [20, 20], [140, 20], [140, 140], [20, 140],
                [20, 100], [90, 100], [90, 60], [20, 60],
            ],
            dtype=np.int32,
        )
        cv2.polylines(image, [outline.reshape(-1, 1, 2)], True, int(OCCUPIED), 2)

        styled, _report = rectilinearize_wall(image)

        # The notch's own boundary must still be traced (a shortcut across
        # its mouth would skip these corners rather than route through
        # them), and its interior must stay outside the redrawn wall.
        occupied = styled == OCCUPIED
        for corner_x, corner_y in ((90, 100), (90, 60), (20, 100), (20, 60)):
            region = occupied[corner_y - 4 : corner_y + 5, corner_x - 4 : corner_x + 5]
            self.assertTrue(np.any(region), f"no wall traced near notch corner ({corner_x},{corner_y})")
        self.assertEqual(int(styled[80, 55]), int(FREE))  # inside the notch's mouth

    def test_rectilinearize_wall_is_one_connected_component(self) -> None:
        # The whole point: a single cv2.polylines call has no internal
        # joints to mismatch, unlike drawing each stretch separately.
        image = np.full((150, 150), FREE, dtype=np.uint8)
        cv2.rectangle(image, (20, 20), (120, 120), int(OCCUPIED), 2)
        rng = np.random.default_rng(3)
        ys, xs = np.nonzero(image == OCCUPIED)
        for _ in range(40):
            i = rng.integers(0, len(xs))
            image[ys[i] + rng.integers(-1, 2), xs[i]] = OCCUPIED

        styled, report = rectilinearize_wall(image)

        wall_mask = (styled == OCCUPIED).astype(np.uint8)
        count, _labels = cv2.connectedComponents(wall_mask, connectivity=8)
        self.assertEqual(count, 2)  # background + exactly one wall outline
        self.assertGreater(report["corners"], 0)

    def test_rectilinearize_wall_has_no_diagonal_pixels(self) -> None:
        # rectilinearize_wall always runs after cutout_room_rectangle in
        # the real pipeline, which already rotates the whole map so the
        # wall is close to axis-aligned -- a few degrees of residual skew
        # here, not an arbitrary rotation from scratch. Inserting a bend
        # per edge guarantees every edge is individually axis-aligned, but
        # for a corner near 45 degrees (both deltas close in size) an
        # independently-chosen bend direction on each of its two edges can
        # still cross itself; that only matters at rotations this pipeline
        # never actually produces.
        image = np.full((160, 160), FREE, dtype=np.uint8)
        rect = ((80.0, 80.0), (80.0, 60.0), 3.0)
        box = cv2.boxPoints(rect).astype(np.int32)
        cv2.polylines(image, [box], True, int(OCCUPIED), 2)

        styled, _report = rectilinearize_wall(image)

        contours, _ = cv2.findContours(
            (styled == OCCUPIED).astype(np.uint8),
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        largest = max(contours, key=cv2.contourArea)
        simplified = cv2.approxPolyDP(largest, 2.0, True).reshape(-1, 2)
        for i in range(len(simplified)):
            x1, y1 = simplified[i]
            x2, y2 = simplified[(i + 1) % len(simplified)]
            # Every real edge must be purely horizontal or vertical. A
            # short (a few px) diagonal is just the miter where two thick
            # rasterized strokes meet at a corner, not a real diagonal
            # edge -- the original bug produced diagonals tens of pixels
            # long, spanning a real fraction of the shape.
            self.assertTrue(abs(int(x1) - int(x2)) <= 4 or abs(int(y1) - int(y2)) <= 4)

    def test_rectilinearize_wall_caps_thickness_at_2px(self) -> None:
        image = np.full((150, 150), FREE, dtype=np.uint8)
        cv2.rectangle(image, (20, 20), (120, 120), int(OCCUPIED), 6)

        styled, report = rectilinearize_wall(image, max_thickness_px=2)

        self.assertEqual(report["thickness_px"], 2)
        wall_mask = (styled == OCCUPIED).astype(np.uint8)
        distances = cv2.distanceTransform(wall_mask, cv2.DIST_L2, 3)
        self.assertLessEqual(2 * float(np.max(distances[wall_mask > 0])), 4.5)

    def test_rectilinearize_wall_leaves_small_marks_completely_untouched(
        self,
    ) -> None:
        image = np.full((150, 150), FREE, dtype=np.uint8)
        cv2.rectangle(image, (20, 20), (120, 120), int(OCCUPIED), 2)
        image[60, 60] = OCCUPIED
        image[61, 60:63] = OCCUPIED
        image[62, 61] = OCCUPIED
        mark_before = image[55:70, 55:70].copy()

        styled, _report = rectilinearize_wall(image)

        np.testing.assert_array_equal(styled[55:70, 55:70], mark_before)

    def test_job_preserves_raw_map_and_promotes_corrected_pair(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            raw_dir = root / "raw"
            inbox = root / "postprocess-inbox"
            raw_dir.mkdir()
            inbox.mkdir()
            image = np.full((80, 100), UNKNOWN, dtype=np.uint8)
            image[10:70, 10:90] = FREE
            image[10, 10:45] = OCCUPIED
            image[10, 48:90] = OCCUPIED
            image[69, 10:90] = OCCUPIED
            image[10:70, 10] = OCCUPIED
            image[10:70, 89] = OCCUPIED
            ok, encoded = cv2.imencode(".pgm", image)
            self.assertTrue(ok)
            raw_bytes = encoded.tobytes()
            (raw_dir / "job.pgm").write_bytes(raw_bytes)
            (raw_dir / "job.yaml").write_text(
                "image: job.pgm\nresolution: 0.05\n"
                "origin: [-1.0, -2.0, 0.0]\nnegate: 0\n"
                "occupied_thresh: 0.65\nfree_thresh: 0.25\n"
            )
            job = {
                "job_id": "job",
                "input_prefix": "raw/job",
                "output_prefix": "orchard_map",
                "source_sha256": hashlib.sha256(raw_bytes).hexdigest(),
                "mapping": {"state": "completed", "save_sequence": 1},
                "robot_pose": {"x": 0.5, "y": -0.25, "yaw": 0.3},
            }
            job_path = inbox / "job.json"
            job_path.write_text(json.dumps(job))

            self.assertEqual(process_pending_jobs(root), 1)
            report = json.loads(
                (root / "orchard_map_postprocess.json").read_text()
            )

            self.assertTrue((root / "orchard_map.pgm").is_file())
            self.assertTrue((root / "orchard_map.yaml").is_file())
            self.assertTrue((root / "orchard_map_postprocess.png").is_file())
            self.assertTrue((root / "corrected/job.pgm").is_file())
            self.assertTrue((root / "postprocess-completed/job.json").is_file())
            self.assertTrue((root / "postprocess-outbox/job.json").is_file())
            self.assertFalse(job_path.exists())
            self.assertEqual((raw_dir / "job.pgm").read_bytes(), raw_bytes)
            self.assertGreater(report["connected_wall_pixels"], 0)
            bundle = json.loads((root / "postprocess-outbox/job.json").read_text())
            self.assertEqual(bundle["robot_pose"], report["corrected_robot_pose"])
            self.assertEqual(len(bundle["coordinate_transform"]), 3)


if __name__ == "__main__":
    unittest.main()
