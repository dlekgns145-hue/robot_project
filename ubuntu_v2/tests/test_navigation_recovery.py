from __future__ import annotations

import importlib.util
import math
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import patch


class FakeNode:
    pass


def load_runtime_module(filename: str):
    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = FakeNode
    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.TransformStamped = type("TransformStamped", (), {})
    nav_msgs = types.ModuleType("nav_msgs")
    nav_msgs_msg = types.ModuleType("nav_msgs.msg")
    nav_msgs_msg.Odometry = type("Odometry", (), {})
    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.LaserScan = type("LaserScan", (), {})
    tf2_ros = types.ModuleType("tf2_ros")
    tf2_ros.TransformBroadcaster = type("TransformBroadcaster", (), {})
    stubs = {
        "rclpy": rclpy,
        "rclpy.node": rclpy_node,
        "geometry_msgs": geometry_msgs,
        "geometry_msgs.msg": geometry_msgs_msg,
        "nav_msgs": nav_msgs,
        "nav_msgs.msg": nav_msgs_msg,
        "sensor_msgs": sensor_msgs,
        "sensor_msgs.msg": sensor_msgs_msg,
        "tf2_ros": tf2_ros,
    }
    path = Path(__file__).resolve().parents[2] / "robot_docker" / filename
    spec = importlib.util.spec_from_file_location(f"{path.stem}_under_test", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, stubs):
        spec.loader.exec_module(module)
    return module


def sample_odometry(value: float = 0.0):
    vector = lambda: types.SimpleNamespace(x=value, y=value, z=value)
    orientation = types.SimpleNamespace(x=value, y=value, z=value, w=1.0)
    return types.SimpleNamespace(
        pose=types.SimpleNamespace(
            pose=types.SimpleNamespace(position=vector(), orientation=orientation)
        ),
        twist=types.SimpleNamespace(
            twist=types.SimpleNamespace(linear=vector(), angular=vector())
        ),
    )


class NavigationRecoveryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.odom_relay = load_runtime_module("odom_relay.py")
        cls.scan_filter = load_runtime_module("scan_time_fix.py")
        cls.scan_diagnostics = load_runtime_module("scan_diagnostics.py")

    def test_finite_odometry_is_accepted(self) -> None:
        self.assertTrue(self.odom_relay.odometry_is_finite(sample_odometry()))

    def test_nan_odometry_is_rejected(self) -> None:
        message = sample_odometry()
        message.pose.pose.position.x = math.nan
        self.assertFalse(self.odom_relay.odometry_is_finite(message))

    def test_measured_rear_chassis_reflection_is_filtered(self) -> None:
        self.assertTrue(
            self.scan_filter.is_self_reflection(math.radians(-150.0), 0.15)
        )

    def test_front_obstacle_is_not_filtered(self) -> None:
        self.assertFalse(self.scan_filter.is_self_reflection(0.0, 0.15))

    def test_distant_return_in_rear_sector_is_not_filtered(self) -> None:
        self.assertFalse(
            self.scan_filter.is_self_reflection(math.radians(-150.0), 0.5)
        )

    def test_slam_range_cap_removes_distant_reflection(self) -> None:
        filtered = self.scan_filter.sanitize_ranges(
            [1.0, 4.1],
            angle_min=0.0,
            angle_increment=0.1,
            range_min=0.12,
            range_max=8.0,
            maximum_range_m=4.0,
        )

        self.assertAlmostEqual(filtered[0], 1.0)
        self.assertTrue(math.isinf(filtered[1]))

    def test_slam_spatial_filter_removes_only_isolated_return(self) -> None:
        filtered = self.scan_filter.remove_spatial_speckles(
            [math.inf, 1.0, math.inf, 2.0, 2.03, math.inf],
            radius=1,
            tolerance=0.12,
        )

        self.assertTrue(math.isinf(filtered[1]))
        self.assertAlmostEqual(filtered[3], 2.0)
        self.assertAlmostEqual(filtered[4], 2.03)

    def test_slam_temporal_filter_rejects_one_frame_speckle(self) -> None:
        filter_ = self.scan_filter.TemporalMedianFilter(window=3, minimum_hits=2)

        first = filter_.update([1.0, math.inf])
        second = filter_.update([math.inf, 2.0])
        third = filter_.update([math.inf, 2.05])

        self.assertTrue(math.isinf(first[0]))
        self.assertTrue(math.isinf(second[0]))
        self.assertTrue(math.isinf(third[0]))
        self.assertAlmostEqual(third[1], 2.025)

    def test_slam_temporal_filter_rejects_inconsistent_repeated_noise(self) -> None:
        filter_ = self.scan_filter.TemporalMedianFilter(
            window=3, minimum_hits=2, tolerance=0.15
        )

        filter_.update([0.7])
        filter_.update([2.4])
        result = filter_.update([math.inf])

        self.assertTrue(math.isinf(result[0]))

    def test_slam_filter_warms_up_before_first_publish(self) -> None:
        filter_ = self.scan_filter.TemporalMedianFilter(window=3, minimum_hits=2)

        filter_.update([1.0])
        self.assertFalse(filter_.ready)
        filter_.update([1.02])
        self.assertTrue(filter_.ready)

    def test_scan_indices_are_grouped_into_contiguous_runs(self) -> None:
        self.assertEqual(
            self.scan_diagnostics.contiguous_runs([1, 2, 3, 8, 10, 11]),
            [(1, 3), (8, 8), (10, 11)],
        )

    def test_mapping_runtime_uses_isolated_tf_and_fixed_sensors(self) -> None:
        docker_dir = Path(__file__).resolve().parents[2] / "robot_docker"
        launch_text = (docker_dir / "mapping_runtime_launch.py").read_text()
        dockerfile_text = (docker_dir / "Dockerfile").read_text()
        entrypoint_text = (docker_dir / "entrypoint.sh").read_text()
        compose_text = (docker_dir / "compose.yaml").read_text()
        params_text = (docker_dir / "mapping_slam_params.yaml").read_text()
        nav_params_text = (
            docker_dir / "recovered" / "dwb_nav_params_fixed.yaml"
        ).read_text()

        self.assertIn('SetRemap(src="/tf", dst="/tf_nav")', launch_text)
        self.assertIn(
            'SetRemap(src="/cmd_vel", dst="/cmd_vel_server")', launch_text
        )
        self.assertIn('("/tf", "/tf_nav")', launch_text)
        self.assertIn("scan_time_fix.py", launch_text)
        self.assertIn("odom_relay.py", launch_text)
        self.assertIn('name="base_footprint_to_base_link"', launch_text)
        self.assertIn("navigation_launch.py", launch_text)
        self.assertIn("autonomous_mapping.py", launch_text)
        self.assertIn("map_texture_recorder.py", launch_text)
        self.assertIn("map_texture_core.py", dockerfile_text)
        self.assertIn("obstacle_texture_fusion.py", dockerfile_text)
        self.assertIn("calibrate_map_texture.py", dockerfile_text)
        self.assertIn('LaunchConfiguration("camera_horizontal_fov_deg")', launch_text)
        self.assertIn('"obstacle_layer_render_period"', launch_text)
        self.assertIn('"sensor_sync_maximum_skew"', launch_text)
        self.assertIn("lidar_x_offset_m:=", launch_text)
        self.assertIn("ROBOT_SENSOR_SYNC_MAXIMUM_SKEW", entrypoint_text)
        self.assertIn('LaunchConfiguration("texture_far_m")', launch_text)
        self.assertIn("projection_far_width_m:=", launch_text)
        odom_command = launch_text.split('f"{runtime_dir}/odom_relay.py"', 1)[1]
        odom_command, texture_command = odom_command.split(
            'f"{runtime_dir}/map_texture_recorder.py"', 1
        )
        self.assertNotIn("projection_near_m:=", odom_command)
        self.assertIn("projection_near_m:=", texture_command)
        self.assertIn("ROBOT_TEXTURE_FAR_M", entrypoint_text)
        self.assertIn("ROBOT_MAPPING_MAXIMUM_RUNTIME", entrypoint_text)
        self.assertIn("maximum_exploration_radius:=", launch_text)
        self.assertIn("ROBOT_MAPPING_MAXIMUM_RADIUS", compose_text)
        self.assertIn("ROBOT_TEXTURE_SOURCE_TOP_FRACTION", compose_text)
        camera_guard_command = launch_text.split(
            'f"{runtime_dir}/camera_obstacle_guard.py"', 1
        )[1].split('f"{runtime_dir}/odom_relay.py"', 1)[0]
        self.assertNotIn("source_top_fraction:=", camera_guard_command)
        self.assertIn("camera_obstacle_guard.py", launch_text)
        self.assertIn("IfCondition(camera_guard_enabled)", launch_text)
        self.assertIn("output_topic:=/scan_slam", launch_text)
        self.assertIn("temporal_window:=3", launch_text)
        self.assertIn("maximum_range_m:=4.0", launch_text)
        self.assertIn("RewrittenYaml", launch_text)
        self.assertIn('{"yaw_goal_tolerance": "3.14"}', launch_text)
        self.assertIn('default_value="false"', launch_text)
        self.assertIn("scan_topic: /scan_slam", params_text)
        self.assertIn("odom_frame: odom", params_text)
        self.assertIn("base_frame: base_footprint", params_text)
        self.assertIn("observation_sources: scan camera", nav_params_text)
        self.assertIn("topic: /camera_scan", nav_params_text)
        self.assertIn("default_server_timeout: 1000", nav_params_text)
        self.assertIn("min_speed_theta: 0.0", nav_params_text)
        self.assertIn("required_movement_radius: 0.05", nav_params_text)
        self.assertIn("movement_time_allowance: 45.0", nav_params_text)
        self.assertIn("bt_loop_duration: 100", nav_params_text)
        self.assertIn("transform_timeout: 0.05", params_text)
        self.assertIn("throttle_scans: 1", params_text)

    def test_navigation_runtime_loads_saved_map_for_reboot(self) -> None:
        docker_dir = Path(__file__).resolve().parents[2] / "robot_docker"
        launch_text = (docker_dir / "navigation_runtime_launch.py").read_text()
        compose_text = (docker_dir / "compose.yaml").read_text()
        entrypoint_text = (docker_dir / "entrypoint.sh").read_text()
        params_text = (
            docker_dir / "recovered" / "dwb_nav_params_fixed.yaml"
        ).read_text()

        self.assertIn("/opt/robot-control/maps/orchard_map.yaml", launch_text)
        self.assertIn('"autostart": "true"', launch_text)
        self.assertIn("camera_obstacle_guard.py", launch_text)
        self.assertIn("navigation-runtime:", compose_text)
        self.assertIn('profiles: [navigation]', compose_text)
        self.assertIn('command: ["navigation"]', compose_text)
        self.assertIn("ROBOT_MAP_YAML", entrypoint_text)
        self.assertIn("set_initial_pose: True", params_text)
        self.assertIn("x: -0.468", params_text)
        self.assertIn("y: 0.508", params_text)
        self.assertIn("min_vel_x: 0.0", params_text)
        self.assertNotIn("min_vel_x: -", params_text)
        self.assertIn("prepare_navigation_params.py", entrypoint_text)
        self.assertIn("last_pose.json", entrypoint_text)

    def test_last_localized_pose_replaces_template_fallback(self) -> None:
        module = load_runtime_module("prepare_navigation_params.py")
        docker_dir = Path(__file__).resolve().parents[2] / "robot_docker"
        template = (
            docker_dir / "recovered" / "dwb_nav_params_fixed.yaml"
        ).read_text()

        rendered = module.render_params(
            template, {"x": -0.6405, "y": 0.0329, "yaw": -2.9346}
        )

        self.assertIn("      x: -0.640500", rendered)
        self.assertIn("      y: 0.032900", rendered)
        self.assertIn("      yaw: -2.934600", rendered)
        self.assertIn("min_vel_x: 0.0", rendered)

    def test_navigation_run_owns_and_releases_motor_control(self) -> None:
        docker_dir = Path(__file__).resolve().parents[2] / "robot_docker"
        run_text = (docker_dir / "navigation_run_test.py").read_text()

        self.assertIn("/cmd_bridge/navigation_mode", run_text)
        self.assertIn("/cmd_bridge/emergency_stop", run_text)
        self.assertIn("self._set_navigation_mode(True)", run_text)
        self.assertIn("self._set_navigation_mode(False)", run_text)
        self.assertIn("cancel_goal_async()", run_text)


if __name__ == "__main__":
    unittest.main()
