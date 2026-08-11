import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[2]


class ServerImageTests(unittest.TestCase):
    def test_installer_defaults_to_verified_robot_identity(self):
        installer = (ROOT / "server_image" / "INSTALL_SERVER.sh").read_text()
        self.assertIn('ROBOT_IP="172.30.1.10"', installer)
        self.assertIn('ROBOT_MAC="2c:cf:67:7b:48:d7"', installer)
        self.assertIn("22.04|24.04|26.04", installer)
        self.assertIn('SOURCE_DIR="${SCRIPT_DIR}"', installer)
        self.assertIn('SOURCE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"', installer)
        self.assertIn("docker compose --profile compute build gateway compute-mapping", installer)
        self.assertIn("systemctl enable robot-control-server.service", installer)
        self.assertIn("systemctl restart robot-control-server.service", installer)
        self.assertIn('chown root:docker "${ENV_FILE}"', installer)
        self.assertIn('chmod 0640 "${ENV_FILE}"', installer)

    def test_server_service_starts_the_complete_compute_stack(self):
        service = (
            ROOT / "ubuntu_v2" / "systemd" / "robot-control-server.service"
        ).read_text()
        self.assertIn("scripts/run_compute_mapping.sh", service)
        self.assertIn("After=docker.service network-online.target", service)

        runner = (
            ROOT / "ubuntu_v2" / "scripts" / "run_compute_mapping.sh"
        ).read_text()
        self.assertIn(
            "--profile compute up -d gateway ros-transport compute-mapping", runner
        )
        self.assertIn("/navigation_scan_filter", runner)
        self.assertIn("docker restart robot-v2-ros-transport", runner)

    def test_gateway_image_contains_server_map_payload_module(self):
        dockerfile = (ROOT / "ubuntu_v2" / "docker" / "Dockerfile.gateway").read_text()
        self.assertIn("robot_app/map_payload.py", dockerfile)

    def test_compute_image_uses_reachable_ros_mirror(self):
        dockerfile = (ROOT / "ubuntu_v2" / "docker" / "Dockerfile.compute").read_text()
        self.assertIn("ROS_APT_MIRROR", dockerfile)
        self.assertIn("mirrors.aliyun.com/ros2/ubuntu", dockerfile)
        self.assertIn("Acquire::ForceIPv4=true", dockerfile)

    def test_compute_image_is_lidar_mapping_only(self):
        dockerfile = (ROOT / "ubuntu_v2" / "docker" / "Dockerfile.compute").read_text()
        self.assertIn("robot_docker/mapping_core.py", dockerfile)
        self.assertIn("ros-humble-slam-toolbox", dockerfile)
        self.assertNotIn("COPY orchard_mapper", dockerfile)
        self.assertNotIn("colcon build --merge-install", dockerfile)
        self.assertNotIn("map_texture_recorder.py", dockerfile)

    def test_robot_transport_uses_dedicated_compute_port(self):
        server_compose = (ROOT / "ubuntu_v2" / "compose.yaml").read_text()
        robot_compose = (ROOT / "robot_docker" / "compose.yaml").read_text()
        server_transport = (
            ROOT / "ubuntu_v2" / "zenoh" / "server-transport.json5"
        ).read_text()
        robot_transport = (
            ROOT / "robot_docker" / "zenoh" / "robot-transport.json5"
        ).read_text()

        self.assertIn("server-transport.json5", server_compose)
        self.assertIn("robot-transport.json5", robot_compose)
        self.assertIn("172.30.1.10:7448", server_transport)
        self.assertIn("0.0.0.0:7448", robot_transport)
        self.assertIn('mode: "client"', server_transport)
        self.assertIn('mode: "router"', robot_transport)
        self.assertIn("allow:", server_transport)
        self.assertIn('"/cmd_vel_server"', server_transport)
        self.assertNotIn('"/scan_fixed"', server_transport)
        self.assertIn('"/scan"', robot_transport)
        self.assertNotIn('"/cmd_vel"', robot_transport)
        self.assertIn('"/autonomous_mapping/(start|stop|save|preview)"', robot_transport)
        self.assertIn('"/navigate_to_pose"', server_transport)
        self.assertIn("ros_localhost_only: true", robot_transport)
        self.assertIn('ROS_LOCALHOST_ONLY: "1"', robot_compose)
        self.assertGreaterEqual(
            server_compose.count("FASTDDS_BUILTIN_TRANSPORTS: UDPv4"), 2
        )

    def test_mapping_server_is_lidar_only(self):
        entrypoint = (ROOT / "robot_docker" / "entrypoint.sh").read_text()
        launch_file = (ROOT / "robot_docker" / "mapping_runtime_launch.py").read_text()
        self.assertNotIn("visual_mapper_enabled", entrypoint)
        self.assertNotIn('package="orchard_mapper"', launch_file)
        self.assertNotIn("map_texture_recorder.py", launch_file)
        self.assertIn('"observation_sources": "scan"', launch_file)

    def test_bundle_builder_does_not_copy_desktop_gui(self):
        builder = (ROOT / "server_image" / "build_server_image.sh").read_text()
        self.assertNotIn('cp -a "${PROJECT_DIR}/ubuntu_v2"', builder)
        self.assertIn('"${PROJECT_DIR}/ubuntu_v2/docker/."', builder)
        self.assertIn("check_server_environment.sh", builder)
        self.assertNotIn("configure_camera_source.sh", builder)
        self.assertNotIn("phone_camera_layer_test.py", builder)
        self.assertNotIn("phone_scene_capture.py", builder)
        self.assertIn("robot-control-server-images.tar", builder)
        self.assertNotIn('"${PROJECT_DIR}/orchard_mapper/."', builder)
        self.assertIn('"${PROJECT_DIR}/robot_docker/mapping_core.py"', builder)
        self.assertIn("COPYFILE_DISABLE=1 tar --no-xattrs --no-mac-metadata", builder)

    def test_installer_omits_visual_mapping_build_context(self):
        installer = (ROOT / "server_image" / "INSTALL_SERVER.sh").read_text()
        self.assertNotIn('"${SOURCE_DIR}/orchard_mapper/."', installer)
        self.assertNotIn('"${INSTALL_DIR}/orchard_mapper/"', installer)
        self.assertNotIn("map_texture_recorder.py", installer)


if __name__ == "__main__":
    unittest.main()
