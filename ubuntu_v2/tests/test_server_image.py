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
        self.assertIn("systemctl enable --now robot-control-server.service", installer)

    def test_server_service_starts_the_complete_compute_stack(self):
        service = (
            ROOT / "ubuntu_v2" / "systemd" / "robot-control-server.service"
        ).read_text()
        self.assertIn("--profile compute up -d gateway ros-transport compute-mapping", service)
        self.assertIn("After=docker.service network-online.target", service)

    def test_gateway_image_contains_server_map_payload_module(self):
        dockerfile = (ROOT / "ubuntu_v2" / "docker" / "Dockerfile.gateway").read_text()
        self.assertIn("robot_app/map_payload.py", dockerfile)

    def test_bundle_builder_does_not_copy_desktop_gui(self):
        builder = (ROOT / "server_image" / "build_server_image.sh").read_text()
        self.assertNotIn('cp -a "${PROJECT_DIR}/ubuntu_v2"', builder)
        self.assertIn('"${PROJECT_DIR}/ubuntu_v2/docker/."', builder)
        self.assertIn("robot-control-server-images.tar", builder)
        self.assertIn("COPYFILE_DISABLE=1 tar --no-xattrs --no-mac-metadata", builder)


if __name__ == "__main__":
    unittest.main()
