from __future__ import annotations

import socket
import sys
import threading
import unittest
from ipaddress import ip_network
from pathlib import Path

SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from mac_lan_proxy import LanProxyServer, ProxyConfig  # noqa: E402


class MacLanProxyTests(unittest.TestCase):
    def test_allowed_lan_client_is_relayed_to_vm_target(self) -> None:
        target = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        target.bind(("127.0.0.1", 0))
        target.listen(1)

        def echo_once() -> None:
            connection, _ = target.accept()
            with connection:
                connection.sendall(connection.recv(1024))

        echo_thread = threading.Thread(target=echo_once)
        echo_thread.start()
        config = ProxyConfig(
            "127.0.0.1",
            target.getsockname()[1],
            (ip_network("127.0.0.0/8"),),
        )
        with LanProxyServer(("127.0.0.1", 0), config) as proxy:
            proxy_thread = threading.Thread(target=proxy.serve_forever)
            proxy_thread.start()
            with socket.create_connection(proxy.server_address, timeout=2) as client:
                client.sendall(b"robot-control")
                self.assertEqual(client.recv(1024), b"robot-control")
            proxy.shutdown()
            proxy_thread.join(timeout=2)

        echo_thread.join(timeout=2)
        target.close()


if __name__ == "__main__":
    unittest.main()
