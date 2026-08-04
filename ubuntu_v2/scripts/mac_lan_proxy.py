#!/usr/bin/env python3
"""Expose a UTM Shared/NAT guest TCP service through the macOS LAN address."""

from __future__ import annotations

import argparse
import ipaddress
import socket
import socketserver
import threading
from dataclasses import dataclass


@dataclass(frozen=True)
class ProxyConfig:
    target_host: str
    target_port: int
    allowed_networks: tuple[ipaddress.IPv4Network | ipaddress.IPv6Network, ...]


class LanProxyServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True
    request_queue_size = 8

    def __init__(
        self,
        address: tuple[str, int],
        config: ProxyConfig,
    ) -> None:
        self.config = config
        super().__init__(address, RelayHandler)


class RelayHandler(socketserver.BaseRequestHandler):
    server: LanProxyServer

    def handle(self) -> None:
        client_ip = ipaddress.ip_address(self.client_address[0])
        allowed = self.server.config.allowed_networks
        if allowed and not any(client_ip in network for network in allowed):
            print(f"rejected client outside LAN: {client_ip}", flush=True)
            return

        target = (
            self.server.config.target_host,
            self.server.config.target_port,
        )
        try:
            upstream = socket.create_connection(target, timeout=3.0)
        except OSError as error:
            print(f"Ubuntu VM unavailable at {target[0]}:{target[1]}: {error}", flush=True)
            return

        print(f"client connected: {client_ip} -> {target[0]}:{target[1]}", flush=True)
        with upstream:
            upstream.settimeout(None)
            self.request.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            upstream.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            self.request.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            upstream.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

            reverse = threading.Thread(
                target=self._pump,
                args=(upstream, self.request),
                daemon=True,
            )
            reverse.start()
            self._pump(self.request, upstream)
            reverse.join(timeout=2.0)
        print(f"client disconnected: {client_ip}", flush=True)

    @staticmethod
    def _pump(source: socket.socket, destination: socket.socket) -> None:
        try:
            while chunk := source.recv(65_536):
                destination.sendall(chunk)
        except (ConnectionError, OSError):
            pass
        finally:
            try:
                destination.shutdown(socket.SHUT_WR)
            except OSError:
                pass


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--listen-host", default="0.0.0.0")
    parser.add_argument("--listen-port", type=int, default=9999)
    parser.add_argument("--target-host", required=True)
    parser.add_argument("--target-port", type=int, default=9999)
    parser.add_argument(
        "--allow-cidr",
        action="append",
        default=[],
        help="Client network allowed to use the proxy; may be repeated",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    networks = tuple(
        ipaddress.ip_network(value, strict=False) for value in args.allow_cidr
    )
    config = ProxyConfig(args.target_host, args.target_port, networks)
    with LanProxyServer((args.listen_host, args.listen_port), config) as server:
        allowed = ", ".join(str(network) for network in networks) or "all"
        print(
            f"LAN proxy ready: {args.listen_host}:{args.listen_port} -> "
            f"{args.target_host}:{args.target_port} (allowed: {allowed})",
            flush=True,
        )
        server.serve_forever(poll_interval=0.5)


if __name__ == "__main__":
    main()
