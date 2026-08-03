"""Resolve a robot's changing IPv4 address from a stable identity."""

from __future__ import annotations

import ipaddress
import re
import socket
import subprocess
import threading
import time
from dataclasses import dataclass


MAC_PATTERN = re.compile(r"^(?:[0-9a-f]{2}:){5}[0-9a-f]{2}$")


def normalize_mac(value: str) -> str:
    normalized = value.strip().lower().replace("-", ":")
    if normalized and not MAC_PATTERN.fullmatch(normalized):
        raise ValueError(f"invalid ROBOT_MAC: {value!r}")
    return normalized


def valid_ipv4(value: str) -> str:
    address = ipaddress.ip_address(value.strip())
    if address.version != 4:
        raise ValueError(f"not an IPv4 address: {value!r}")
    return str(address)


def parse_neighbor_table(output: str, expected_mac: str) -> str | None:
    """Parse `ip -4 neigh` output and return a reachable matching address."""
    expected_mac = normalize_mac(expected_mac)
    for raw_line in output.splitlines():
        fields = raw_line.lower().split()
        if not fields or "lladdr" not in fields or expected_mac not in fields:
            continue
        if any(state in fields for state in ("failed", "incomplete")):
            continue
        try:
            return valid_ipv4(fields[0])
        except ValueError:
            continue
    return None


def parse_arp_scan(output: str, expected_mac: str) -> str | None:
    """Parse `arp-scan` output, which starts matching rows with IP and MAC."""
    expected_mac = normalize_mac(expected_mac)
    for raw_line in output.splitlines():
        fields = raw_line.lower().split()
        if len(fields) < 2 or fields[1].replace("-", ":") != expected_mac:
            continue
        try:
            return valid_ipv4(fields[0])
        except ValueError:
            continue
    return None


@dataclass(frozen=True)
class Resolution:
    ip: str
    method: str


class RobotLocator:
    """Cache and refresh the robot address without treating DHCP IP as identity."""

    def __init__(
        self,
        *,
        robot_mac: str = "",
        robot_host: str = "",
        robot_ip: str = "",
        interface: str = "",
        discovery_cidr: str = "",
        cache_seconds: float = 30.0,
        command_timeout: float = 8.0,
    ) -> None:
        self.robot_mac = normalize_mac(robot_mac)
        self.robot_host = robot_host.strip()
        self.robot_ip = valid_ipv4(robot_ip) if robot_ip.strip() else ""
        self.interface = interface.strip()
        self.discovery_cidr = discovery_cidr.strip()
        self.cache_seconds = max(0.0, cache_seconds)
        self.command_timeout = max(1.0, command_timeout)
        self._cached: Resolution | None = None
        self._cached_at = 0.0
        self._runtime_ip = ""
        self._lock = threading.Lock()

    def set_runtime_ip(self, value: str, method: str = "runtime-hint") -> None:
        """Accept an authenticated GUI-side hostname result for NAT environments."""
        address = valid_ipv4(value)
        with self._lock:
            self._runtime_ip = address
            self._cached = Resolution(address, method)
            self._cached_at = time.monotonic()

    def invalidate(self, *, clear_runtime: bool = True) -> None:
        with self._lock:
            self._cached = None
            self._cached_at = 0.0
            if clear_runtime:
                self._runtime_ip = ""

    def resolve(self, *, force: bool = False) -> Resolution:
        with self._lock:
            now = time.monotonic()
            if (
                not force
                and self._cached is not None
                and now - self._cached_at <= self.cache_seconds
            ):
                return self._cached

            resolution = self._resolve_uncached()
            self._cached = resolution
            self._cached_at = now
            return resolution

    def _run(self, args: list[str]) -> str:
        result = subprocess.run(
            args,
            check=False,
            capture_output=True,
            text=True,
            timeout=self.command_timeout,
        )
        return result.stdout

    def _default_interface(self) -> str:
        if self.interface:
            return self.interface
        output = self._run(["ip", "-4", "route", "show", "default"])
        for line in output.splitlines():
            fields = line.split()
            if "dev" in fields:
                index = fields.index("dev")
                if index + 1 < len(fields):
                    return fields[index + 1]
        return ""

    def _resolve_by_mac(self) -> Resolution | None:
        if not self.robot_mac:
            return None

        neighbor_output = self._run(["ip", "-4", "neigh", "show"])
        address = parse_neighbor_table(neighbor_output, self.robot_mac)
        if address:
            return Resolution(address, "mac-neighbor")

        interface = self._default_interface()
        command = ["arp-scan"]
        if interface:
            command.extend(["--interface", interface])
        if self.discovery_cidr:
            command.append(self.discovery_cidr)
        else:
            command.append("--localnet")
        scan_output = self._run(command)
        address = parse_arp_scan(scan_output, self.robot_mac)
        if address:
            return Resolution(address, "mac-arp-scan")
        return None

    def _resolve_by_hostname(self) -> Resolution | None:
        if not self.robot_host:
            return None
        addresses = socket.getaddrinfo(
            self.robot_host,
            None,
            family=socket.AF_INET,
            type=socket.SOCK_STREAM,
        )
        if not addresses:
            return None
        return Resolution(valid_ipv4(addresses[0][4][0]), "hostname")

    def _resolve_uncached(self) -> Resolution:
        if self.robot_ip:
            return Resolution(self.robot_ip, "fixed-ip")

        if self._runtime_ip:
            return Resolution(self._runtime_ip, "gui-hostname")

        mac_resolution = self._resolve_by_mac()
        if mac_resolution:
            return mac_resolution

        try:
            hostname_resolution = self._resolve_by_hostname()
        except socket.gaierror:
            hostname_resolution = None
        if hostname_resolution:
            return hostname_resolution

        identities = []
        if self.robot_mac:
            identities.append(f"MAC {self.robot_mac}")
        if self.robot_host:
            identities.append(f"hostname {self.robot_host}")
        if not identities:
            raise RuntimeError("set ROBOT_MAC, ROBOT_HOST, or ROBOT_IP")
        raise RuntimeError("robot not found using " + " or ".join(identities))
