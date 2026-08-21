#!/usr/bin/env python3
"""Save the robot's current localized AMCL pose as the active map's home.

Run this only while the robot is physically parked at the desired home position.
The token is sent to the gateway but is never printed.
"""

from __future__ import annotations

import argparse
import getpass
import json
import os
import socket


def home_set_payload(token: str, robot_ip_hint: str) -> dict:
    payload = {"type": "home_set"}
    if token:
        payload["token"] = token
    if robot_ip_hint:
        payload["robot_ip_hint"] = robot_ip_hint
    return payload


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--gateway", required=True, help="Ubuntu gateway IP/hostname")
    parser.add_argument("--port", default=9999, type=int)
    parser.add_argument(
        "--token-env",
        default="COMMAND_TOKEN",
        help="environment variable to read; prompts securely when unset",
    )
    parser.add_argument("--robot", default="raspberrypi.local")
    parser.add_argument("--timeout", default=8.0, type=float)
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        raise SystemExit("port must be between 1 and 65535")

    token = os.getenv(args.token_env, "")
    if not token:
        token = getpass.getpass("Gateway control token: ")
    payload = home_set_payload(token, args.robot)
    encoded = (json.dumps(payload, separators=(",", ":")) + "\n").encode()
    with socket.create_connection((args.gateway, args.port), timeout=args.timeout) as connection:
        connection.settimeout(args.timeout)
        connection.sendall(encoded)
        response = bytearray()
        while b"\n" not in response:
            chunk = connection.recv(4096)
            if not chunk:
                raise SystemExit("gateway closed before returning a response")
            response.extend(chunk)
            if len(response) > 1_048_576:
                raise SystemExit("gateway response exceeded 1 MiB")
    result = json.loads(bytes(response).splitlines()[0])
    if result.get("type") == "error" or result.get("robot_error"):
        raise SystemExit(str(result.get("message") or result.get("robot_error")))
    print("home_set request accepted; verify home_pose.json on the robot")


if __name__ == "__main__":
    main()
