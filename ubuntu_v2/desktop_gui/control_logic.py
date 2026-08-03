"""Pure follow-control functions shared by the desktop vision worker."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class FollowSettings:
    linear_speed: float = 0.35
    angular_speed: float = 0.4
    stop_height_ratio: float = 0.55
    servo_pan_max_angle: int = 60
    servo_max_step: int = 6
    servo_deadzone_angle: int = 25
    align_threshold: int = 20
    lost_frame_limit: int = 10
    grace_frames: int = 3
    search_angular_speed: float = 0.3
    search_direction: int = 1


def servo_target(center_x: float, frame_width: int, settings: FollowSettings) -> int:
    half_width = frame_width / 2.0
    offset = max(-1.0, min(1.0, (center_x - half_width) / half_width))
    target = int(offset * settings.servo_pan_max_angle)
    return 0 if abs(target) <= settings.servo_deadzone_angle else target


def smooth_servo(current: int, target: int | None, max_step: int) -> int:
    if target is None:
        return current
    difference = max(-max_step, min(max_step, target - current))
    return current + difference


def movement(
    servo_pan: int,
    box_height: float,
    frame_height: int,
    settings: FollowSettings,
) -> tuple[float, float, str]:
    if abs(servo_pan) > settings.align_threshold:
        angular = -settings.angular_speed if servo_pan > 0 else settings.angular_speed
        return 0.0, angular, "ALIGN"
    if box_height / frame_height >= settings.stop_height_ratio:
        return 0.0, 0.0, "STOP"
    return settings.linear_speed, 0.0, "FORWARD"
