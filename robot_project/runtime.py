"""Execution-mode rules shared by the integrated ROS runner and its tests."""

from __future__ import annotations


VALID_MODES = ("perception", "follow", "navigation")


def components_for_mode(mode: str) -> tuple[str, ...]:
    """Return the ROS components that are safe to run for *mode*.

    Follow owns ``/cmd_vel`` directly, while Nav2 also controls robot motion.
    They are deliberately mutually exclusive so two controllers cannot fight.
    """

    normalized = mode.strip().lower()
    if normalized == "perception":
        return ("perception",)
    if normalized == "follow":
        return ("perception", "follow")
    if normalized == "navigation":
        return ("navigation",)
    raise ValueError(
        f"unsupported mode: {mode!r}; choose one of {', '.join(VALID_MODES)}"
    )
