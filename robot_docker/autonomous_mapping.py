#!/usr/bin/env python3
"""Safely explore occupancy-grid frontiers and save the completed map."""

from __future__ import annotations

import json
import math
from dataclasses import asdict

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import SaveMap
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from frontier_core import GridSpec, frontier_candidates


class AutonomousMappingNode(Node):
    def __init__(self) -> None:
        super().__init__("autonomous_mapping")
        self.declare_parameter("start_enabled", False)
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("decision_period", 1.0)
        self.declare_parameter("startup_delay", 8.0)
        self.declare_parameter("goal_timeout", 90.0)
        self.declare_parameter("minimum_runtime", 60.0)
        self.declare_parameter("maximum_runtime", 1800.0)
        self.declare_parameter("completion_stable_maps", 5)
        self.declare_parameter("minimum_frontier_length", 0.35)
        self.declare_parameter("minimum_goal_distance", 0.45)
        self.declare_parameter("maximum_goal_distance", 7.0)
        self.declare_parameter("frontier_goal_standoff", 0.10)
        self.declare_parameter("maximum_exploration_radius", 25.0)
        self.declare_parameter("blacklist_radius", 0.7)
        self.declare_parameter("failed_goal_blacklist_seconds", 300.0)
        self.declare_parameter("reached_goal_blacklist_seconds", 12.0)
        self.declare_parameter("map_output", "/opt/robot-control/maps/orchard_map")

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        map_topic = str(self.get_parameter("map_topic").value)

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_subscription = self.create_subscription(
            OccupancyGrid, map_topic, self._on_map, map_qos
        )
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_publisher = self.create_publisher(
            String, "/autonomous_mapping/status", status_qos
        )
        self.goal_publisher = self.create_publisher(
            PoseStamped, "/autonomous_mapping/goal", 10
        )
        self.stop_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.navigator = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.map_saver = self.create_client(SaveMap, "/map_saver/save_map")
        self.command_mode = self.create_client(
            SetBool, "/cmd_bridge/navigation_mode"
        )
        self.emergency_subscription = self.create_subscription(
            Bool,
            "/cmd_bridge/emergency_stop",
            self._on_emergency_stop,
            10,
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_service(Trigger, "/autonomous_mapping/start", self._start)
        self.create_service(Trigger, "/autonomous_mapping/stop", self._stop)
        self.create_service(Trigger, "/autonomous_mapping/save", self._save)
        self.create_service(Trigger, "/autonomous_mapping/preview", self._preview)
        self.timer = self.create_timer(
            float(self.get_parameter("decision_period").value), self._tick
        )

        self.latest_map: OccupancyGrid | None = None
        self.map_sequence = 0
        self.last_empty_map_sequence = -1
        self.empty_map_count = 0
        self.enabled = bool(self.get_parameter("start_enabled").value)
        self.state = "starting" if self.enabled else "idle"
        self.started_at = self._seconds() if self.enabled else 0.0
        self.start_pose: tuple[float, float] | None = None
        self.goal_candidate = None
        self.goal_handle = None
        self.goal_request_pending = False
        self.goal_sent_at = 0.0
        self.distance_remaining = None
        self.blacklist: list[tuple[float, float, float]] = []
        self.save_pending = False
        self.save_requested_reason = ""
        self.saving_map_output = ""
        self.saved_map = ""
        self._publish_status("autonomous mapping ready; waiting for start")

    def _seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _on_map(self, message: OccupancyGrid) -> None:
        self.latest_map = message
        self.map_sequence += 1

    def _robot_pose(self) -> tuple[float, float]:
        transform = self.tf_buffer.lookup_transform(
            self.map_frame, self.base_frame, Time(), timeout=Duration(seconds=0.2)
        )
        translation = transform.transform.translation
        return float(translation.x), float(translation.y)

    def _start(self, _request: Trigger.Request, response: Trigger.Response):
        if self.enabled:
            response.success = False
            response.message = "autonomous mapping is already active"
            return response
        try:
            self.start_pose = self._robot_pose()
        except TransformException as error:
            response.success = False
            response.message = f"map transform is not ready: {error}"
            return response
        if not self.command_mode.service_is_ready():
            response.success = False
            response.message = "command bridge navigation lock is unavailable"
            return response
        self._set_navigation_mode(True)
        self.enabled = True
        self.state = "starting"
        self.started_at = self._seconds()
        self.empty_map_count = 0
        self.last_empty_map_sequence = -1
        self.blacklist.clear()
        self.saved_map = ""
        response.success = True
        response.message = "autonomous mapping started"
        self._publish_status(response.message)
        return response

    def _stop(self, _request: Trigger.Request, response: Trigger.Response):
        was_active = self.enabled or self.goal_handle is not None
        self._stop_exploration("stopped by operator")
        response.success = was_active
        response.message = "autonomous mapping stopped"
        return response

    def _save(self, _request: Trigger.Request, response: Trigger.Response):
        map_output = str(self.get_parameter("map_output").value)
        if self._request_map_save("manual save requested"):
            response.success = True
            response.message = f"saving map to {map_output}"
        else:
            response.success = False
            response.message = "map saver is unavailable or already busy"
        return response

    def _preview(self, _request: Trigger.Request, response: Trigger.Response):
        try:
            _robot_x, _robot_y, candidates = self._find_candidates()
        except (RuntimeError, TransformException, ValueError) as error:
            response.success = False
            response.message = str(error)
            return response
        payload = {"candidate_count": len(candidates)}
        if candidates:
            payload["best"] = asdict(candidates[0])
        response.success = True
        response.message = json.dumps(payload, separators=(",", ":"))
        return response

    def _publish_zero(self) -> None:
        stop = Twist()
        for _ in range(5):
            self.stop_publisher.publish(stop)

    def _set_navigation_mode(self, enabled: bool) -> None:
        if not self.command_mode.service_is_ready():
            self.get_logger().error("command bridge navigation lock is unavailable")
            return
        request = SetBool.Request()
        request.data = enabled
        future = self.command_mode.call_async(request)
        future.add_done_callback(self._on_navigation_mode_changed)

    def _on_navigation_mode_changed(self, future) -> None:
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(f"navigation lock request failed: {error}")
            return
        if not response.success:
            self.get_logger().error(
                f"navigation lock request rejected: {response.message}"
            )

    def _on_emergency_stop(self, message: Bool) -> None:
        if message.data and (self.enabled or self.goal_handle is not None):
            self._stop_exploration("emergency stop received from command bridge")

    def _stop_exploration(self, reason: str) -> None:
        self.enabled = False
        self.state = "stopped"
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.goal_handle = None
        self.goal_request_pending = False
        self.goal_candidate = None
        self._publish_zero()
        self._set_navigation_mode(False)
        self._publish_status(reason)

    def _prune_blacklist(self, now: float) -> list[tuple[float, float]]:
        self.blacklist = [entry for entry in self.blacklist if entry[2] > now]
        return [(entry[0], entry[1]) for entry in self.blacklist]

    def _map_spec(self, message: OccupancyGrid) -> GridSpec:
        orientation = message.info.origin.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        return GridSpec(
            width=int(message.info.width),
            height=int(message.info.height),
            resolution=float(message.info.resolution),
            origin_x=float(message.info.origin.position.x),
            origin_y=float(message.info.origin.position.y),
            origin_yaw=yaw,
        )

    def _find_candidates(self):
        if self.latest_map is None:
            raise RuntimeError("occupancy grid is not ready")
        robot_x, robot_y = self._robot_pose()
        spec = self._map_spec(self.latest_map)
        min_length = float(self.get_parameter("minimum_frontier_length").value)
        min_cells = max(2, math.ceil(min_length / spec.resolution))
        candidates = frontier_candidates(
            self.latest_map.data,
            spec,
            robot_x=robot_x,
            robot_y=robot_y,
            min_cells=min_cells,
            min_distance=float(self.get_parameter("minimum_goal_distance").value),
            max_distance=float(self.get_parameter("maximum_goal_distance").value),
            blacklisted=self._prune_blacklist(self._seconds()),
            blacklist_radius=float(self.get_parameter("blacklist_radius").value),
            goal_standoff=float(
                self.get_parameter("frontier_goal_standoff").value
            ),
        )
        start_x, start_y = self.start_pose or (robot_x, robot_y)
        max_radius = float(self.get_parameter("maximum_exploration_radius").value)
        candidates = [
            candidate
            for candidate in candidates
            if max_radius <= 0.0
            or math.hypot(candidate.x - start_x, candidate.y - start_y) <= max_radius
        ]
        return robot_x, robot_y, candidates

    def _tick(self) -> None:
        if self.save_requested_reason and not self.save_pending:
            pending_reason = self.save_requested_reason
            self._request_map_save(pending_reason)
        if not self.enabled:
            return
        now = self._seconds()
        elapsed = now - self.started_at
        maximum_runtime = float(self.get_parameter("maximum_runtime").value)
        if maximum_runtime > 0.0 and elapsed >= maximum_runtime:
            self._stop_exploration("maximum mapping time reached")
            self._request_map_save("saving partial map after time limit")
            return

        if self.goal_handle is not None:
            timeout = float(self.get_parameter("goal_timeout").value)
            if now - self.goal_sent_at >= timeout:
                self.get_logger().warn("frontier goal timed out; cancelling")
                self.goal_handle.cancel_goal_async()
            return
        if self.goal_request_pending:
            return
        if elapsed < float(self.get_parameter("startup_delay").value):
            return
        if self.latest_map is None:
            self.state = "waiting_for_map"
            self._publish_status("waiting for occupancy grid")
            return
        if not self.navigator.server_is_ready():
            self.state = "waiting_for_nav2"
            self._publish_status("waiting for Nav2 action server")
            return

        try:
            robot_x, robot_y, candidates = self._find_candidates()
        except (TransformException, ValueError) as error:
            self.state = "waiting_for_tf"
            self._publish_status(f"waiting for robot pose: {error}")
            return
        if self.start_pose is None:
            self.start_pose = (robot_x, robot_y)

        if not candidates:
            if self.last_empty_map_sequence != self.map_sequence:
                self.empty_map_count += 1
                self.last_empty_map_sequence = self.map_sequence
            self.state = "checking_completion"
            minimum_runtime = float(self.get_parameter("minimum_runtime").value)
            stable_maps = int(self.get_parameter("completion_stable_maps").value)
            if elapsed >= minimum_runtime and self.empty_map_count >= stable_maps:
                self.enabled = False
                self.state = "completed"
                self._publish_zero()
                self._set_navigation_mode(False)
                self._publish_status("exploration complete")
                self._request_map_save("saving completed map")
            else:
                self._publish_status("no usable frontier; checking completion")
            return

        self.empty_map_count = 0
        self.last_empty_map_sequence = -1
        self._send_goal(candidates[0], robot_x, robot_y, len(candidates))

    def _send_goal(self, candidate, robot_x: float, robot_y: float, count: int) -> None:
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = candidate.x
        goal.pose.pose.position.y = candidate.y
        yaw = math.atan2(candidate.y - robot_y, candidate.x - robot_x)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.goal_candidate = candidate
        self.goal_request_pending = True
        self.goal_sent_at = self._seconds()
        self.distance_remaining = candidate.distance
        self.state = "sending_goal"
        self.goal_publisher.publish(goal.pose)
        self._publish_status(f"selected frontier from {count} candidates")
        future = self.navigator.send_goal_async(
            goal, feedback_callback=self._on_feedback
        )
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        self.goal_request_pending = False
        try:
            handle = future.result()
        except Exception as error:  # rclpy future transports exceptions here
            self._blacklist_current(True)
            self.state = "goal_error"
            self._publish_status(f"goal request failed: {error}")
            return
        if not handle.accepted:
            self._blacklist_current(True)
            self.state = "goal_rejected"
            self._publish_status("frontier goal rejected")
            return
        self.goal_handle = handle
        if not self.enabled:
            handle.cancel_goal_async()
            return
        self.state = "navigating"
        self._publish_status("frontier goal accepted")
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_feedback(self, feedback_message) -> None:
        self.distance_remaining = float(feedback_message.feedback.distance_remaining)

    def _blacklist_current(self, failed: bool) -> None:
        if self.goal_candidate is None:
            return
        lifetime_name = (
            "failed_goal_blacklist_seconds"
            if failed
            else "reached_goal_blacklist_seconds"
        )
        expires = self._seconds() + float(self.get_parameter(lifetime_name).value)
        self.blacklist.append(
            (self.goal_candidate.x, self.goal_candidate.y, expires)
        )
        self.goal_candidate = None

    def _on_goal_result(self, future) -> None:
        self.goal_handle = None
        try:
            status = int(future.result().status)
        except Exception as error:
            self._blacklist_current(True)
            self.state = "goal_error"
            self._publish_status(f"goal result failed: {error}")
            return
        succeeded = status == GoalStatus.STATUS_SUCCEEDED
        self._blacklist_current(not succeeded)
        self.distance_remaining = None
        self.state = "goal_reached" if succeeded else "goal_failed"
        self._publish_status(
            "frontier reached" if succeeded else f"frontier failed with status={status}"
        )

    def _request_map_save(self, reason: str) -> bool:
        if self.save_pending:
            return False
        if not self.map_saver.service_is_ready():
            self.save_requested_reason = reason
            self._publish_status("map saver unavailable; save queued")
            return True
        self.save_requested_reason = ""
        request = SaveMap.Request()
        map_output = str(self.get_parameter("map_output").value)
        request.map_topic = str(self.get_parameter("map_topic").value)
        request.map_url = map_output
        request.image_format = "pgm"
        request.map_mode = "trinary"
        request.free_thresh = 0.25
        request.occupied_thresh = 0.65
        self.save_pending = True
        self.saving_map_output = map_output
        self._publish_status(reason)
        future = self.map_saver.call_async(request)
        future.add_done_callback(self._on_map_saved)
        return True

    def _on_map_saved(self, future) -> None:
        self.save_pending = False
        try:
            saved = bool(future.result().result)
        except Exception as error:
            saved = False
            self._publish_status(f"map save failed: {error}")
        if saved:
            self.saved_map = self.saving_map_output
            self._publish_status(f"map saved: {self.saving_map_output}")
        else:
            self._publish_status("map saver reported failure")

    def _publish_status(self, message: str) -> None:
        payload = {
            "state": self.state,
            "enabled": self.enabled,
            "message": message,
            "map_sequence": self.map_sequence,
            "empty_map_count": self.empty_map_count,
            "distance_remaining": self.distance_remaining,
            "saved_map": self.saved_map,
        }
        if self.goal_candidate is not None:
            payload["goal"] = asdict(self.goal_candidate)
        status = String()
        status.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.status_publisher.publish(status)
        self.get_logger().info(message)


def main() -> None:
    rclpy.init()
    node = AutonomousMappingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._stop_exploration("process interrupted")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
