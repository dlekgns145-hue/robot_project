"""Place camera BEV frames in ROS map coordinates and maintain a mosaic."""

from __future__ import annotations

import json
import math
from pathlib import Path
import threading

from cv_bridge import CvBridge
import message_filters
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .coordinate_transform import (
    BevGeometry,
    MapGeometry,
    Pose2D,
    compose_pose,
    pose_delta,
    pose_from_transform,
)
from .frame_database import FrameDatabase
from .image_blender import WeightedCanvas
from .map_saver import save_visual_map


class GlobalVisualMapper(Node):
    def __init__(self) -> None:
        super().__init__("orchard_visual_mapper")
        self._declare_parameters()
        self.bridge = CvBridge()
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.bev_geometry = BevGeometry(
            width=int(self.get_parameter("bev_width").value),
            height=int(self.get_parameter("bev_height").value),
            forward_range=float(self.get_parameter("bev_forward_range").value),
            side_range=float(self.get_parameter("bev_side_range").value),
        )
        resolution = float(self.get_parameter("map_resolution").value)
        initial_geometry = MapGeometry(
            resolution=resolution,
            origin_x=float(self.get_parameter("map_origin_x").value),
            origin_y=float(self.get_parameter("map_origin_y").value),
            width=max(
                1,
                math.ceil(float(self.get_parameter("map_width_m").value) / resolution),
            ),
            height=max(
                1,
                math.ceil(float(self.get_parameter("map_height_m").value) / resolution),
            ),
        )
        self.canvas = WeightedCanvas(initial_geometry)
        self.lock = threading.RLock()
        self.last_pose: Pose2D | None = None
        self.accepted_frames = 0
        self.last_rerender_source = "live"
        self.tf_buffer = Buffer(cache_time=Duration(seconds=120.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.database = (
            FrameDatabase(str(self.get_parameter("frame_db_path").value))
            if bool(self.get_parameter("store_frames").value)
            else None
        )

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("occupancy_map_topic").value),
            self._on_occupancy_map,
            map_qos,
        )
        image_subscriber = message_filters.Subscriber(
            self,
            Image,
            str(self.get_parameter("bev_image_topic").value),
            qos_profile=qos_profile_sensor_data,
        )
        mask_subscriber = message_filters.Subscriber(
            self,
            Image,
            str(self.get_parameter("bev_mask_topic").value),
            qos_profile=qos_profile_sensor_data,
        )
        synchronizer = message_filters.ApproximateTimeSynchronizer(
            [image_subscriber, mask_subscriber], queue_size=10, slop=0.08
        )
        synchronizer.registerCallback(self._on_bev)
        self._sync = synchronizer

        self.map_publisher = self.create_publisher(
            Image, str(self.get_parameter("visual_map_topic").value), 1
        )
        publish_fps = max(float(self.get_parameter("publish_fps").value), 0.1)
        self.create_timer(1.0 / publish_fps, self._publish_map)
        autosave = float(self.get_parameter("autosave_interval").value)
        if autosave > 0.0:
            self.create_timer(autosave, self._save_silent)
        self.create_service(Trigger, "~/save", self._save_service)
        self.create_service(Trigger, "~/rerender", self._rerender_service)
        self.create_service(Trigger, "~/reset", self._reset_service)

    def _declare_parameters(self) -> None:
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("occupancy_map_topic", "/map")
        self.declare_parameter("bev_image_topic", "/camera_bev/bev/image")
        self.declare_parameter("bev_mask_topic", "/camera_bev/bev/mask")
        self.declare_parameter("visual_map_topic", "/orchard_visual_map/image")
        self.declare_parameter("bev_width", 480)
        self.declare_parameter("bev_height", 640)
        self.declare_parameter("bev_forward_range", 2.5)
        self.declare_parameter("bev_side_range", 1.2)
        self.declare_parameter("map_resolution", 0.02)
        self.declare_parameter("map_origin_x", -20.0)
        self.declare_parameter("map_origin_y", -20.0)
        self.declare_parameter("map_width_m", 40.0)
        self.declare_parameter("map_height_m", 40.0)
        self.declare_parameter("follow_occupancy_geometry", True)
        self.declare_parameter("min_translation", 0.10)
        self.declare_parameter("min_rotation", 0.0872665)
        self.declare_parameter("accept_rotation_only", False)
        self.declare_parameter("tf_timeout", 0.15)
        self.declare_parameter("allow_latest_tf_fallback", True)
        self.declare_parameter("feather_pixels", 24)
        self.declare_parameter("observation_weight", 1.0)
        self.declare_parameter("publish_fps", 1.0)
        self.declare_parameter("autosave_interval", 30.0)
        self.declare_parameter(
            "output_path", "/opt/robot-control/maps/orchard_visual_map"
        )
        self.declare_parameter(
            "frame_db_path", "/opt/robot-control/maps/orchard_visual_frames"
        )
        self.declare_parameter("store_frames", True)
        self.declare_parameter("corrected_trajectory_path", "")

    def _on_occupancy_map(self, message: OccupancyGrid) -> None:
        if not bool(self.get_parameter("follow_occupancy_geometry").value):
            return
        resolution = self.canvas.geometry.resolution
        width_m = float(message.info.width) * float(message.info.resolution)
        height_m = float(message.info.height) * float(message.info.resolution)
        geometry = MapGeometry(
            resolution=resolution,
            origin_x=float(message.info.origin.position.x),
            origin_y=float(message.info.origin.position.y),
            width=max(1, math.ceil(width_m / resolution)),
            height=max(1, math.ceil(height_m / resolution)),
        )
        with self.lock:
            self.canvas.reconfigure(geometry)

    def _lookup_pose(self, target: str, stamp: Time) -> Pose2D:
        timeout = Duration(
            seconds=float(self.get_parameter("tf_timeout").value)
        )
        try:
            transform = self.tf_buffer.lookup_transform(
                target, self.base_frame, stamp, timeout=timeout
            )
        except TransformException:
            if not bool(
                self.get_parameter("allow_latest_tf_fallback").value
            ):
                raise
            # MJPEG carries no camera timestamp. Its arrival stamp can be a few
            # milliseconds newer than the latest distributed TF, so use the
            # most recent pose rather than dropping an otherwise valid frame.
            transform = self.tf_buffer.lookup_transform(
                target, self.base_frame, Time(), timeout=timeout
            )
        return pose_from_transform(transform)

    def _on_bev(self, image_message: Image, mask_message: Image) -> None:
        stamp = Time.from_msg(image_message.header.stamp)
        try:
            map_pose = self._lookup_pose(self.map_frame, stamp)
            try:
                odom_pose = self._lookup_pose(self.odom_frame, stamp)
            except TransformException:
                odom_pose = None
            image = self.bridge.imgmsg_to_cv2(image_message, "bgr8")
            mask = self.bridge.imgmsg_to_cv2(mask_message, "mono8")
        except (TransformException, ValueError, TypeError) as error:
            self.get_logger().warning(
                f"BEV skipped: {error}", throttle_duration_sec=2.0
            )
            return

        if not self._should_accept_pose(self.last_pose, map_pose):
            return
        with self.lock:
            pixels = self.canvas.blend(
                image,
                mask,
                map_pose,
                self.bev_geometry,
                feather_pixels=int(self.get_parameter("feather_pixels").value),
                observation_weight=float(
                    self.get_parameter("observation_weight").value
                ),
            )
            if pixels == 0:
                return
            if self.database is not None:
                self.database.add(stamp.nanoseconds, image, mask, map_pose, odom_pose)
            self.last_pose = map_pose
            self.accepted_frames += 1
            self.last_rerender_source = "live"

    def _should_accept_pose(
        self, previous: Pose2D | None, current: Pose2D
    ) -> bool:
        if previous is None:
            return True
        translation, rotation = pose_delta(previous, current)
        if translation >= float(self.get_parameter("min_translation").value):
            return True
        return bool(self.get_parameter("accept_rotation_only").value) and (
            rotation >= float(self.get_parameter("min_rotation").value)
        )

    def _publish_map(self) -> None:
        image, _weight = self.canvas.snapshot()
        message = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.map_frame
        self.map_publisher.publish(message)

    def _save(self) -> dict[str, str]:
        image, weight = self.canvas.snapshot()
        frame_count = self.database.count() if self.database is not None else 0
        return save_visual_map(
            str(self.get_parameter("output_path").value),
            image,
            weight,
            self.canvas.geometry,
            metadata={
                "accepted_frames": self.accepted_frames,
                "stored_frames": frame_count,
                "last_render_source": self.last_rerender_source,
                "map_frame": self.map_frame,
                "base_frame": self.base_frame,
            },
        )

    def _save_silent(self) -> None:
        try:
            self._save()
        except OSError as error:
            self.get_logger().error(f"autosave failed: {error}")

    def _save_service(self, _request, response):
        try:
            paths = self._save()
            response.success = True
            response.message = f"saved {paths['image']}"
        except OSError as error:
            response.success = False
            response.message = str(error)
        return response

    def _corrected_poses(self) -> dict[int, Pose2D]:
        configured = str(self.get_parameter("corrected_trajectory_path").value)
        if not configured:
            return {}
        path = Path(configured)
        if not path.exists():
            raise OSError(f"corrected trajectory does not exist: {path}")
        data = json.loads(path.read_text(encoding="utf-8"))
        poses: dict[int, Pose2D] = {}
        for item in data:
            poses[int(item["stamp_ns"])] = Pose2D(
                float(item["x"]), float(item["y"]), float(item["yaw"])
            )
        return poses

    @staticmethod
    def _nearest_pose(poses: dict[int, Pose2D], stamp_ns: int) -> Pose2D | None:
        if not poses:
            return None
        key = min(poses, key=lambda value: abs(value - stamp_ns))
        return poses[key]

    def _rerender(self) -> tuple[int, str]:
        if self.database is None:
            raise RuntimeError("store_frames is disabled")
        corrected = self._corrected_poses()
        final_map_to_odom: Pose2D | None = None
        if not corrected:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.odom_frame,
                Time(),
                timeout=Duration(seconds=1.0),
            )
            final_map_to_odom = pose_from_transform(transform)
        records = self.database.records()
        with self.lock:
            self.canvas.clear()
            rendered = 0
            last_render_pose: Pose2D | None = None
            for record in records:
                pose = self._nearest_pose(corrected, record.stamp_ns)
                if pose is None and record.odom_pose is not None:
                    pose = compose_pose(final_map_to_odom, record.odom_pose)
                if pose is None:
                    pose = record.map_pose
                if not self._should_accept_pose(last_render_pose, pose):
                    continue
                image, mask = self.database.load(record)
                if self.canvas.blend(
                    image,
                    mask,
                    pose,
                    self.bev_geometry,
                    feather_pixels=int(self.get_parameter("feather_pixels").value),
                    observation_weight=float(
                        self.get_parameter("observation_weight").value
                    ),
                ):
                    rendered += 1
                    last_render_pose = pose
            source = "corrected_trajectory" if corrected else "final_map_to_odom"
            self.last_rerender_source = source
            self.last_pose = last_render_pose
        return rendered, source

    def _rerender_service(self, _request, response):
        try:
            rendered, source = self._rerender()
            paths = self._save()
            response.success = True
            response.message = (
                f"rendered {rendered} frames via {source}: {paths['image']}"
            )
        except (OSError, RuntimeError, TransformException, ValueError) as error:
            response.success = False
            response.message = str(error)
        return response

    def _reset_service(self, _request, response):
        with self.lock:
            self.canvas.clear()
            self.last_pose = None
            self.accepted_frames = 0
        response.success = True
        response.message = "canvas cleared; stored frames were preserved"
        return response

    def destroy_node(self) -> bool:
        if self.database is not None:
            self.database.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GlobalVisualMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
