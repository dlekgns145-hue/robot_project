import math

import numpy as np

from orchard_mapper.coordinate_transform import (
    BevGeometry,
    MapGeometry,
    Pose2D,
    bev_pixels_to_robot,
    compose_pose,
    map_to_pixels,
    robot_to_map,
)


def test_bev_bottom_center_is_robot_origin():
    geometry = BevGeometry(101, 201, 5.0, 3.0)
    point = bev_pixels_to_robot(np.array([[50.0, 200.0]]), geometry)[0]
    np.testing.assert_allclose(point, [0.0, 0.0], atol=1e-12)


def test_bev_top_left_uses_ros_forward_and_left():
    geometry = BevGeometry(101, 201, 5.0, 3.0)
    point = bev_pixels_to_robot(np.array([[0.0, 0.0]]), geometry)[0]
    np.testing.assert_allclose(point, [5.0, 3.0], atol=1e-12)


def test_robot_rotation_and_north_up_map_pixel():
    local = np.array([[2.0, 0.0]])
    world = robot_to_map(local, Pose2D(10.0, 4.0, math.pi / 2.0))
    np.testing.assert_allclose(world, [[10.0, 6.0]], atol=1e-12)
    pixel = map_to_pixels(world, MapGeometry(0.5, 0.0, 0.0, 40, 30))
    np.testing.assert_allclose(pixel, [[20.0, 17.0]], atol=1e-12)


def test_pose_composition():
    pose = compose_pose(Pose2D(10.0, 5.0, math.pi / 2.0), Pose2D(2.0, 1.0, 0.2))
    np.testing.assert_allclose([pose.x, pose.y], [9.0, 7.0], atol=1e-12)
    assert math.isclose(pose.yaw, math.pi / 2.0 + 0.2)
