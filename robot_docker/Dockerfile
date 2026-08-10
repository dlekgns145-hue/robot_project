ARG BASE_IMAGE=yahboomtechnology/ros-humble:4.1.2
FROM ${BASE_IMAGE}

COPY robot_cmd_bridge.py /opt/robot-control/robot_cmd_bridge.py
COPY camera_stream_server.py /opt/robot-control/camera_stream_server.py
COPY entrypoint.sh /opt/robot-control/entrypoint.sh
COPY navigation_runtime_launch.py mapping_runtime_launch.py navigation_probe.py navigation_run_test.py \
    publish_initial_pose.py prepare_navigation_params.py scan_diagnostics.py odom_relay.py scan_time_fix.py \
    autonomous_mapping.py frontier_core.py map_texture_core.py map_texture_recorder.py calibrate_map_texture.py camera_obstacle_guard.py \
    /opt/robot-control/navigation/
COPY recovered/Broom.yaml recovered/Broom.pgm recovered/dwb_nav_params_fixed.yaml \
    mapping_slam_params.yaml \
    /opt/robot-control/navigation/

RUN chmod 0755 /opt/robot-control/entrypoint.sh \
    /opt/robot-control/navigation/*.py

RUN mkdir -p /opt/robot-control/maps

# Yahboom's base stack publishes stale and NaN transforms on /tf.  Nav2 keeps
# its action/service names at the root, but uses a dedicated dynamic TF topic.
RUN sed -i "s|'/tf', 'tf'|'/tf', '/tf_nav'|g" \
    /opt/ros/humble/share/nav2_bringup/launch/bringup_launch.py \
    /opt/ros/humble/share/nav2_bringup/launch/localization_launch.py \
    /opt/ros/humble/share/nav2_bringup/launch/navigation_launch.py

ENV ROS_DOMAIN_ID=20
ENTRYPOINT ["/opt/robot-control/entrypoint.sh"]
