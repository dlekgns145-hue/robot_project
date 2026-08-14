ARG BASE_IMAGE=yahboomtechnology/ros-humble:4.1.2
FROM ${BASE_IMAGE}

COPY robot_cmd_bridge.py map_bundle.py /opt/robot-control/
COPY camera_stream_server.py /opt/robot-control/camera_stream_server.py
COPY entrypoint.sh /opt/robot-control/entrypoint.sh
COPY fastdds_profile.xml /opt/robot-control/fastdds_profile.xml
COPY navigation_runtime_launch.py mapping_runtime_launch.py navigation_probe.py navigation_run_test.py \
    publish_initial_pose.py prepare_navigation_params.py scan_diagnostics.py odom_relay.py scan_time_fix.py \
    cmd_vel_relay.py autonomous_mapping.py map_service_relay.py frontier_core.py mapping_core.py map_bundle.py camera_obstacle_guard.py \
    /opt/robot-control/navigation/
COPY recovered/Broom.yaml recovered/Broom.pgm recovered/dwb_nav_params_fixed.yaml \
    mapping_slam_params.yaml \
    /opt/robot-control/navigation/

RUN chmod 0755 /opt/robot-control/entrypoint.sh \
    /opt/robot-control/navigation/*.py

RUN mkdir -p /opt/robot-control/maps

# The image's baked-in ROS apt signing key expires periodically; refresh it
# before installing anything new or apt-get update fails with EXPKEYSIG.
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - \
    && apt-get update \
    && apt-get install -y --no-install-recommends ros-humble-rmw-cyclonedds-cpp \
    && apt-get clean

# Nav2's stock launch files remap absolute /tf back to the relative name tf on
# every node.  That node-local rule overrides the mapping/navigation launch
# scope and would reconnect Nav2 to the Yahboom EKF's conflicting /tf graph.
# Keep Nav2 on the isolated transform topic owned by odom_relay and SLAM/AMCL.
RUN sed -i "s|'/tf', 'tf'|'/tf', '/tf_nav'|g" \
    /opt/ros/humble/share/nav2_bringup/launch/bringup_launch.py \
    /opt/ros/humble/share/nav2_bringup/launch/localization_launch.py \
    /opt/ros/humble/share/nav2_bringup/launch/navigation_launch.py \
    && sed -i \
        "s|('cmd_vel_smoothed', 'cmd_vel')|('cmd_vel_smoothed', '/cmd_vel_server')|g" \
        /opt/ros/humble/share/nav2_bringup/launch/navigation_launch.py \
    && sed -i \
        "/package='nav2_behaviors'/,/remappings=remappings)/ s|remappings=remappings)|remappings=remappings + [('cmd_vel', '/cmd_vel_server')])|" \
        /opt/ros/humble/share/nav2_bringup/launch/navigation_launch.py

ENV ROS_DOMAIN_ID=20
# FastDDS's default PREALLOCATED history memory policy sizes a reader's
# sample buffer from the first message it sees and cannot grow it. Messages
# relayed through the (CycloneDDS-based) Zenoh bridge can arrive at a
# different size than a native FastDDS peer would produce, which then hits
# "payload size ... larger than the history payload size ... cannot be
# resized" and the sample is silently dropped -- observed on the
# /navigate_to_pose action's goal-response, leaving navigation stuck
# forever. DYNAMIC_REUSABLE lets the buffer grow instead.
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/opt/robot-control/fastdds_profile.xml
ENTRYPOINT ["/opt/robot-control/entrypoint.sh"]
