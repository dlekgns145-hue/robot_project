ARG BASE_IMAGE=yahboomtechnology/ros-humble:4.1.2
FROM ${BASE_IMAGE}

COPY robot_cmd_bridge.py map_bundle.py /opt/robot-control/
COPY camera_stream_server.py /opt/robot-control/camera_stream_server.py
COPY entrypoint.sh /opt/robot-control/entrypoint.sh
COPY fastdds_profile.xml /opt/robot-control/fastdds_profile.xml
COPY navigation_runtime_launch.py mapping_runtime_launch.py navigation_probe.py navigation_run_test.py \
    publish_initial_pose.py prepare_navigation_params.py scan_diagnostics.py odom_relay.py scan_time_fix.py \
    cmd_vel_relay.py autonomous_mapping.py map_service_relay.py frontier_core.py mapping_core.py map_bundle.py camera_obstacle_guard.py \
    loadcell_guard.py \
    /opt/robot-control/navigation/
COPY recovered/Broom.yaml recovered/Broom.pgm recovered/dwb_nav_params_fixed.yaml \
    mapping_slam_params.yaml \
    /opt/robot-control/navigation/
COPY perception/detect.py perception/best.pt perception/botsort_reid.yaml \
    perception/face_detection_yunet.onnx perception/face_recognition_sface.onnx \
    /opt/robot-control/perception/
COPY follow/follow_person.py /opt/robot-control/follow/

RUN chmod 0755 /opt/robot-control/entrypoint.sh \
    /opt/robot-control/navigation/*.py \
    /opt/robot-control/perception/*.py \
    /opt/robot-control/follow/*.py

RUN mkdir -p /opt/robot-control/maps

# The image's baked-in ROS apt signing key expires periodically; refresh it
# before installing anything new or apt-get update fails with EXPKEYSIG (and,
# worse, silently falls back to a stale cached package index whose URLs 404).
# follow-me(YOLO 사람 인식 + 추적)에 필요한 cv_bridge도 여기서 같이 설치 -- 반드시
# 키 갱신 다음에 와야 함, 순서가 바뀌면 오래된 인덱스를 참조해서 404가 남.
# gpiozero+lgpio: loadcell_guard.py의 HX711 GPIO 비트뱅잉용. Pi5의 RP1 GPIO
# 칩은 옛날 RPi.GPIO 패키지를 지원 안 해서 이 조합이 필요함.
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - \
    && apt-get update \
    && apt-get install -y --no-install-recommends ros-humble-rmw-cyclonedds-cpp ros-humble-cv-bridge \
    && apt-get clean \
    && pip3 install --no-cache-dir "numpy<2" \
    && pip3 install --no-cache-dir torch==2.3.0 torchvision==0.18.0 --index-url https://download.pytorch.org/whl/cpu \
    && pip3 install --no-cache-dir "numpy<2" ultralytics opencv-python-headless "lap>=0.5.12" \
    && pip3 install --no-cache-dir gpiozero lgpio

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
