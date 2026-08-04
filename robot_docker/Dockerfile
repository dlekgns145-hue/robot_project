ARG BASE_IMAGE=yahboomtechnology/ros-humble:4.1.2
FROM ${BASE_IMAGE}

COPY robot_cmd_bridge.py /opt/robot-control/robot_cmd_bridge.py
COPY camera_stream_server.py /opt/robot-control/camera_stream_server.py
COPY entrypoint.sh /opt/robot-control/entrypoint.sh

RUN chmod 0755 /opt/robot-control/entrypoint.sh

ENV ROS_DOMAIN_ID=20
ENTRYPOINT ["/opt/robot-control/entrypoint.sh"]
