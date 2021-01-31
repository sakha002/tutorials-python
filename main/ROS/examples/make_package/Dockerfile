
FROM ros:foxy

ENV ROS2_WS /root/dev_ws/

RUN mkdir -p ${ROS2_WS}/src

# clone overlay source
# ARG OVERLAY_WS
WORKDIR $OVERLAY_WS/src

RUN . /opt/ros/foxy/setup.sh && \
ros2 pkg create --build-type ament_python py_pubsub

WORKDIR $OVERLAY_WS/src/py_pubsub/py_pubsub

RUN apt-get update && apt-get install wget

RUN wget https://raw.githubusercontent.com/ros2/examples/master/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py

RUN apt-get update && apt-get install nano`