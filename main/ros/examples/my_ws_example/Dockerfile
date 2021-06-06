
FROM ros:foxy

ENV ROS2_WS /root/dev_ws/

RUN mkdir -p ${ROS2_WS}/src

# clone overlay source
# ARG OVERLAY_WS
WORKDIR $OVERLAY_WS/src

RUN . /opt/ros/foxy/setup.sh && \
ros2 pkg create --build-type ament_python --node-name my_node my_first_package

RUN colcon build
RUN . install/setup.bash




