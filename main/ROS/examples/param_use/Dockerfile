
FROM ros:foxy

ARG ROS2_WS=/root/dev_ws

RUN mkdir -p ${ROS2_WS}/src

# clone overlay source
ARG ROS2_WS
WORKDIR $ROS2_WS/src


RUN . /opt/ros/foxy/setup.sh && \
ros2 pkg create --build-type ament_python python_parameters --dependencies rclpy





WORKDIR $ROS2_WS/

# RUN . install/setup.bash && \
RUN . /opt/ros/foxy/setup.sh && \
    apt-get update && rosdep install -i  -y \
      --from-path src --rosdistro foxy && \
      colcon build --packages-select addressbook_pubsub
# RUN . install/setup.bash

# CMD [ros2, run, py_pubsub, talker]
  

# RUN apt-get update && apt-get install nano

#  ros2 run addressbook_pubsub talker & ros2 run addressbook_pubsub listener