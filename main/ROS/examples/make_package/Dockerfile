
FROM ros:foxy

ARG ROS2_WS=/root/dev_ws/

RUN mkdir -p ${ROS2_WS}/src

# clone overlay source
ARG ROS2_WS
WORKDIR $ROS2_WS/src

RUN . /opt/ros/foxy/setup.sh && \
ros2 pkg create --build-type ament_python py_pubsub

ARG ROS2_WS
WORKDIR $ROS2_WS/src/py_pubsub/py_pubsub

RUN apt-get update && apt-get install wget

RUN wget https://raw.githubusercontent.com/ros2/examples/master/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py

RUN wget https://raw.githubusercontent.com/ros2/examples/master/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py

ARG ROS2_WS
WORKDIR $ROS2_WS/src/py_pubsub/

ADD ./py_pubsub/package.xml .
ADD ./py_pubsub/setup.py .

ARG ROS2_WS
WORKDIR $ROS2_WS/

RUN . /opt/ros/foxy/setup.sh && \
    apt-get update && rosdep install -i  -y \
      --from-path src --rosdistro foxy && \
      colcon build --packages-select py_pubsub
# RUN . install/setup.bash

# CMD [ros2, run, py_pubsub, talker]
  

# RUN apt-get update && apt-get install nano