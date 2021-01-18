FROM ros:foxy

ENV ROS2_WS /root/dev_ws/

RUN mkdir -p ${ROS2_WS}/src

RUN cd ${ROS2_WS}/src &&  git clone https://github.com/ros/ros_tutorials.git -b foxy-devel
 
WORKDIR ${ROS2_WS}

RUN apt-get update && rosdep install -i --from-path src --rosdistro foxy -y

WORKDIR ${ROS2_WS}

RUN   colcon build

# RUN source /opt/ros/foxy/setup.bash &&  . install/local_setup.bash


