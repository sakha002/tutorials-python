
FROM ros:foxy

ARG ROS2_WS=/root/dev_ws

RUN mkdir -p ${ROS2_WS}/src

# clone overlay source
ARG ROS2_WS
WORKDIR $ROS2_WS/src

RUN . /opt/ros/foxy/setup.sh && \
ros2 pkg create --build-type ament_cmake tutorial_interfaces

ARG ROS2_WS
WORKDIR $ROS2_WS/src/tutorial_interfaces

RUN mkdir msg && mkdir srv

WORKDIR $ROS2_WS/src/tutorial_interfaces/msg

# RUN echo "int64 num" >> Num.msg
ADD ./Num.msg .

WORKDIR $ROS2_WS/src/tutorial_interfaces/srv
# RUN echo " int64 a \n int64 b \n int64 c  \n --- \n int64 sum" >> AddThreeInts.srv
ADD ./AddThreeInts.srv .

WORKDIR $ROS2_WS/src/tutorial_interfaces/
ADD ./package.xml .
ADD ./CMakeLists.txt .

WORKDIR $ROS2_WS
RUN . /opt/ros/foxy/setup.sh && \
colcon build 

# ARG ROS2_WS
# WORKDIR $ROS2_WS/src/py_pubsub/

# ADD ./py_pubsub/package.xml .
# ADD ./py_pubsub/setup.py .

# ARG ROS2_WS
# WORKDIR $ROS2_WS/

# RUN . /opt/ros/foxy/setup.sh && \
#     apt-get update && rosdep install -i  -y \
#       --from-path src --rosdistro foxy && \
#       colcon build --packages-select py_pubsub
# RUN . install/setup.bash

# CMD [ros2, run, py_pubsub, talker]
  

# RUN apt-get update && apt-get install nano