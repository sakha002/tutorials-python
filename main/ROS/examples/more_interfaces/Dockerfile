
FROM ros:foxy

ARG ROS2_WS=/root/dev_ws

RUN mkdir -p ${ROS2_WS}/src

# clone overlay source
ARG ROS2_WS
WORKDIR $ROS2_WS/src

RUN . /opt/ros/foxy/setup.sh && \
ros2 pkg create --build-type ament_cmake addressbook_msg

ARG ROS2_WS
WORKDIR $ROS2_WS/src/addressbook_msg

RUN mkdir msg 

WORKDIR $ROS2_WS/src/addressbook_msg/msg

ADD ./addressbook_msg/msg/AddressBook.msg .

# WORKDIR $ROS2_WS/src/addressbook_msg/srv
# ADD ./AddThreeInts.srv .

WORKDIR $ROS2_WS/src/addressbook_msg/
ADD ./addressbook_msg/package.xml .
ADD ./addressbook_msg/CMakeLists.txt .

WORKDIR $ROS2_WS
RUN . /opt/ros/foxy/setup.sh && \
colcon build --packages-select addressbook_msg

WORKDIR $ROS2_WS/src
RUN . /opt/ros/foxy/setup.sh && \
 ros2 pkg create --build-type ament_python addressbook_pubsub
# ARG ROS2_WS

WORKDIR $ROS2_WS/src/addressbook_pubsub/

ADD ./addressbook_pubsub/package.xml .
ADD ./addressbook_pubsub/setup.py .
ADD ./addressbook_pubsub/publish_address_book.py     ./addressbook_pubsub/
ADD ./addressbook_pubsub/subscribe.py     ./addressbook_pubsub/

# ARG ROS2_WS
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