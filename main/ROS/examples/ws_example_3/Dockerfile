
ARG FROM_IMAGE=ros:foxy
ARG OVERLAY_WS=/opt/ros/overlay_ws

# multi-stage for caching
FROM $FROM_IMAGE AS cacher

# clone overlay source
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS/src
RUN git clone https://github.com/ros/ros_tutorials.git -b foxy-devel


# multi-stage for building
FROM $FROM_IMAGE AS builder

# install overlay dependencies
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS

COPY --from=cacher $OVERLAY_WS/src ./src

RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && rosdep install -i  -y \
      --from-path \
        src --rosdistro foxy 
  


RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build 
   


# source entrypoint setup
ENV OVERLAY_WS $OVERLAY_WS
RUN sed --in-place --expression \
      '$isource "$OVERLAY_WS/install/local_setup.bash"' \
      /ros_entrypoint.sh