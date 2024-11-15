FROM ros:humble-ros-base

WORKDIR /workspace

COPY ./src ./src/

RUN apt update \
  && rosdep update \
  && DEBIAN_FRONTEND=noninteractive rosdep install -yi --from-paths ./src

RUN . /opt/ros/humble/setup.sh && colcon build

CMD ["sh", "-c", ". install/setup.sh && ros2 launch bringup bringup_launch.yaml"]
