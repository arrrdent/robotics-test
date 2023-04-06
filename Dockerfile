FROM ghcr.io/remyrobotics/robotics-test:latest

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-moveit \
    && rm -rf /var/lib/apt/lists/* \
    && rm -rf /etc/ros/rosdep/* \
    && rosdep init \
    && rosdep update

COPY catkin_ws/src /home/catkin_ws/src
COPY catkin_ws/src/simple_scene/worlds /usr/share/gazebo-11/worlds
COPY catkin_ws/src/simple_scene/models /root/.gazebo/models

RUN . /opt/ros/noetic/setup.sh \
    && apt-get update \
    && cd /home/catkin_ws \
    && rosdep install --rosdistro=noetic --from-path . -y --ignore-src \
    && catkin build

