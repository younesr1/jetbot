FROM ros:noetic-robot
WORKDIR /root
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN sudo apt-get update && sudo apt-get install ros-noetic-catkin python3-catkin-tools python3-osrf-pycommon apt-utils -y
RUN mkdir -p /root/catkin_ws/src
COPY ./ /root/catkin_ws/src
