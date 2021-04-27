FROM ros:noetic-robot
WORKDIR /root
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
COPY ./ /root/jetbot
