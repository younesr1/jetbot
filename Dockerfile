FROM ros:noetic-robot
WORKDIR /root
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
COPY ./ /root/jetbot

# install vim
RUN sudo apt install update && sudo apt install vim
