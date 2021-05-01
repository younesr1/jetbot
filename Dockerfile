FROM ros:noetic-robot

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

# Install tools for using catkin build
RUN sudo apt-get update && sudo apt-get install ros-noetic-catkin python3-catkin-tools \
    python3-osrf-pycommon apt-utils python3-pip -y
RUN sudo pip3 install adafruit-circuitpython-motorkit

# Install OpenCV build tools and dependecies
RUN sudo apt-get install build-essential cmake git pkg-config libgtk-3-dev \
    libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
    libxvidcore-dev libx264-dev libjpeg-dev libpng-dev libtiff-dev \
    gfortran openexr libatlas-base-dev python3-dev python3-numpy \
    libtbb2 libtbb-dev libdc1394-22-dev libopenexr-dev \
    libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev -y
# Clone OpenCV repo
RUN mkdir /root/opencv_build && cd /root/opencv_build && \
    git clone https://github.com/opencv/opencv.git --branch '4.2.0' --depth 1
# Create temporary build folder
RUN cd /root/opencv_build/opencv && mkdir -p build && cd build
# Setup OpenCV build with CMake
RUN cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D INSTALL_C_EXAMPLES=ON \
    -D INSTALL_PYTHON_EXAMPLES=ON \
    -D OPENCV_GENERATE_PKGCONFIG=ON \
    -D OPENCV_EXTRA_MODULES_PATH=/root/opencv_build/opencv_contrib/modules \
    -D BUILD_EXAMPLES=ON ..
# Compile and install OpenCV
RUN make -j$(nproc) && sudo make install
# remove directory used for building
RUN cd /root && rm -rf opencv_build

# Copy repo
RUN mkdir -p /root/catkin_ws/src
COPY ./ /root/catkin_ws/src/jetbot
