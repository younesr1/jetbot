FROM ros:noetic-robot

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

# Install tools for using catkin build
RUN sudo apt-get update && sudo apt-get install dialog apt-utils ros-noetic-catkin python3-catkin-tools \
    python3-osrf-pycommon apt-utils python3-pip -y
RUN sudo pip3 install adafruit-circuitpython-motorkit

# Install OpenCV build tools and dependecies
RUN DEBIAN_FRONTEND=noninteractive sudo apt-get install build-essential cmake git pkg-config libgtk-3-dev \
    libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
    libxvidcore-dev libx264-dev libjpeg-dev libpng-dev libtiff-dev \
    gfortran openexr libatlas-base-dev python3-dev python3-numpy \
    libtbb2 libtbb-dev libdc1394-22-dev libopenexr-dev \
    libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev -y
# Clone OpenCV repo
RUN git config --global advice.detachedHead false && \
    mkdir /root/opencv_build && cd /root/opencv_build && \
    git clone https://github.com/opencv/opencv.git --branch '4.2.0' --depth 1
# Remove invalid compiler option
RUN sed --in-place '/-Werror=non-virtual-dtor/d' /root/opencv_build/opencv/cmake/OpenCVCompilerOptions.cmake
# Create temporary build folder
RUN cd /root/opencv_build/opencv && mkdir -p build
# Setup OpenCV build with CMake
RUN cd /root/opencv_build/opencv/build && \
    cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D ENABLE_CXX11=ON \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D INSTALL_C_EXAMPLES=ON \
    -D INSTALL_PYTHON_EXAMPLES=ON \
    -D OPENCV_GENERATE_PKGCONFIG=ON \
    -D BUILD_EXAMPLES=ON ..
# Compile and install OpenCV
RUN cd /root/opencv_build/opencv/build && make -j$(nproc) && sudo make install
# remove directory used for building
RUN rm -rf /root/opencv_build

# Copy repo
RUN mkdir -p /root/catkin_ws/src
COPY ./ /root/catkin_ws/src/jetbot
WORKDIR /root/catkin_ws
