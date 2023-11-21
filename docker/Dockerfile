################################################################################################
#   - Date: 2023/11/17
#   - Base image: ros:noetic (AMD64)
#       - https://hub.docker.com/layers/library/ros/noetic/images/sha256-7400e73ed09bf9643c8712de7b3a5cb0bac977c1e39c31f8d638f044a75d1e81?context=explore
################################################################################################

FROM ros:noetic

### installation
RUN apt-get update && apt-get install -y \
    ## basic tools
    sudo net-tools ssh nano git \
    ## aruco ros
    apt-utils ros-noetic-tf* \
    ## realsense SDK 
    libssl-dev libusb-1.0-0-dev libudev-dev pkg-config udev \ 
    # realsense SDK (docker)
    libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev \
    libglu1-mesa-dev libglew-dev mesa-common-dev libxi-dev\ 
    ros-noetic-diagnostic-updater \
    && rm -rf /var/lib/apt/lists/*


### install realsense SDK
RUN git clone https://github.com/IntelRealSense/librealsense.git
WORKDIR /librealsense/build
RUN cmake ../ -DBUILD_EXAMPLES=true -DCMAKE_BUILD_TYPE=Release && \
    make -j4 && make install

### ros pkg
WORKDIR /arucoros-ws/src
RUN \ 
    ## aruco ros
    git clone -b noetic-devel https://github.com/pal-robotics/aruco_ros.git && \
    git clone --branch 0.3.7 https://github.com/ros-drivers/usb_cam.git && \
    git clone https://github.com/ros-perception/image_pipeline.git && \
    ## realsense 
    git clone https://github.com/pal-robotics/ddynamic_reconfigure.git && \
    git clone --branch ros1-legacy --single-branch https://github.com/IntelRealSense/realsense-ros.git && \
    # checkout the latest Git tag
    (cd realsense-ros && git checkout $(git tag | sort -V | grep -P "^2.\\d+\\.\\d+" | tail -1))

### install and build dependencies
RUN apt-get update && \
    rosdep install --from-paths . --ignore-src -r -y && \
    cd .. && \
    /bin/bash -c 'source /opt/ros/$ROS_DISTRO/setup.bash; catkin_make' && \
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc \
    && rm -rf /var/lib/apt/lists/*

CMD ["/bin/bash"]
