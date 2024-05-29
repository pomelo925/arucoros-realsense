## AMD64 architecture ##
FROM ros:noetic 

LABEL org.opencontainers.image.authors="yoseph.huang@gmail.com"

## pkgs installation
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    sudo net-tools ssh nano git usbutils x11-apps \
    apt-utils ros-noetic-tf* \
    libssl-dev libusb-1.0-0-dev libudev-dev pkg-config udev \
    libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev \
    libglu1-mesa-dev libglew-dev mesa-common-dev libxi-dev \
    ros-noetic-diagnostic-updater ros-noetic-rviz ros-noetic-robot-state-publisher \
    ros-noetic-pcl-conversions ros-noetic-pcl-ros pcl-tools \
    && rm -rf /var/lib/apt/lists/*d

### install realsense SDK
WORKDIR /opt/librealsense
RUN git clone https://github.com/IntelRealSense/librealsense.git /opt/librealsense \
    && git rm -r --cached .

### remove control_transfer error msg from librealsense
COPY /scripts/rm-controlTransfer-warning.sh /rm-controlTransfer-warning.sh
RUN /bin/bash /rm-controlTransfer-warning.sh

WORKDIR /opt/librealsense/build
RUN cmake ../ -DBUILD_EXAMPLES=true -DCMAKE_BUILD_TYPE=Release \
    && make -j4 && make install

### ros pkg
WORKDIR /home/realsense-ws/src
RUN git clone --branch 0.3.7 https://github.com/ros-drivers/usb_cam.git && \
    git clone --branch noetic https://github.com/ros-perception/image_pipeline.git && \
    git clone --branch kinetic-devel https://github.com/pal-robotics/ddynamic_reconfigure.git && \
    git clone --branch ros1-legacy --single-branch https://github.com/IntelRealSense/realsense-ros.git && \
    (cd realsense-ros && git checkout $(git tag | sort -V | grep -P "^2.\\d+\\.\\d+" | tail -1)) && \
    rm -rf .git

### install and build dependencies
RUN sudo apt-get update && \
    rosdep update && \
    rosdep install --from-paths . --ignore-src -r -y && \
    cd .. && \
    /bin/bash -c 'source /opt/ros/$ROS_DISTRO/setup.bash; catkin_make' && \
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc \
    && sudo rm -rf /var/lib/apt/lists/* 

### fix tf2_server_test_nodelets.xml missing error
COPY /scripts/fix-tf2.sh /fix-tf2.sh
RUN /bin/bash /fix-tf2.sh 