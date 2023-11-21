# Intel® RealSense™ and ArUco Marker Detection Guide

This repository integrates Intel RealSense cameras with ArUco marker detection under ROS Noetic.


## Hardware Check

Make sure Intel® RealSense™ is detected by the host machine.

Type `lsusb` in the terminal, and it should show something like:

```
Bus 001 Device 004: ID 8086:0b3a Intel Corp. Intel(R) RealSense(TM) Depth Camera 435i
```

>Note that RealSense™ would be shown in  `dmesg | grep tty`  beacuse it do NOT communicate as traditional serial devices.  Instead, it interface with USB Video Class (UVC) devices or via specialized drivers provided by the Intel RealSense SDK.


## Installation and Run
> Host machine MUST be linux/arm/v7 architecture.


1. Git clone this repo.
    ```
    git clone https://github.com/pomelo925/arucoros-realsense.git
    ```
2. Navigate to aruco folder page and build up the docker image.

    ```
    docker compose up
    ```
3. In Visual Studio Code, press `F1` to open the command palette.

4. Type and select Remote-Containers: "Attach to Running Container...".

5. Choose the container named `pomelo` from the list to enter container.


