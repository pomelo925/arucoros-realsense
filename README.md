# Intel® RealSense™ and ArUco Marker Detection Guide

This repository integrates Intel RealSense cameras with ArUco marker detection under ROS Noetic.


## Hardware Check

Make sure Intel® RealSense™ is detected by the host machine.

Type `lsusb` in the terminal, and it should show something like:

```
Bus 001 Device 004: ID 8086:0b3a Intel Corp. Intel(R) RealSense(TM) Depth Camera 435i
```

>Note that RealSense™ would be shown in  `dmesg | grep tty`  beacuse it do NOT communicate as traditional serial devices.  Instead, it interface with USB Video Class (UVC) devices or via specialized drivers provided by the Intel RealSense SDK.


## Build Image and Run Container
Host machine MUST be linux/arm/v7 architecture.


1. Git clone this repo.
    ```
    git clone https://github.com/pomelo925/arucoros-realsense.git
    ```
2. Navigate to `/realsense-arucoros-noetic-arm64 page` (for example) and run the docker image.

    ```
    docker compose up
    ```

3. In Visual Studio Code, press `F1` to open the command palette.

4. Type and select Remote-Containers : "Attach to Running Container...".

5. Choose the container named `pomelo` from the list to enter container.


## Intel® RealSense™ Connection Test
if error like this:
```
 07/12 05:04:42,078 ERROR [281472812437904] (librealsense-exception.h:52) xioctl(VIDIOC_G_CTRL) failed Last Error: Connection timed out
 ```
 please make sure Intel® RealSense™ is inserted on USB 3.0 port (with the blue mark).



 ## Existed Problem
 Any GUI-app would occur this error : 
```powershell=
MoTTY X11 proxy: Authorisation not recognised
In case you are trying to start a graphical application with "sudo", read this article in order to avoid this issue:
https://blog.mobatek.net/post/how-to-keep-X11-display-after-su-or-sudo/
Error: Can't open display: localhost:10.0
```