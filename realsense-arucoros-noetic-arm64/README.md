# Intel® RealSense™ & ArUco Marker on ROS Noetic Guide (ARM64)

WARNING: This Repo Is Incomplete

## Settings

1. Please change hostname in `docker-compose.yaml`:

    ```py
    ## [IMPORTANT] Change container hostname to that of the host machine's 
        hostname: cleaner  
    ```

## What've Done ?

1. fundamental functions and packages. 
    e.g `x11-apps`, `openssh-server`, `net-tools`, `usbutils` etc.
2. Realsense SDK and ros:noetic env built.
3. realsense-ros repository built.

## Existed Problem

 Any GUI-app would occur this error :

```bash
MoTTY X11 proxy: Authorisation not recognised
In case you are trying to start a graphical application with "sudo", read this article in order to avoid this issue:
https://blog.mobatek.net/post/how-to-keep-X11-display-after-su-or-sudo/
Error: Can't open display: localhost:10.0
```
