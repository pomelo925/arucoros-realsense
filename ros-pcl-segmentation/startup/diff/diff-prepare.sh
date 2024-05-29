#!/bin/bash

echo "[DIFF][INSPECTION] Waiting for ready signal ..."

cd /home/extraction-ws
source devel/setup.bash
rosrun diff-rs-pcl2 D-entrypoint