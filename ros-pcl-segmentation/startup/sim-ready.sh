#!/bin/bash

echo "[SIMULATION] Sending Ready Signal ..."

cd /home/extraction-ws/src/diff-rs-pcl2/src
chmod +x ins-ready.py
python3 ins-ready.py