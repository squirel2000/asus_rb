#!/usr/bin/env bash
set -e

SRC_DIR="$(dirname "$0")/src"
SDK_ZIP="$SRC_DIR/slamware_sdk.zip"
SDK_DIR="$SRC_DIR/slamware_sdk"

echo "=== ROS2 Workspace Auto Setup Script ==="
echo "Workspace: $(dirname "$0")"
echo "----------------------------------------"

# 1. 解壓縮 SDK
if [ -d "$SDK_DIR" ]; then
    echo "[1/4] slamware_sdk already exists. Skipping unzip."
else
    if [ -f "$SDK_ZIP" ]; then
        echo "[1/4] Extracting slamware_sdk.zip..."
        unzip "$SDK_ZIP" -d "$SRC_DIR"
        echo "  -> slamware_sdk extracted."
    else
        echo "ERROR: $SDK_ZIP not found!"
        exit 1
    fi
fi

# 2. 偵測 ROS 版本
if [ -z "$ROS_DISTRO" ]; then
    echo "ERROR: ROS2 environment not sourced!"
    echo "Please run: source /opt/ros/<distro>/setup.bash"
    exit 1
fi

echo "[2/4] Detected ROS distro: $ROS_DISTRO"
sudo apt-get update
sudo apt-get install -y "ros-$ROS_DISTRO-tf-transformations"

# 3. 安裝 python3-serial
echo "[3/4] Installing python3-serial..."
sudo apt-get install -y python3-serial

# 4. colcon build
echo "[4/4] Running colcon build --symlink-install ..."
cd "$(dirname "$0")"
colcon build --symlink-install

echo "=== Setup complete! ==="
