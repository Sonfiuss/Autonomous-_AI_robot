# Source this before any slam/ work:  source slam/env.sh
# ROS 2 Foxy is NOT auto-sourced on this Jetson — this script fixes that.

source /opt/ros/foxy/setup.bash

# Overlay the slam workspace if it has been built
_SLAM_WS="$HOME/Documents/slam/ros2_ws"
if [ -f "$_SLAM_WS/install/setup.bash" ]; then
    source "$_SLAM_WS/install/setup.bash"
else
    echo "[slam/env.sh] workspace not built yet — run: (cd $_SLAM_WS && colcon build)"
fi

# Sanity check (A1 risk mitigation: 'package not found' everywhere if unsourced)
if [ "$ROS_DISTRO" != "foxy" ]; then
    echo "[slam/env.sh] WARNING: ROS_DISTRO='$ROS_DISTRO' (expected 'foxy')"
fi

# DA-V2 + serial bridge own the GPU/serial; never run deepmap or motivation at the
# same time (see interfaces.md: /dev/ttyUSB0 single-owner rule).
export SLAM_ROOT="$HOME/Documents/slam"
