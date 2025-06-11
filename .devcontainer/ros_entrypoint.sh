#!/usr/bin/env bash
set -e

# ROS-Umgebung
source /opt/ros/$ROS_DISTRO/setup.bash

# Falls ein Workspace installiert wurde
if [ -f "/home/$USER/autonomous-system/install/setup.bash" ]; then
  source /home/$USER/autonomous-system/install/setup.bash
fi

exec "$@"
