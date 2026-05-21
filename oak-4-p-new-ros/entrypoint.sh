#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/OAK_WS/install/setup.bash"
source "$HOME/.bashrc"
exec "$@"