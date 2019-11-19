#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros2_ws/install/local_setup.sh"
source "/opt/dependencies_ws/install/local_setup.sh"

file="/opt/overlay_ws/install/setup.sh"
if [ -f "$file" ]
then
  source "$file"
else
  source "/opt/overlay_ws/install/local_setup.sh"
fi

source "$HOME/.nvm/nvm.sh"

exec "$@"
