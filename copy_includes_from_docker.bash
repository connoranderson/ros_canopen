#!/bin/bash

# NOTE(sam): used to allow vscode intellisense to find headers
mkdir -p ./docker_includes
docker cp ros_canopen_dev:/opt/ros2_ws/install/include ./docker_includes/ros2_include
docker cp ros_canopen_dev:/opt/dependencies_ws/install/include ./docker_includes/dependencies_include
docker cp --follow-link ros_canopen_dev:/opt/overlay_ws/install ./docker_includes/overlay_include
