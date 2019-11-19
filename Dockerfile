FROM dynorobotics/balena-amd64-ros2:dashing

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && apt-get install -y \
  can-utils \
  libsocketcan-dev \
  && rm -rf /var/likb/apt/lists/*

# Install nvm, Node.js and node-gyp
# NOTE(sam): Node is already installed base image, but this version is needed by ros2-web-bridge

ENV NODE_VERSION v10.16.2
RUN wget -qO- https://raw.githubusercontent.com/creationix/nvm/v0.34.0/install.sh | bash \
    && . $HOME/.nvm/nvm.sh \
    && nvm install $NODE_VERSION && nvm alias default $NODE_VERSION

# Clone ros2-web-bridge
WORKDIR /opt  
RUN git clone https://github.com/RobotWebTools/ros2-web-bridge.git --branch 0.2.7

ENV ROS2_WS /opt/ros2_ws

ENV OVERLAY_WS /opt/overlay_ws
RUN mkdir -p $OVERLAY_WS/src
WORKDIR $OVERLAY_WS/src

ENV DEPENDENCIES_WS /opt/dependencies_ws
RUN mkdir -p $DEPENDENCIES_WS/src
WORKDIR $DEPENDENCIES_WS

# clone dependency ros package repos
COPY ros_dependencies.repos $DEPENDENCIES_WS/ros_dependencies.repos
RUN vcs import src < $DEPENDENCIES_WS/ros_dependencies.repos

# install dependency ros package dependencies
WORKDIR $DEPENDENCIES_WS
RUN . $ROS2_WS/install/setup.sh && \
    apt-get update && \
    rosdep install -q -y \
      --from-paths \
        src \
      --ignore-src \
      --rosdistro $CHOOSE_ROS_DISTRO \
    && rm -rf /var/lib/apt/lists/*

# build dependency ros package source
ARG CMAKE_BUILD_TYPE=Release
WORKDIR $DEPENDENCIES_WS
RUN . $ROS2_WS/install/local_setup.sh && \
     colcon build \
       --symlink-install \
       --cmake-args \
         -DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE

# copy ros packages from this repo
WORKDIR $OVERLAY_WS/src
COPY can_msgs can_msgs
COPY canopen_402 canopen_402
COPY canopen_chain_node canopen_chain_node
COPY canopen_master canopen_master
COPY canopen_motor_node canopen_motor_node
COPY canopen_msgs canopen_msgs
COPY socketcan_bridge socketcan_bridge
COPY socketcan_interface socketcan_interface

# install ros package dependencies
WORKDIR $OVERLAY_WS
RUN . $ROS2_WS/install/local_setup.sh && \
  . $DEPENDENCIES_WS/install/local_setup.sh && \
  apt-get update && \
  rosdep install -q -y \
    --from-paths \
      src \
    --ignore-src \
    --rosdistro $CHOOSE_ROS_DISTRO \
    --skip-keys "filters diagnostic_updater" \
  && rm -rf /var/lib/apt/lists/*

# build package source
ARG CMAKE_BUILD_TYPE=Release
WORKDIR $OVERLAY_WS
RUN . $ROS2_WS/install/local_setup.sh && \
  . $DEPENDENCIES_WS/install/local_setup.sh && \
     colcon build \
       --symlink-install \
       --cmake-args \
         -DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE

# build messages for web bridge
WORKDIR /opt/ros2-web-bridge
RUN . $ROS2_WS/install/local_setup.sh \
  && . $DEPENDENCIES_WS/install/local_setup.sh \
  && . $OVERLAY_WS/install/local_setup.sh \
  && . $HOME/.nvm/nvm.sh \
  && npm install

WORKDIR $OVERLAY_WS
# NOTE(sam): Needed for autocomplete (does not seem to work with only entrypoint sourcing...)
RUN echo "source $ROS2_WS/install/local_setup.bash" >> $HOME/.bashrc
RUN echo "source $DEPENDENCIES_WS/install/local_setup.bash" >> $HOME/.bashrc
RUN echo "source $OVERLAY_WS/install/local_setup.bash" >> $HOME/.bashrc

COPY tools $OVERLAY_WS/

# source workspace from entrypoint if available
COPY tools/can-bringup.sh /opt_ws/can-bringup.sh
COPY ros_entrypoint.sh /
