cmake_minimum_required(VERSION 3.5)
project(canopen_chain_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(class_loader REQUIRED)

find_package(canopen_master REQUIRED)

find_package(diagnostic_updater REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(socketcan_interface REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(canopen_msgs REQUIRED)

find_package(Boost REQUIRED
  COMPONENTS
    filesystem
    chrono
    thread
)

include_directories(
  include
)

# canopen_ros_chain
add_library(canopen_ros_chain SHARED
  src/ros_chain.cpp
)

target_link_libraries(canopen_ros_chain
  ${Boost_LIBRARIES}
)

ament_target_dependencies(canopen_ros_chain
  "class_loader"
  "canopen_master"
  "canopen_msgs"
  "std_srvs"
  "std_msgs"
  "diagnostic_updater"
  "socketcan_interface"
)

# canopen_chain_node
add_executable(${PROJECT_NAME}
  src/chain_node.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  canopen_ros_chain
)

ament_target_dependencies(${PROJECT_NAME}
  "canopen_master"
  "diagnostic_updater"
  "socketcan_interface"
  "std_srvs"
  "rclcpp"
  "canopen_msgs"
)

ament_export_dependencies(
  ament_cmake
  canopen_master
  diagnostic_updater
  socketcan_interface
  std_srvs
  rclcpp
  canopen_msgs
  Boost
)

ament_export_include_directories(include)
ament_export_libraries(canopen_ros_chain)

install(TARGETS
  canopen_ros_chain
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS
    ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY test
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
