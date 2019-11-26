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
find_package(rclcpp_lifecycle REQUIRED)
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
  ${rclcpp_lifecycle_INCLUDE_DIRS}
)

# canopen_chain_component
add_library(canopen_chain_component SHARED
  src/canopen_chain_component.cpp
)

target_link_libraries(canopen_chain_component
  ${rclcpp_lifecycle_LIBRARIES}
  ${Boost_LIBRARIES}
)

ament_target_dependencies(canopen_chain_component
  "class_loader"
  "canopen_master"
  "canopen_msgs"
  "std_srvs"
  "std_msgs"
  "diagnostic_updater"
  "socketcan_interface"
  "rclcpp"
)

# manual_composition
add_executable(manual_composition
  src/manual_composition.cpp
)
target_link_libraries(manual_composition
  # ${Boost_LIBRARIES}
  canopen_chain_component
)

ament_target_dependencies(manual_composition
  # "canopen_master"
  # "diagnostic_updater"
  # "socketcan_interface"
  # "std_srvs"
  "rclcpp"
  # "canopen_msgs"
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
ament_export_libraries(canopen_chain_component)

install(TARGETS
  canopen_chain_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS
    manual_composition
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
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
