cmake_minimum_required(VERSION 3.8)
project(remote_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(Threads REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

# Create executable
add_executable(remote_node
  src/main.cpp
  src/remote_node.cpp
  src/remote_comm.cpp
)

target_compile_features(remote_node PUBLIC cxx_std_17)




# Include directories
target_include_directories(remote_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dependencies
ament_target_dependencies(remote_node
  rclcpp
  robot_msgs
  serial
  geometry_msgs
  sensor_msgs
  tf2
)


# Link system libraries
target_link_libraries(remote_node
  Threads::Threads
)

# Install targets
install(TARGETS
  remote_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
