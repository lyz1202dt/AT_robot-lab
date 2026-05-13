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
find_package(Threads REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

find_package(serial QUIET)

if(NOT serial_FOUND)
  message(WARNING
    "ROS package 'serial' was not found. Falling back to bundled sources at "
    "src/rl_sar/library/thirdparty/robot_sdk/atdog/serial.")

  set(REMOTE_NODE_BUNDLED_SERIAL_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../rl_sar/library/thirdparty/robot_sdk/atdog/serial")

  if(NOT EXISTS "${REMOTE_NODE_BUNDLED_SERIAL_DIR}/include/serial/serial.h")
    message(FATAL_ERROR
      "Bundled serial sources were not found at "
      "${REMOTE_NODE_BUNDLED_SERIAL_DIR}. "
      "Install the ROS 'serial' package or restore the bundled dependency.")
  endif()

  add_library(remote_node_serial STATIC
    "${REMOTE_NODE_BUNDLED_SERIAL_DIR}/src/serial.cc"
  )

  target_include_directories(remote_node_serial
    PUBLIC
      "${REMOTE_NODE_BUNDLED_SERIAL_DIR}/include"
  )

  target_compile_features(remote_node_serial PUBLIC cxx_std_17)

  if(APPLE)
    find_library(IOKIT_LIBRARY IOKit)
    find_library(FOUNDATION_LIBRARY Foundation)
    target_sources(remote_node_serial PRIVATE
      "${REMOTE_NODE_BUNDLED_SERIAL_DIR}/src/impl/unix.cc"
      "${REMOTE_NODE_BUNDLED_SERIAL_DIR}/src/impl/list_ports/list_ports_osx.cc"
    )
    target_link_libraries(remote_node_serial PUBLIC
      ${FOUNDATION_LIBRARY}
      ${IOKIT_LIBRARY}
    )
  elseif(UNIX)
    target_sources(remote_node_serial PRIVATE
      "${REMOTE_NODE_BUNDLED_SERIAL_DIR}/src/impl/unix.cc"
      "${REMOTE_NODE_BUNDLED_SERIAL_DIR}/src/impl/list_ports/list_ports_linux.cc"
    )
    target_link_libraries(remote_node_serial PUBLIC
      rt
      pthread
    )
  elseif(WIN32)
    target_sources(remote_node_serial PRIVATE
      "${REMOTE_NODE_BUNDLED_SERIAL_DIR}/src/impl/win.cc"
      "${REMOTE_NODE_BUNDLED_SERIAL_DIR}/src/impl/list_ports/list_ports_win.cc"
    )
    target_link_libraries(remote_node_serial PUBLIC setupapi)
  endif()
endif()

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
  geometry_msgs
  sensor_msgs
  tf2
)

if(serial_FOUND)
  ament_target_dependencies(remote_node serial)
else()
  target_link_libraries(remote_node remote_node_serial)
endif()


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
