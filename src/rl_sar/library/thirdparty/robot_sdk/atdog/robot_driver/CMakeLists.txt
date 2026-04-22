cmake_minimum_required(VERSION 3.16)
project(atdog_robot_driver VERSION 0.1.0 LANGUAGES CXX)

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(ATDOG_BUILD_IMU_DRIVER "Build the IMU driver shared library" ON)
option(ATDOG_BUILD_LEG_DRIVER "Build the leg driver shared library" ON)

find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED IMPORTED_TARGET libusb-1.0)

set(ATDOG_SERIAL_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../serial)
set(ATDOG_SERIAL_SOURCES
  ${ATDOG_SERIAL_ROOT}/src/serial.cc
)

if(APPLE)
  find_library(IOKIT_LIBRARY IOKit REQUIRED)
  find_library(FOUNDATION_LIBRARY Foundation REQUIRED)
  list(APPEND ATDOG_SERIAL_SOURCES
    ${ATDOG_SERIAL_ROOT}/src/impl/unix.cc
    ${ATDOG_SERIAL_ROOT}/src/impl/list_ports/list_ports_osx.cc
  )
elseif(UNIX)
  list(APPEND ATDOG_SERIAL_SOURCES
    ${ATDOG_SERIAL_ROOT}/src/impl/unix.cc
    ${ATDOG_SERIAL_ROOT}/src/impl/list_ports/list_ports_linux.cc
  )
elseif(WIN32)
  list(APPEND ATDOG_SERIAL_SOURCES
    ${ATDOG_SERIAL_ROOT}/src/impl/win.cc
    ${ATDOG_SERIAL_ROOT}/src/impl/list_ports/list_ports_win.cc
  )
endif()

add_library(atdog_serial STATIC ${ATDOG_SERIAL_SOURCES})
add_library(atdog::serial ALIAS atdog_serial)
set_target_properties(atdog_serial PROPERTIES
  EXPORT_NAME serial
)

target_include_directories(atdog_serial
  PUBLIC
    $<BUILD_INTERFACE:${ATDOG_SERIAL_ROOT}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(atdog_serial
  PUBLIC
    Threads::Threads
)

if(APPLE)
  target_link_libraries(atdog_serial PUBLIC ${FOUNDATION_LIBRARY} ${IOKIT_LIBRARY})
elseif(UNIX)
  target_link_libraries(atdog_serial PUBLIC rt)
elseif(WIN32)
  target_link_libraries(atdog_serial PUBLIC setupapi)
endif()

set(ATDOG_EXPORT_TARGETS atdog_serial)

if(ATDOG_BUILD_IMU_DRIVER)
  add_library(atdog_imu_driver SHARED
    src/imu_driver/imu_driver.cpp
  )
  add_library(atdog::imu_driver ALIAS atdog_imu_driver)

  set_target_properties(atdog_imu_driver PROPERTIES
    OUTPUT_NAME atdog_imu_driver
    EXPORT_NAME imu_driver
  )

  target_include_directories(atdog_imu_driver
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  )

  target_link_libraries(atdog_imu_driver
    PUBLIC
      atdog::serial
      Eigen3::Eigen
      Threads::Threads
  )

  list(APPEND ATDOG_EXPORT_TARGETS atdog_imu_driver)
endif()

if(ATDOG_BUILD_LEG_DRIVER)
  add_library(atdog_leg_driver SHARED
    src/leg_driver/cdc_trans.cpp
    src/leg_driver/kalman_filter.cpp
    src/leg_driver/leg_driver.cpp
  )
  add_library(atdog::leg_driver ALIAS atdog_leg_driver)

  set_target_properties(atdog_leg_driver PROPERTIES
    OUTPUT_NAME atdog_leg_driver
    EXPORT_NAME leg_driver
  )

  target_include_directories(atdog_leg_driver
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  )

  target_link_libraries(atdog_leg_driver
    PUBLIC
      PkgConfig::LIBUSB
      Threads::Threads
  )

  list(APPEND ATDOG_EXPORT_TARGETS atdog_leg_driver)
endif()

install(TARGETS ${ATDOG_EXPORT_TARGETS}
  EXPORT atdog_robot_driver_targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY ${ATDOG_SERIAL_ROOT}/include/serial
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/atdog_robot_driverConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/atdog_robot_driverConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/atdog_robot_driver
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/atdog_robot_driverConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(EXPORT atdog_robot_driver_targets
  FILE atdog_robot_driverTargets.cmake
  NAMESPACE atdog::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/atdog_robot_driver
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/atdog_robot_driverConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/atdog_robot_driverConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/atdog_robot_driver
)
