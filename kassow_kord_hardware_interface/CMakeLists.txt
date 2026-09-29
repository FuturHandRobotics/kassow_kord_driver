cmake_minimum_required(VERSION 3.8)
project(kassow_kord_hardware_interface)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(ENABLE_TEST FALSE)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

set(THIS_PACKAGE_DEPS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  kassow_kord_vendor
)

# find dependencies
find_package(ament_cmake REQUIRED)

foreach(PACKAGE ${THIS_PACKAGE_DEPS})
  find_package(${PACKAGE} REQUIRED)
endforeach()

# kord-api v3 and v4 differ in a few places this driver touches. The client
# library's major must match the robot's CBun, so the same source is built
# against either (the RT box picks one per robot, futur_docker build_rt_ws).
# Feature checks rather than version numbers: the v3.0.1 fork, upstream
# v3.0.2 and v4 all resolve correctly.
#   KORD_HAS_S_ACTUAL      joint state as S_ACTUAL_Q/QD/QDD (v3). v4 renamed
#                          them T_REFERENCE_*; v3 also declares T_REFERENCE_*
#                          but getJoint() returns nothing for them, so this
#                          must be decided here, not by what compiles.
#   KORD_HAS_MOTION_STATE  getMotionState() / EMotionState (v4 only).
include(CheckCXXSourceCompiles)
set(CMAKE_REQUIRED_LIBRARIES kord-api::kord-api)
set(CMAKE_REQUIRED_FLAGS "-std=c++17")
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
check_cxx_source_compiles("
  #include <kord/api/kord_receive_interface.h>
  int main() { auto v = kr2::kord::ReceiverInterface::EJointValue::S_ACTUAL_Q; (void)v; return 0; }"
  KORD_HAS_S_ACTUAL)
check_cxx_source_compiles("
  #include <kord/api/kord_receive_interface.h>
  int main() {
    auto f = &kr2::kord::ReceiverInterface::getMotionState; (void)f;
    auto s = kr2::kord::protocol::EMotionState::DirectJointControl; (void)s;
    return 0;
  }"
  KORD_HAS_MOTION_STATE)
unset(CMAKE_TRY_COMPILE_TARGET_TYPE)
unset(CMAKE_REQUIRED_FLAGS)
unset(CMAKE_REQUIRED_LIBRARIES)
if(KORD_HAS_S_ACTUAL)
  message(STATUS "kord-api: joint state from S_ACTUAL_* (v3)")
else()
  message(STATUS "kord-api: joint state from T_REFERENCE_* (v4)")
endif()
if(NOT KORD_HAS_MOTION_STATE)
  message(STATUS "kord-api: no motion state API (v3), motion flags are logged instead")
endif()

add_library(
  ${PROJECT_NAME}
  SHARED
  src/kassow_kord_hardware_interface.cpp
)
target_compile_definitions(${PROJECT_NAME} PRIVATE
  KORD_HAS_S_ACTUAL=$<BOOL:${KORD_HAS_S_ACTUAL}>
  KORD_HAS_MOTION_STATE=$<BOOL:${KORD_HAS_MOTION_STATE}>
)
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  include
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    hardware_interface::hardware_interface
    pluginlib::pluginlib
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    kord-api::kord-api
)

# kord-api/kord-protocol are vendored into install/kassow_kord_vendor/lib,
# not a system path -- without an explicit RPATH the loader can fall back to
# an unrelated system-wide libkord-api install (e.g. one baked into a base
# Docker image at a different version) instead of this workspace's build.
# INSTALL_RPATH_USE_LINK_PATH makes CMake compute the install RPATH from the
# actual linked libraries' locations, rather than relying on ambient
# CMAKE_INSTALL_RPATH_USE_LINK_PATH state that isn't guaranteed to be set.
set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_WITH_INSTALL_RPATH FALSE
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface kassow_kord_hardware_interface.xml)

install(
  TARGETS
  ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(ENABLE_TEST)
  ament_add_gmock(test_kassow_kord_hardware_interface test/test_kassow_kord_hardware_interface.cpp)
  target_include_directories(test_kassow_kord_hardware_interface PRIVATE include)
  ament_target_dependencies(
    test_kassow_kord_hardware_interface
    hardware_interface
    pluginlib
    ros2_control_test_assets
  )
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  ${THIS_PACKAGE_DEPS}
)

ament_package()
