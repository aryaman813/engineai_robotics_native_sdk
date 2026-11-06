cmake_minimum_required(VERSION 3.16)
project(engineai_motion_service)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interface_protocol REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

# Read the SDK's existing generated LCM header; never modify core/.
get_filename_component(SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../.." ABSOLUTE)
find_path(LCM_INCLUDE_DIR lcm/lcm-cpp.hpp
  HINTS /opt/engineai_robotics_third_party/include REQUIRED)
find_library(LCM_LIBRARY lcm
  HINTS /opt/engineai_robotics_third_party/lib REQUIRED)
get_filename_component(LCM_LIBRARY_DIR "${LCM_LIBRARY}" DIRECTORY)
find_path(SDK_LCM_INCLUDE_DIR lcm_data/TaskState.hpp
  HINTS "${SDK_ROOT}/core/include/data" REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} "srv/ChangeMotionState.srv")
rosidl_get_typesupport_target(service_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)

add_library(motion_service_impl STATIC src/motion_service.cpp)
target_include_directories(motion_service_impl PUBLIC include
  "${SDK_LCM_INCLUDE_DIR}" "${LCM_INCLUDE_DIR}")
ament_target_dependencies(motion_service_impl rclcpp interface_protocol)
target_link_libraries(motion_service_impl yaml-cpp "${LCM_LIBRARY}"
  "${service_typesupport}" Threads::Threads)
target_compile_options(motion_service_impl PRIVATE -Wall -Wextra -Wpedantic)

add_executable(motion_state_service src/main.cpp)
target_link_libraries(motion_state_service motion_service_impl)
# colcon strips build-tree RPATH on install. Retain the bundled LCM location
# and this package's generated service library without a manual environment edit.
set_target_properties(motion_state_service PROPERTIES
  INSTALL_RPATH "\$ORIGIN/..;${LCM_LIBRARY_DIR}")
install(TARGETS motion_state_service DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY srv DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_motion_service test/test_motion_service.cpp)
  target_link_libraries(test_motion_service motion_service_impl)
  target_compile_definitions(test_motion_service PRIVATE
    TEST_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/test/tasks.yaml")
  add_test(NAME motion_service_integration COMMAND test_motion_service)
  set_tests_properties(motion_service_integration PROPERTIES
    TIMEOUT 45 ENVIRONMENT "ROS_DOMAIN_ID=71;ROS_LOCALHOST_ONLY=1")
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
