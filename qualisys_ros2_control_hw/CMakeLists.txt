cmake_minimum_required(VERSION 3.14)
project(qualisys_ros2_control_hw)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

include(FetchContent)
if (NOT TARGET qualisys_cpp_sdk)
  message(STATUS "${PROJECT_NAME}: `qualisys_cpp_sdk` targets not found. Attempting to fetch contents...")
  FetchContent_Declare(
    qualisys_cpp_sdk
    GIT_REPOSITORY https://github.com/qualisys/qualisys_cpp_sdk
    GIT_TAG        rt_protocol_1.23
  )
  FetchContent_MakeAvailable(qualisys_cpp_sdk)

  set_target_properties(qualisys_cpp_sdk PROPERTIES
    POSITION_INDEPENDENT_CODE ON
  )
else()
  message(STATUS "qualisys_cpp_sdk: `qualisys_cpp_sdk` targets found.")
endif()

add_library(${PROJECT_NAME} SHARED
  src/qualisys_sensor.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_interface
  pluginlib
)

target_link_libraries(${PROJECT_NAME}
  qualisys_cpp_sdk
)

pluginlib_export_plugin_description_file(
  hardware_interface
  qualisys_ros2_control_hw.xml
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES qualisys_ros2_control_hw.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY description/config description/ros2_control
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

