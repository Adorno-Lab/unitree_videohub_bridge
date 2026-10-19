cmake_minimum_required(VERSION 3.8)
project(unitree_videohub_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs)
# unitree_sdk2 is often installed in /opt/unitree_robotics, which is not a default search path. Found here first, so
# that unitree_drivers (which needs it too) finds the same one.
find_package(unitree_sdk2 REQUIRED PATHS /opt/unitree_robotics)
# https://github.com/Adorno-Lab/unitree_drivers (DriverUnitreeVideohub)
find_package(unitree_drivers REQUIRED)

set(RCLCPP_LOCAL_BINARY_NAME unitree_videohub_bridge_node)

add_executable(${RCLCPP_LOCAL_BINARY_NAME}
    src/${RCLCPP_LOCAL_BINARY_NAME}.cpp
    src/UnitreeVideohubBridge.cpp
    )

ament_target_dependencies(${RCLCPP_LOCAL_BINARY_NAME}
    rclcpp
    sensor_msgs
    image_transport
    )

target_link_libraries(${RCLCPP_LOCAL_BINARY_NAME}
    unitree_drivers::videohub
    unitree_sdk2
    opencv_core
    opencv_imgcodecs
    )

target_compile_features(${RCLCPP_LOCAL_BINARY_NAME} PUBLIC cxx_std_17)

install(TARGETS ${RCLCPP_LOCAL_BINARY_NAME}
    DESTINATION lib/${PROJECT_NAME})

unset(RCLCPP_LOCAL_BINARY_NAME)

# Install the launch and configuration files.
install(DIRECTORY
    launch
    config
    DESTINATION share/${PROJECT_NAME}/)

ament_package()
