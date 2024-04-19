cmake_minimum_required(VERSION 3.10)
project(map_compressor)

set(CMAKE_BUILD_TYPE "Debug")

add_compile_options(-std=c++20)
add_compile_options(-std=c++20)
set(CMAKE_CXX_FLAGS "-std=c++20")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

set(dependencies
    rclcpp
    rclcpp_components
    nav_msgs
    visualization_msgs
    pcl_ros
    pcl_conversions
    livox_ros_driver2)

include_directories(include ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

set(src_list src/main.cpp)

add_executable(run ${src_list})

ament_target_dependencies(run ${dependencies})

target_link_libraries(run ${PCL_LIBRARIES})

install(TARGETS run DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
