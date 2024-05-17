cmake_minimum_required(VERSION 3.10)
project(local_nav)

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
find_package(nav_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

set(src src/main.cpp src/map_maker/filter.cpp)
set(dependencies rclcpp nav_msgs pcl_ros pcl_conversions livox_ros_driver2
                 OpenCV)
set(library ${PCL_LIBRARIES})
set(include ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(run ${src})
target_link_libraries(run ${library})
ament_target_dependencies(run ${dependencies})
include_directories(${include})

set(install config launch)

install(TARGETS run DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY ${install} DESTINATION share/${PROJECT_NAME})

ament_package()
