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

find_package(Eigen3 REQUIRED)

set(src main.cpp)
set(dependencies)
set(library)
set(include_dir ${EIGEN3_INCLUDE_DIR})

add_executable(run ${src})
target_link_libraries(run ${library})
include_directories(${include_dir})
