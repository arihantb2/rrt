cmake_minimum_required(VERSION 3.10.2)
project(rrt)

set(CMAKE_CXX_STANDARD_REQUIRED 14)

find_package(Eigen3 REQUIRED)

include_directories(include)

add_executable(rrt src/rrt_main.cpp src/rrt.cpp src/search_space.cpp src/obstacle.cpp)