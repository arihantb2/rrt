cmake_minimum_required(VERSION 3.10.2)
project(rrt)

set(CMAKE_CXX_STANDARD_REQUIRED 14)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include gtest/include)

add_executable(rrt src/rrt_main.cpp src/rrt.cpp src/search_space.cpp src/obstacle.cpp)
target_link_libraries(rrt yaml-cpp)

add_executable(test_obstacle src/test/test_obstacle.cpp src/obstacle.cpp)
