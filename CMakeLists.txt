cmake_minimum_required(VERSION 3.10.2)
project(rrt)

set(CMAKE_CXX_STANDARD_REQUIRED 14)
set(CMAKE_BUILD_TYPE Debug)

find_package(Eigen3 REQUIRED)
find_package(GTest REQUIRED)

enable_testing()

include_directories(include ${GTEST_INCLUDE_DIRS})

add_executable(rrt src/rrt_main.cpp src/rrt.cpp src/search_space.cpp src/obstacle.cpp)
target_link_libraries(rrt yaml-cpp)

add_executable(LineObstacleTest src/test/unittest_LineObstacle.cpp src/obstacle.cpp)
target_link_libraries(LineObstacleTest ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES} pthread)

add_executable(CircleObstacleTest src/test/unittest_CircleObstacle.cpp src/obstacle.cpp)
target_link_libraries(CircleObstacleTest ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES} pthread)

add_test(LineObstacle-Test LineObstacleTest)
add_test(CircleObstacle-Test CircleObstacleTest)