cmake_minimum_required(VERSION 3.0)

project(trajectory)

find_package(Eigen3 REQUIRED)

include_directories(
     include
     ${EIGEN3_INCLUDE_DIRS}
)

add_library(Trajectory SHARED 
     src/cubic.cpp
)


add_executable(trajTest bin/cubicTraj.cpp)
target_link_libraries(trajTest PUBLIC Trajectory)