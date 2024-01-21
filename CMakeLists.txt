cmake_minimum_required(VERSION 3.0)

project(trajectory)

find_package(Eigen3 REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development NumPy)

include_directories(
     include
     ${EIGEN3_INCLUDE_DIRS}
     ${Python3_INCLUDE_DIRS}
)

add_library(Trajectory
     src/cubic.cpp
     src/quintic.cpp
     src/parabolicBlend.cpp
     src/septic.cpp
     src/cubic_via_point.cpp
     src/cubicMultiViaPoint.cpp
     src/quinticMulitViapt.cpp
     src/quintUpdated.cpp
     src/septicUpdated.cpp
     src/Scurve.cpp
)

add_executable(trajTest bin/cubicTraj.cpp)
target_link_libraries(trajTest
     Trajectory
     plotter
     ${Python3_LIBRARIES}
)
add_executable(run bin/run.cpp )
target_link_libraries(run Trajectory)
install(TARGETS Trajectory DESTINATION lib)
install(FILES include/cubic.h
               include/trajectories/trajectories.h
               include/cubic_via_point.h
               include/cubicMultiViaPoint.h
               include/quinticMulitViapt.h
               include/quintic.h
     DESTINATION include
)