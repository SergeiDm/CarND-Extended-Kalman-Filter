project(ExtendedKF)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++0x)

set(sources
    Source/FusionEKF.cpp
    Source/kalman_filter.cpp
    Source/main.cpp
    Source/tools.cpp)

add_executable(ExtendedKF ${sources})
