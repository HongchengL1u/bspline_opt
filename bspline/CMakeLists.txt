
cmake_minimum_required(VERSION 2.8.3)
project(bspline)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall")
find_package(Eigen3 REQUIRED)
include_directories( 
    include 
    ${Eigen3_INCLUDE_DIRS} 
)
add_library(bspline src/non_uniform_bspline.cpp)
 
