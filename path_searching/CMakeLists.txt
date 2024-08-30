cmake_minimum_required(VERSION 2.8.3)
project(path_searching)
set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3 -Wall")


find_package(Eigen3 REQUIRED)

include_directories( 
    include 
    ${Eigen3_INCLUDE_DIRS} 
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/sdf
)

add_library(path_searching 
    src/kinodynamic_astar.cpp 
    )
target_link_libraries(path_searching
    sdf
    )  
