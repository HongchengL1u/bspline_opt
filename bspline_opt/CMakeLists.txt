cmake_minimum_required(VERSION 2.8.3)
project(bspline_opt)
set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3 -Wall")
find_package(Eigen3 REQUIRED)
find_package(NLopt)



include_directories( 
    include 
    ../third_party/nlopt/include
    ${Eigen3_INCLUDE_DIRS} 
    ${NLOPT_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/sdf
)

add_library(${PROJECT_NAME}
    src/bspline_optimizer.cpp 
    )

target_link_libraries(${PROJECT_NAME} 
                        ${NLOPT_LIBRARIES}
                        sdf)
