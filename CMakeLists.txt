project(main)
SET(CMAKE_BUILD_TYPE "Debug") 
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_subdirectory(bspline)
add_subdirectory(bspline_opt)
add_subdirectory(path_searching)
add_subdirectory(sdf)
find_package(Eigen3 REQUIRED)
find_package(NLopt REQUIRED)
find_package(glog)
add_executable(${PROJECT_NAME} main.cpp)
include_directories(
                    ${CMAKE_CURRENT_SOURCE_DIR}/bspline/include
                    ${CMAKE_CURRENT_SOURCE_DIR}/sdf/include
                    ${CMAKE_CURRENT_SOURCE_DIR}/sdf
                    ${CMAKE_CURRENT_SOURCE_DIR}
                    ${Eigen3_INCLUDE_DIRS}
                    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/matplotlibcpp/include
                    ${NLOPT_INCLUDE_DIRS}
                    )
target_link_libraries(${PROJECT_NAME} bspline)
target_link_libraries(${PROJECT_NAME} bspline_opt)

# target_link_libraries(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/third_party/nlopt/lib/libnlopt.so)
target_link_libraries(${PROJECT_NAME}  glog::glog)
target_link_libraries(${PROJECT_NAME}  path_searching)
target_link_libraries(${PROJECT_NAME}  sdf)


find_package(PythonLibs 2.7)
target_include_directories(${PROJECT_NAME}  PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}  ${PYTHON_LIBRARIES})



