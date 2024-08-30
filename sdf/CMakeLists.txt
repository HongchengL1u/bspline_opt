project(sdf_test)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
add_library(sdf)
include_directories(include ${CMAKE_CURRENT_SOURCE_DIR})
find_package (glog REQUIRED)
target_sources(sdf PRIVATE
    src/signed_distance_field_2d.cc
    src/grid_map_2d.cc
)
# add_executable(sdf_test test/signed_distance_field_2d_test.cc)
target_link_libraries(sdf PUBLIC ${OpenCV_LIBS})
target_link_libraries (sdf PUBLIC glog::glog)
target_link_libraries(sdf PUBLIC OpenMP::OpenMP_CXX)