cmake_minimum_required(VERSION 4.2.3)
project(CathPath)

find_package(Open3D REQUIRED)

add_library(cathpath_core src/mesh_utils.cpp src/voxel_grid.cpp src/centreline.cpp)

target_include_directories(cathpath_core PUBLIC include)
target_link_libraries(cathpath_core Open3D::Open3D)

add_executable(cathpath src/main.cpp)
target_link_libraries(cathpath PRIVATE cathpath_core)