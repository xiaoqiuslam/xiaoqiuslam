cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(VoxelGrid)

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITION})


add_executable(VoxelGrid VoxelGrid.cpp)

target_link_libraries(VoxelGrid ${PCL_LIBRARIES})

# install(TARGETS voxelgrid_filter RUNTIME DESTINATION bin)
