cmake_minimum_required(VERSION 2.8 )




project(cluster_extraction)

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# add_executable (cluster_extraction cluster_extraction.cpp)
# target_link_libraries (cluster_extraction ${PCL_LIBRARIES})

add_executable (extract_indices extract_indices.cpp)
target_link_libraries (extract_indices ${PCL_LIBRARIES})

# add_executable (cc cc.cpp)
# target_link_libraries (cc ${PCL_LIBRARIES})


# add_executable (3 3.cpp)
# target_link_libraries (3 ${PCL_LIBRARIES})

