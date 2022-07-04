cmake_minimum_required(VERSION 2.8.3)

SET(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CXX_STANDARD 11)

project(yaml)

# Installing: /usr/local/lib/libyaml-cpp.so
# -- Up-to-date: /usr/local/include

find_package(OpenCV)
set(YAML_CPP_LIBRARIES /usr/local/lib/libyaml-cpp.so)

add_executable(yaml_read_write yaml_read_write.cpp)
target_link_libraries(yaml_read_write ${OpenCV_LIBS} ${YAML_CPP_LIBRARIES})



add_executable(ifstream ifstream.cpp)
target_link_libraries(ifstream ${OpenCV_LIBS})

add_executable(ofstream ofstream.cpp)
target_link_libraries(ofstream ${OpenCV_LIBS})

add_executable(yaml_loadfile yaml_loadfile.cpp)
target_link_libraries(yaml_loadfile ${YAML_CPP_LIBRARIES})

add_executable(file_storage_read file_storage_read.cpp)
target_link_libraries(file_storage_read ${OpenCV_LIBS})

add_executable(file_storage_write file_storage_write.cpp)
target_link_libraries(file_storage_write ${OpenCV_LIBS})




