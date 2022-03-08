cmake_minimum_required(VERSION 3.14)
project(mcptam_cali2multicol_cali)

set(CMAKE_CXX_STANDARD 11)

find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIR})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Boost REQUIRED COMPONENTS filesystem)
include_directories(${Boost_INCLUDE_DIRS})


add_executable(mcptam_cali2multicol_cali main.cpp)

target_link_libraries(mcptam_cali2multicol_cali yaml-cpp ${OpenCV_LIBS} ${Boost_LIBRARIES})
