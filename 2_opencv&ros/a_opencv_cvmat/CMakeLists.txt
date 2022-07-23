cmake_minimum_required(VERSION 2.8)
project(opencv_ros)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)

MESSAGE(STATUS "OpenCV_VERSION " ${OpenCV_VERSION})

add_executable(creat_CV_8UC1 creat_CV_8UC1.cpp)
target_link_libraries(creat_CV_8UC1 ${OpenCV_LIBS})

add_executable(read_CV_8UC1 read_CV_8UC1.cpp)
target_link_libraries(read_CV_8UC1 ${OpenCV_LIBS})

add_executable(traverse_CV_8UC1 traverse_CV_8UC1.cpp)
target_link_libraries(traverse_CV_8UC1 ${OpenCV_LIBS})



add_executable(creat_CV_8UC3 creat_CV_8UC3.cpp)
target_link_libraries(creat_CV_8UC3 ${OpenCV_LIBS})

add_executable(read_CV_8UC3 read_CV_8UC3.cpp)
target_link_libraries(read_CV_8UC3 ${OpenCV_LIBS})

add_executable(traverse_CV_8UC3 traverse_CV_8UC3.cpp)
target_link_libraries(traverse_CV_8UC3 ${OpenCV_LIBS})


add_executable(creat_CV_8UC4 creat_CV_8UC4.cpp)
target_link_libraries(creat_CV_8UC4 ${OpenCV_LIBS})

add_executable(read_CV_8UC4 read_CV_8UC4.cpp)
target_link_libraries(read_CV_8UC4 ${OpenCV_LIBS})


add_executable(read_depth read_depth.cpp)
target_link_libraries(read_depth ${OpenCV_LIBS})


