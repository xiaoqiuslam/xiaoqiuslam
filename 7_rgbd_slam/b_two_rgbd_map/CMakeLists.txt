CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )
PROJECT( slam )

SET(CMAKE_CXX_COMPILER "g++")
SET( CMAKE_BUILD_TYPE Debug  )
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS "-O3")

# 增加opencv的依赖
FIND_PACKAGE( OpenCV REQUIRED )

# 增加PCL库的依赖
FIND_PACKAGE( PCL REQUIRED COMPONENTS common io visualization )

include_directories("/usr/include/eigen3")

# 安装视觉SLAM十四讲第一版slambook下面的Sophus
set(Sophus_LIBRARIES /usr/local/lib/libSophus.so)

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

# 添加头文件和库文件
ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS}  )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )


ADD_EXECUTABLE( two_rgbd_map two_rgbd_map.cpp)
target_link_libraries(two_rgbd_map ${Sophus_LIBRARIES} ${Pangolin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

