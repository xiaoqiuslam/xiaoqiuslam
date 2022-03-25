cmake_minimum_required( VERSION 2.8 )
project ( myslam )

set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_BUILD_TYPE "Debug" )
set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3" )


# APPEND添加新元素到list中,告诉cmake在哪儿去找我们的modules
# cmake 编译的时候会在list中查找需要的东西
# 寻找第三方库
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
# 设置可执行程序生成的路径
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
# 设置生成库文件存放的路径
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

############### dependencies ######################

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# find_package(Eigen3 REQUIRED)
set(EIGEN3_INCLUDE_DIRS /usr/local/include/eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})
# 也可以这样写 include_directories( "/usr/local/include/eigen3" )

# find_package( Sophus REQUIRED )
set( Sophus_INCLUDE_DIRS "/home/q/projects/slambook/3rdparty/Sophus/install/include" )
# 即显示链接 Sophus_LIBRARIES 链接到 libSophus.so
# set(Sophus_LIBRARIES libSophus.so)
set( Sophus_LIBRARIES "/usr/local/lib/libSophus.so" )
include_directories(${Sophus_INCLUDE_DIRS})
link_directories(${Sophus_LIBRARIES})

# find_package( G2O REQUIRED )
set( G2O_INCLUDE_DIRS "/home/q/projects/slambook/3rdparty/g2o/install/include" )
set( G2O_LIBS "/home/q/projects/slambook/3rdparty/g2o/install/lib/" )
include_directories(${G2O_INCLUDE_DIRS})
link_directories(${G2O_LIBS})


set( THIRD_PARTY_LIBS ${OpenCV_LIBS} ${Sophus_LIBRARIES} g2o_core g2o_stuff g2o_types_sba)

############### source and test ######################

include_directories( ${PROJECT_SOURCE_DIR}/include )
add_subdirectory( src )
add_subdirectory( test )