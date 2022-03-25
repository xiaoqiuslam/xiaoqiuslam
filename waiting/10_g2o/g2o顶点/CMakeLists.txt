cmake_minimum_required( VERSION 2.8 )
project( BA )

# 添加c++ 11标准支持
set( CMAKE_CXX_FLAGS "-std=c++11" )


find_package( OpenCV REQUIRED )


# 添加g2o的依赖
# 因为g2o不是常用库，要添加它的findg2o.cmake文件
#set( G2O_ROOT /usr/local/include/g2o )
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
find_package( G2O REQUIRED )
find_package( CSparse REQUIRED )

# 添加头文件
include_directories( "/usr/include/eigen3"
        ${OpenCV_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIRS}
        ${CSPARSE_INCLUDE_DIR}
        )

add_executable( BA BA-3Dto2D.cpp )

target_link_libraries( BA
        ${OpenCV_LIBS}
        ${CSPARSE_LIBRARY}
        g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension
        )


