CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

PROJECT( slam )

SET( CMAKE_CXX_COMPILER "g++")
SET( CMAKE_BUILD_TYPE DEBUG  )
SET( CMAKE_CXX_FLAGS "-std=c++11 -O3")

# 增加PCL库的依赖
FIND_PACKAGE( PCL REQUIRED)

# 增加opencv的依赖
FIND_PACKAGE( OpenCV REQUIRED )

# 添加g2o的依赖
# 因为g2o不是常用库，要添加它的findg2o.cmake文件
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
SET( G2O_ROOT /usr/local/include/g2o )
FIND_PACKAGE( G2O REQUIRED )

# CSparse
FIND_PACKAGE( CSparse REQUIRED )
INCLUDE_DIRECTORIES( ${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR} )


ADD_EXECUTABLE( visualOdometry visualOdometry.cpp)
TARGET_LINK_LIBRARIES( visualOdometry ${OpenCV_LIBS}  ${PCL_LIBRARIES} g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension ${CSPARSE_LIBRARY})




