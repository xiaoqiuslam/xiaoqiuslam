cmake_minimum_required( VERSION 2.8 )
project( directBA )

# 添加c++ 11标准支持
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
set(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear
        g2o_types_icp g2o_types_slam2d g2o_types_sba g2o_types_slam3d g2o_core g2o_interface
        g2o_solver_csparse g2o_solver_structure_only g2o_csparse_extension g2o_opengl_helper g2o_solver_dense
        g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse )
# 添加cmake模块以使用g2o库
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
find_package( G2O REQUIRED)
find_package( Eigen3 REQUIRED)
find_package( OpenCV REQUIRED )
find_package( Sophus REQUIRED )
find_package( Pangolin REQUIRED)


# 添加头文件
include_directories( "/usr/include/eigen3" )
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${Sophus_INCLUDE_DIRS} )
include_directories( ${Pangolin_INCLUDE_DIRS})
include_directories( ${G2O_INCLUDE_DIRS})



add_executable( directBA directBA.cpp )

target_link_libraries( directBA ${OpenCV_LIBS} ${G2O_LIBS})
target_link_libraries( directBA ${Sophus_LIBRARIES} )
target_link_libraries( directBA   /usr/local/lib/libSophus.so )
target_link_libraries( directBA ${Pangolin_LIBRARIES})



