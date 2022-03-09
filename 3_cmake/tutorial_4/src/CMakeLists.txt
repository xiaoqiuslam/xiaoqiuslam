#工程添加多个特定的头文件搜索路径
include_directories(include)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
# 共享库
add_library( tutorial_4 SHARED tutorial_4.cpp)




