cmake_minimum_required(VERSION 2.8.4)

# 隐式定义变量<projectname>_BINARY_DIR,<projectname>_SOURCE_DIR,PROJECT_BINARY_DIR,PROJECT_SOURCE_DIR
# HELLO_SOURCE_DIR 工程路径~/cmake_tutorial/tutorial_1
# HELLO_BINARY_DIR 编译路径~/cmake_tutorial/tutorial_1/build
# 可以使用MESSAGE打印这两个变量的值
project(tutorial_1)

# 生成可执行文件
add_executable(tutorial_1 main.cpp)

# MESSAGE 向终端输出信息
message(STATUS "This is BINARY dir " ${PROJECT_BINARY_DIR})
message(STATUS "This is SOURCE dir " ${PROJECT_SOURCE_DIR})

message( "This is BINARY dir " ${PROJECT_BINARY_DIR})
message( "This is SOURCE dir " ${PROJECT_SOURCE_DIR})


# 跨平台 Linux/Unix平台生成 makefile,苹果平台生成 xcode,Windows平台生成 MSVC 的工程文件
# 管理大型项目
# Cmake 的工具链：cmake+make
# CMakeLists.txt每个目录一个
# CMakeLists.txt文件名是大小写相关
# cmake语法
# 变量大小写相关使用${}方式取值,区别于IF需要变量名而不是变量值
# 指令大小写无关(参数 1 参数 2...)参数大小写相关使用括弧括起之间使用空格或分号分开