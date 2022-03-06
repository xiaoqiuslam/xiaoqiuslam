# Install script for directory: /home/q/projects/slambook2/examples/interactive_slam/slam_parser/parser

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/q/projects/slambook2/examples/cmake-build-debug/interactive_slam/slam_parser/parser/libparser.a")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/examples/interactive_slam/slam_parser/parser/FlexLexer.h;/examples/interactive_slam/slam_parser/parser/bison_parser.h;/examples/interactive_slam/slam_parser/parser/commands.h;/examples/interactive_slam/slam_parser/parser/driver.h;/examples/interactive_slam/slam_parser/parser/location.hh;/examples/interactive_slam/slam_parser/parser/position.hh;/examples/interactive_slam/slam_parser/parser/scanner.h;/examples/interactive_slam/slam_parser/parser/slam_context.h;/examples/interactive_slam/slam_parser/parser/stack.hh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/examples/interactive_slam/slam_parser/parser" TYPE FILE FILES
    "/home/q/projects/slambook2/examples/interactive_slam/slam_parser/parser/FlexLexer.h"
    "/home/q/projects/slambook2/examples/interactive_slam/slam_parser/parser/bison_parser.h"
    "/home/q/projects/slambook2/examples/interactive_slam/slam_parser/parser/commands.h"
    "/home/q/projects/slambook2/examples/interactive_slam/slam_parser/parser/driver.h"
    "/home/q/projects/slambook2/examples/interactive_slam/slam_parser/parser/location.hh"
    "/home/q/projects/slambook2/examples/interactive_slam/slam_parser/parser/position.hh"
    "/home/q/projects/slambook2/examples/interactive_slam/slam_parser/parser/scanner.h"
    "/home/q/projects/slambook2/examples/interactive_slam/slam_parser/parser/slam_context.h"
    "/home/q/projects/slambook2/examples/interactive_slam/slam_parser/parser/stack.hh"
    )
endif()

