cmake_minimum_required(VERSION 3.10)
project(csrc)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set (CMAKE_CXX_STANDARD 17)
set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR})

set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# this folder clone from https://github.com/pybind/pybind11,
# and our own program need to use the compiled results of pybind11 
add_subdirectory(pybind11)
add_subdirectory(common)