cmake_minimum_required(VERSION 2.6)
project(cptr_ctrl)

find_package(Armadillo)

set(CMAKE_CXX_FLAGS "-fPIC -shared")
add_subdirectory(cblx)

