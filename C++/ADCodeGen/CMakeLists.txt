cmake_minimum_required(VERSION 3.16)
project(ADCodeGen)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_LIBRARIES -ldl)

add_executable(ADCodeGen
        ad_dynamics.cpp
        ad_dynamics.h
        ../types.cpp
        ../Params/params.cpp
        main.cpp)

target_include_directories(ADCodeGen PRIVATE
        ../
        ../../External/Eigen/
        ../../External/Json/include/
        ../../External/CppAD/include/
        ../../External/CppADCodeGen/include/)

target_link_libraries(ADCodeGen ${CMAKE_SOURCE_DIR}/../../External/CppAD/lib/libcppad_lib.so)
