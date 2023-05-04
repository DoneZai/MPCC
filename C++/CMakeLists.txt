cmake_minimum_required(VERSION 3.8)
project(MPCC)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-O0")

add_executable(MPCC
        main.cpp
        types.cpp
        types.h
        config.h
        Params/params.cpp
        Spline/cubic_spline.cpp
        Spline/arc_length_spline.cpp
        Interfaces/hpipm_interface.cpp
        Interfaces/solver_interface.cpp
        Constraints/constraints.cpp
        Constraints/bounds.cpp
        Cost/cost.cpp
        Model/model.cpp
        Model/integrator.cpp
        # Tests/spline_test.cpp
        # Tests/cost_test.cpp
        # Tests/model_integrator_test.cpp
        # Tests/constratins_test.cpp
        MPC/mpc.cpp
        Params/track.cpp
        Params/track.h
        Plotting/plotting.cpp
        Plotting/plotting.h)

find_package(Python3 COMPONENTS Interpreter NumPy Development)
target_include_directories(MPCC PRIVATE 
        ./
        ../External/blasfeo/include
        ../External/hpipm/include
        ../External/matplotlib/
        ../External/Eigen/
        ../External/Json/include/
        ../External/CppAD/include/
        ../External/CppADCodeGen/include/
        ${Python3_INCLUDE_DIRS}
        )
target_link_libraries(MPCC ${Python3_LIBRARIES})

target_link_libraries(MPCC ${CMAKE_SOURCE_DIR}/../External/hpipm/lib/libhpipm.a ${CMAKE_SOURCE_DIR}/../External/blasfeo/lib/libblasfeo.a m dl)