cmake_minimum_required(VERSION 3.7)
project(simulation_1)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_VERBOSE_MAKEFILE ON)

# Set Compile and Linker flags

set(GCC_COVERAGE_COMPILE_FLAGS "-D FREEGLUT_STATIC")
set(GCC_COVERAGE_LINK_FLAGS    "-static")


set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}" )
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}")

set(SOURCE_FILES
        ParticleToy/CircularWireConstraint.cpp
        ParticleToy/CircularWireConstraint.h
        ParticleToy/linearSolver.cpp
        ParticleToy/linearSolver.h
        ParticleToy/Particle.cpp
        ParticleToy/Particle.h
        #ParticleToy/ParticleToy.cpp
        ParticleToy/RodConstraint.cpp
        ParticleToy/RodConstraint.h
        ParticleToy/Solver.cpp
        ParticleToy/SpringForce.cpp
        ParticleToy/SpringForce.h
        ParticleToy/Force.cpp
        ParticleToy/Force.h
        ParticleToy/GravityForce.cpp
        ParticleToy/GravityForce.h
        ParticleToy/ConstraintForce.cpp
        ParticleToy/ConstraintForce.h
        ParticleToy/ForwardEuler.cpp
        ParticleToy/ForwardEuler.h
        ParticleToy/RK4.cpp
        ParticleToy/RK4.h
        ParticleToy/Midpoint.cpp
        ParticleToy/Midpoint.h
        ParticleToy/LambdaSolver.cpp
        ParticleToy/LambdaSolver.h
        ParticleToy/SlidingWireConstraint.cpp
        ParticleToy/SlidingWireConstraint.h
        ParticleToy/HorizontalForce.cpp
        ParticleToy/HorizontalForce.h
        ParticleToy/WallForce.cpp
        ParticleToy/WallForce.h
        ParticleToy/AngularSpringForce.cpp
        ParticleToy/AngularSpringForce.h
        ParticleToy/SetUp.cpp
        ParticleToy/SetUp.h
        ParticleToy/Stam/solver.cpp ParticleToy/Stam/solver.h ParticleToy/Stam/demo.cpp ParticleToy/Stam/demo.h ParticleToy/Stam/Cell.cpp ParticleToy/Stam/Cell.h ParticleToy/Stam/Marker.cpp ParticleToy/Stam/Marker.h ParticleToy/SolidFluidForce.cpp ParticleToy/SolidFluidForce.h ParticleToy/Stam/ComputeFluidForce.cpp ParticleToy/Stam/ComputeFluidForce.h)

find_package (OpenGL REQUIRED)
#find_package (GLUT REQUIRED)

link_directories(ParticleToy/lib/freeglut ParticleToy/lib/lib_win64)

include_directories(${OPENGL_INCLUDE_DIR} ParticleToy/include ParticleToy/include/Eigen/)

add_executable(simulation_1 ${SOURCE_FILES})

target_link_libraries(simulation_1 ${OPENGL_LIBRARIES} freeglut_static glu32 opengl32 winmm gdi32)

MESSAGE( STATUS "CMAKE_CURRENT_SOURCE_DIR: " ${CMAKE_CURRENT_SOURCE_DIR} )

add_custom_command(TARGET simulation_1 POST_BUILD           # Adds a post-build event to simulation_1
        COMMAND ${CMAKE_COMMAND} -E copy_if_different       # which executes "cmake - E copy_if_different..."
        ${CMAKE_CURRENT_SOURCE_DIR}/ParticleToy/lib/freeglut/bin/freeglut.dll         # <--this is in-file
        $<TARGET_FILE_DIR:simulation_1>)                    # <--this is out-file path
