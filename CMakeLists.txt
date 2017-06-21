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
        ParticleToy/constraints/CircularWireConstraint.cpp
        ParticleToy/constraints/CircularWireConstraint.h
        ParticleToy/Particle.cpp
        ParticleToy/Particle.h
        #ParticleToy/ParticleToy.cpp
        ParticleToy/constraints/RodConstraint.cpp
        ParticleToy/constraints/RodConstraint.h
        ParticleToy/Solver.cpp
        ParticleToy/forces/SpringForce.cpp
        ParticleToy/forces/SpringForce.h
        ParticleToy/forces/Force.cpp
        ParticleToy/forces/Force.h
        ParticleToy/forces/GravityForce.cpp
        ParticleToy/forces/GravityForce.h
        ParticleToy/constraints/ConstraintForce.cpp
        ParticleToy/constraints/ConstraintForce.h
        ParticleToy/solvers/ForwardEuler.cpp
        ParticleToy/solvers/ForwardEuler.h
        ParticleToy/solvers/RK4.cpp
        ParticleToy/solvers/RK4.h
        ParticleToy/solvers/Midpoint.cpp
        ParticleToy/solvers/Midpoint.h
        ParticleToy/constraints/LambdaSolver.cpp
        ParticleToy/constraints/LambdaSolver.h
        ParticleToy/constraints/SlidingWireConstraint.cpp
        ParticleToy/constraints/SlidingWireConstraint.h
        ParticleToy/forces/HorizontalForce.cpp
        ParticleToy/forces/HorizontalForce.h
        ParticleToy/forces/WallForce.cpp
        ParticleToy/forces/WallForce.h
        ParticleToy/forces/AngularSpringForce.cpp
        ParticleToy/forces/AngularSpringForce.h
        ParticleToy/SetUp.cpp
        ParticleToy/SetUp.h
        ParticleToy/Stam/solver.cpp ParticleToy/Stam/solver.h ParticleToy/Stam/demo.cpp ParticleToy/Stam/demo.h ParticleToy/Stam/Marker.cpp ParticleToy/Stam/Marker.h ParticleToy/Stam/RigidBody.cpp ParticleToy/Stam/RigidBody.h ParticleToy/Stam/Macros.h ParticleToy/utility.cpp ParticleToy/utility.h ParticleToy/solvers/Solver.cpp ParticleToy/solvers/Solver.h ParticleToy/solvers/RigidForwardEuler.cpp ParticleToy/solvers/RigidForwardEuler.h ParticleToy/forces/MomentumForce.cpp ParticleToy/forces/MomentumForce.h)

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
