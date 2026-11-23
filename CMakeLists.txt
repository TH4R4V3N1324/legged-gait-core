cmake_minimum_required(VERSION 3.16)
project(gait_core VERSION 0.1.0 LANGUAGES CXX)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  set(GAIT_CORE_TOP_LEVEL ON)
else()
  set(GAIT_CORE_TOP_LEVEL OFF)
endif()

# Eigen is very slow unoptimised, so default to Release when building this repo on its own
get_property(GAIT_CORE_MULTI_CONFIG GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
if(GAIT_CORE_TOP_LEVEL AND NOT GAIT_CORE_MULTI_CONFIG AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(GAIT_CORE_USE_DOUBLE "Use double instead of float for Scalar (the ROS build turns this on)" OFF)
option(GAIT_CORE_BUNDLED_EIGEN "Always use third_party/eigen instead of looking for a system Eigen" OFF)
option(GAIT_CORE_BUILD_TESTS "Build the GoogleTest suite" ${GAIT_CORE_TOP_LEVEL})
set(GAIT_CORE_MAX_JOINTS 6 CACHE STRING "Maximum joints per leg (sizes the fixed-size joint arrays)")
set(GAIT_CORE_MAX_LEGS 6 CACHE STRING "Maximum legs per robot (sizes the fixed-size leg arrays)")

# ---------------------------------------------------------------------------
# Eigen: use the system copy when there is one (ROS, so every package in the process shares one
# Eigen version), otherwise the pinned submodule (Pico SDK, PlatformIO, Windows)
# ---------------------------------------------------------------------------
set(GAIT_CORE_EIGEN_FROM_SYSTEM OFF)
if(NOT TARGET Eigen3::Eigen AND NOT GAIT_CORE_BUNDLED_EIGEN AND NOT CMAKE_CROSSCOMPILING)
  find_package(Eigen3 3.3 QUIET NO_MODULE)
  if(TARGET Eigen3::Eigen)
    set(GAIT_CORE_EIGEN_FROM_SYSTEM ON)
  endif()
endif()

if(NOT TARGET Eigen3::Eigen)
  set(GAIT_CORE_EIGEN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen)
  if(NOT EXISTS ${GAIT_CORE_EIGEN_DIR}/Eigen/Dense)
    message(FATAL_ERROR "Eigen not found. Run: git submodule update --init --recursive")
  endif()
  add_library(gait_core_eigen INTERFACE)
  target_include_directories(gait_core_eigen SYSTEM INTERFACE
    $<BUILD_INTERFACE:${GAIT_CORE_EIGEN_DIR}>
    $<INSTALL_INTERFACE:include/gait_core/third_party/eigen>
  )
  set(GAIT_CORE_EIGEN_TARGET gait_core_eigen)
else()
  set(GAIT_CORE_EIGEN_TARGET Eigen3::Eigen)
endif()

# ---------------------------------------------------------------------------
# Gait library, by area (include/gait_core/<area>/, src/<area>/):
#   robot       what a robot is: legs, joint chains, stance, gaits (data only)
#   kinematics  solving one leg: the LegKinematics interface and its solvers
#   locomotion  making the robot walk: the engine, gait timing, trajectories, frames
#   estimation  what the robot measures: leg odometry
# ---------------------------------------------------------------------------
add_library(gait_core STATIC
  src/robot/gait_pattern.cpp
  src/kinematics/chain_kinematics.cpp
  src/kinematics/mass_properties.cpp
  src/kinematics/yaw_pitch_pitch_leg.cpp
  src/locomotion/balance_controller.cpp
  src/locomotion/gait_config.cpp
  src/locomotion/gait_engine.cpp
  src/locomotion/gait_engine_balance.cpp
  src/locomotion/gait_engine_commands.cpp
  src/locomotion/gait_engine_inputs.cpp
  src/locomotion/gait_engine_terrain.cpp
  src/locomotion/gait_engine_tick.cpp
  src/locomotion/gait_engine_walk.cpp
  src/locomotion/support_polygon.cpp
  src/locomotion/trajectory_generator.cpp
  src/estimation/body_velocity.cpp
  src/estimation/leg_odometry.cpp
)
add_library(gait_core::gait_core ALIAS gait_core)

target_include_directories(gait_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(gait_core PUBLIC ${GAIT_CORE_EIGEN_TARGET})
target_compile_features(gait_core PUBLIC cxx_std_17)

if(NOT GAIT_CORE_MAX_JOINTS MATCHES "^[1-9][0-9]*$")
  message(FATAL_ERROR "GAIT_CORE_MAX_JOINTS must be a positive integer, got '${GAIT_CORE_MAX_JOINTS}'")
endif()

target_compile_definitions(gait_core PUBLIC GAIT_CORE_MAX_JOINTS=${GAIT_CORE_MAX_JOINTS})

if(NOT GAIT_CORE_MAX_LEGS MATCHES "^[1-9][0-9]*$")
  message(FATAL_ERROR "GAIT_CORE_MAX_LEGS must be a positive integer, got '${GAIT_CORE_MAX_LEGS}'")
endif()

target_compile_definitions(gait_core PUBLIC GAIT_CORE_MAX_LEGS=${GAIT_CORE_MAX_LEGS})

if(GAIT_CORE_USE_DOUBLE)
  target_compile_definitions(gait_core PUBLIC GAIT_CORE_SCALAR=double)
endif()

if(MSVC)
  target_compile_definitions(gait_core PUBLIC _USE_MATH_DEFINES NOMINMAX)
endif()

# Linked into ROS shared libraries, so it must be position independent there
if(NOT CMAKE_CROSSCOMPILING)
  set_target_properties(gait_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

# ---------------------------------------------------------------------------
# Behaviour library: decisions from sensors (battery so far), on top of the gait's types.
# Exported as gait_core::behaviour.
# ---------------------------------------------------------------------------
add_library(gait_core_behaviour STATIC
  src/behaviour/battery_monitor.cpp
)
add_library(gait_core::behaviour ALIAS gait_core_behaviour)
set_target_properties(gait_core_behaviour PROPERTIES EXPORT_NAME behaviour)
target_link_libraries(gait_core_behaviour PUBLIC gait_core)
if(NOT CMAKE_CROSSCOMPILING)
  set_target_properties(gait_core_behaviour PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

# ---------------------------------------------------------------------------
# Robot definitions: real robots built on the gait core, which itself has none built in.
# Exported as gait_core::robots (gait_core/robots/explorer.hpp: gait_core::explorer::robot()).
# ---------------------------------------------------------------------------
add_library(gait_core_robots STATIC
  src/robots/explorer.cpp
)
add_library(gait_core::robots ALIAS gait_core_robots)
set_target_properties(gait_core_robots PROPERTIES EXPORT_NAME robots)
target_link_libraries(gait_core_robots PUBLIC gait_core)
if(NOT CMAKE_CROSSCOMPILING)
  set_target_properties(gait_core_robots PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

# ---------------------------------------------------------------------------
# Header-only, dependency-free libraries (no Eigen):
#   gait_core::protocol  the controller <-> robot packets (gait_core/protocol/packets.h, also valid C)
#   gait_core::teleop    the gamepad layout every stack shares (gait_core/teleop/gamepad_teleop.hpp)
# ---------------------------------------------------------------------------
add_library(gait_core_protocol INTERFACE)
add_library(gait_core::protocol ALIAS gait_core_protocol)
set_target_properties(gait_core_protocol PROPERTIES EXPORT_NAME protocol)
target_include_directories(gait_core_protocol INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(gait_core_teleop INTERFACE)
add_library(gait_core::teleop ALIAS gait_core_teleop)
set_target_properties(gait_core_teleop PROPERTIES EXPORT_NAME teleop)
target_include_directories(gait_core_teleop INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(gait_core_teleop INTERFACE cxx_std_17)

if(GAIT_CORE_TOP_LEVEL)
  foreach(target gait_core gait_core_behaviour gait_core_robots)
    if(MSVC)
      target_compile_options(${target} PRIVATE /W4)
    else()
      target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Wdouble-promotion)
    endif()
  endforeach()
endif()

# ---------------------------------------------------------------------------
# Install + CMake package config, so colcon/ROS can find_package(gait_core)
# ---------------------------------------------------------------------------
if(GAIT_CORE_TOP_LEVEL)
  include(GNUInstallDirs)
  include(CMakePackageConfigHelpers)

  set(GAIT_CORE_EXPORT_TARGETS gait_core gait_core_behaviour gait_core_robots gait_core_protocol gait_core_teleop)
  if(TARGET gait_core_eigen)
    list(APPEND GAIT_CORE_EXPORT_TARGETS gait_core_eigen)
    install(DIRECTORY ${GAIT_CORE_EIGEN_DIR}/Eigen
            DESTINATION include/gait_core/third_party/eigen)
  endif()

  install(TARGETS ${GAIT_CORE_EXPORT_TARGETS}
    EXPORT gait_coreTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    INCLUDES DESTINATION include
  )
  install(DIRECTORY include/ DESTINATION include)
  install(EXPORT gait_coreTargets
    NAMESPACE gait_core::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/gait_core
  )
  configure_package_config_file(cmake/gait_coreConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/gait_coreConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/gait_core
  )
  write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/gait_coreConfigVersion.cmake
    COMPATIBILITY SameMinorVersion
  )
  install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/gait_coreConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/gait_coreConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/gait_core
  )
  install(FILES package.xml DESTINATION share/gait_core)
endif()

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(GAIT_CORE_BUILD_TESTS)
  enable_testing()
  add_subdirectory(test)
endif()
