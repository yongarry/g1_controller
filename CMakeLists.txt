cmake_minimum_required(VERSION 3.16)

# Superbuild to build both controller and GUI in one configure/build.
project(g1_stack VERSION 1.0.0 LANGUAGES C CXX)

# Keep outputs a bit nicer when building from a single build dir.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_subdirectory(g1_controller)
add_subdirectory(g1_gui)

