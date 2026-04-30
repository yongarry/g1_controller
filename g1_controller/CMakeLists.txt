cmake_minimum_required(VERSION 3.5)
project(g1_controller VERSION 1.0.0)

## Set compiler to use c++ 17 features
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)
find_package(pinocchio REQUIRED)

add_executable(g1_ctl src/main.cpp src/g1_controller.cpp)
target_include_directories(g1_ctl PRIVATE include)
target_link_libraries(g1_ctl unitree_sdk2 pinocchio::pinocchio)

add_executable(g1_state_estimator src/state_estimator.cpp)
target_link_libraries(g1_state_estimator unitree_sdk2)
