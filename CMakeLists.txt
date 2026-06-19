cmake_minimum_required(VERSION 3.12)
project(g1_29dof_controller)

set(CMAKE_CXX_STANDARD 17)

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)
# Pinocchio: full-body forward kinematics, CoM and Jacobians for the footstep
# command generator (State_Footstep). Install via robotpkg or `apt install
# robotpkg-py3*-pinocchio` / conda `pinocchio`.
find_package(pinocchio REQUIRED)

# Pinocchio raises BOOST_MPL_LIMIT_*_SIZE to 30. Boost.MPL must then build its
# sequences on the fly (its preprocessed headers only go up to 20), otherwise
# Boost.Bimap (FSMStringMap) fails to instantiate. Define globally so every TU
# sees a consistent Boost.MPL configuration.
add_compile_definitions(
  BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
  BOOST_MPL_LIMIT_LIST_SIZE=30
  BOOST_MPL_LIMIT_VECTOR_SIZE=30
)

include_directories(
  /usr/include/eigen3
  /usr/local/include/ddscxx # dds
  /usr/local/include/iceoryx/v2.0.2 # dds
  include
  ${PROJECT_SOURCE_DIR}/thirdparty/onnxruntime-linux-x64-1.22.0/include
  ${PROJECT_SOURCE_DIR}/include/
)

link_libraries(
  unitree_sdk2 ddsc ddscxx rt pthread # dds
  libboost_program_options.a libyaml-cpp.a fmt
  ${PROJECT_SOURCE_DIR}/thirdparty/onnxruntime-linux-x64-1.22.0/lib/libonnxruntime.so.1.22.0
  pinocchio::pinocchio
)

file(GLOB_RECURSE ADD_SRC_LIST
  ${PROJECT_SOURCE_DIR}/src/*.cpp
)

add_library(${PROJECT_NAME}_lib ${ADD_SRC_LIST})
link_libraries(${PROJECT_NAME}_lib)

add_executable(g1_ctrl main.cpp)
