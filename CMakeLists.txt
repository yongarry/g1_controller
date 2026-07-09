cmake_minimum_required(VERSION 3.12)
project(g1_controller)

set(CMAKE_CXX_STANDARD 17)

# --- ONNX Runtime: 빌드 머신 아키텍처에 맞는 배포본 선택 ---
# x86 개발 머신과 aarch64(Jetson) 배포 머신에서 동일 소스로 빌드되게 함
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(ONNX_DIR ${PROJECT_SOURCE_DIR}/thirdparty/onnxruntime-linux-aarch64-1.22.0)
else()
  set(ONNX_DIR ${PROJECT_SOURCE_DIR}/thirdparty/onnxruntime-linux-x64-1.22.0)
endif()

if(NOT EXISTS ${ONNX_DIR})
  message(FATAL_ERROR "ONNX Runtime not found: ${ONNX_DIR}\n"
                      "이 아키텍처(${CMAKE_SYSTEM_PROCESSOR})용 배포본을 thirdparty/에 풀어주세요.")
endif()
message(STATUS "Using ONNX Runtime: ${ONNX_DIR}")

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)
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
  include
  ${ONNX_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/
)

link_libraries(
  unitree_sdk2 rt pthread
  libboost_program_options.a libyaml-cpp.a fmt
  ${ONNX_DIR}/lib/libonnxruntime.so.1.22.0
  pinocchio::pinocchio
)

file(GLOB_RECURSE ADD_SRC_LIST
  ${PROJECT_SOURCE_DIR}/src/*.cpp
)

add_library(${PROJECT_NAME}_lib ${ADD_SRC_LIST})
link_libraries(${PROJECT_NAME}_lib)

add_executable(g1_ctrl main.cpp)