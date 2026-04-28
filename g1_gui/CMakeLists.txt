cmake_minimum_required(VERSION 3.16)
project(g1_gui VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)

add_library(mongoose STATIC third_party/mongoose/mongoose.c)
target_include_directories(mongoose PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/third_party/mongoose)

add_executable(g1_gui_server src/main.cpp)
target_include_directories(g1_gui_server PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(g1_gui_server PRIVATE mongoose unitree_sdk2 pthread)
target_compile_definitions(
  g1_gui_server
  PRIVATE G1_GUI_WEB_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/web"
)
