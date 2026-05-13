cmake_minimum_required(VERSION 3.20)
project(force_res_demos LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

get_filename_component(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

find_package(Boost REQUIRED)
find_package(Threads REQUIRED)

set(FORCE_RES_LIB "${CMAKE_CURRENT_SOURCE_DIR}/../lib/libforce_res.a")
if(NOT EXISTS "${FORCE_RES_LIB}")
  message(FATAL_ERROR "未找到 ${FORCE_RES_LIB}。请先在仓库根目录构建主静态库。")
endif()

# 与业务代码一致：通过仓库根 -I 解析 #include "rokae_header/rokae_force_controller.hpp"
set(DEMO_INCLUDES
    "${REPO_ROOT}"
    "${REPO_ROOT}/3rd/eigen"
    "${REPO_ROOT}/3rd/spdlog"
    "${REPO_ROOT}/3rd/kdl")

function(add_force_res_demo name)
  add_executable(${name} "${name}.cpp")
  target_compile_definitions(${name} PRIVATE SPDLOG_HEADER_ONLY FMT_HEADER_ONLY)
  target_include_directories(${name} PRIVATE ${DEMO_INCLUDES})
  target_link_libraries(${name} PRIVATE "${FORCE_RES_LIB}" Boost::headers Threads::Threads)
endfunction()

add_force_res_demo(joint_drag_test)
add_force_res_demo(joint_impedence_demo)
add_force_res_demo(cart_drag_test)
add_force_res_demo(cart_rot_drag_test)
add_force_res_demo(cart_impedence_demo)
add_force_res_demo(cross)
