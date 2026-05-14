cmake_minimum_required(VERSION 3.20)

# 自包含示例包：复制本目录到任意位置（需已含 3rd/、lib/、include/），在同版本 GCC + Boost 环境下即可配置构建。
project(force_res_example_sdk LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(EIGEN3_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rd/eigen")
if(NOT EXISTS "${EIGEN3_INCLUDE_DIR}/Eigen/Core")
  message(
    FATAL_ERROR
      "缺少 Eigen: \"${EIGEN3_INCLUDE_DIR}/Eigen/Core\"。\n"
      "请将 Eigen 源码树放到本包目录 3rd/eigen/（与交付说明一致）。")
endif()
if(NOT EXISTS "${EIGEN3_INCLUDE_DIR}/Eigen/src/Core/util/Macros.h")
  message(FATAL_ERROR "Eigen 不完整，请补全 3rd/eigen/ 后再 cmake。")
endif()

find_package(Boost REQUIRED)
find_package(Threads REQUIRED)

set(FORCE_RES_LIB "${CMAKE_CURRENT_SOURCE_DIR}/lib/libforce_res.a")
if(NOT EXISTS "${FORCE_RES_LIB}")
  message(
    FATAL_ERROR
      "未找到 ${FORCE_RES_LIB}。\n"
      "请将 libforce_res.a 放到本包 lib/ 目录（与交付说明一致）。")
endif()

add_subdirectory(demos)
