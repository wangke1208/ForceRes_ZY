cmake_minimum_required(VERSION 3.20)

project(ForceRes_ZY LANGUAGES CXX)

# 禁止 FetchContent 等在配置期访问网络（本工程也不使用 FetchContent）
set(FETCHCONTENT_FULLY_DISCONNECTED ON CACHE BOOL "Disallow FetchContent network access" FORCE)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 仅使用仓库内 3rd/eigen；不完整则配置失败，不尝试下载
set(EIGEN3_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/3rd/eigen")
if(NOT EXISTS "${EIGEN3_INCLUDE_DIR}/Eigen/Core")
  message(
    FATAL_ERROR
      "缺少 Eigen 头文件: \"${EIGEN3_INCLUDE_DIR}/Eigen/Core\"。\n"
      "请将完整 Eigen 的 Eigen/ 目录放入 3rd/eigen/ 后再运行 cmake。\n"
      "本工程 CMake 不会在配置阶段联网获取依赖。")
endif()
if(NOT EXISTS "${EIGEN3_INCLUDE_DIR}/Eigen/src/Core/util/Macros.h")
  message(
    FATAL_ERROR
      "缺少 Eigen 头文件: \"${EIGEN3_INCLUDE_DIR}/Eigen/src/Core/util/Macros.h\"。\n"
      "当前 3rd/eigen 不是完整 Eigen 源码树，请补全后再 cmake。")
endif()

find_package(Boost REQUIRED)
find_package(Threads REQUIRED)

# 仅编译业务与 KDL，不包含 example/ 下任何源码
file(GLOB_RECURSE ROKAE_CXX CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/rokae_src/*.cpp")
file(GLOB_RECURSE KDL_CXX CONFIGURE_DEPENDS
  "${CMAKE_SOURCE_DIR}/3rd/kdl/*.cpp"
  "${CMAKE_SOURCE_DIR}/3rd/kdl/*.cxx"
)

add_library(force_res STATIC ${ROKAE_CXX} ${KDL_CXX})

# 静态库输出到 example/lib（模拟对外发布目录）
set_target_properties(
  force_res
  PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/example/lib"
             OUTPUT_NAME "force_res")

# spdlog/fmt：避免链接期拉 libfmt；KDL 常量定义在 utilities/*.cxx
target_compile_definitions(
  force_res
  PUBLIC SPDLOG_HEADER_ONLY FMT_HEADER_ONLY
)

target_include_directories(
  force_res
  PUBLIC "${CMAKE_SOURCE_DIR}"
         "${CMAKE_SOURCE_DIR}/rokae_header"
         "${EIGEN3_INCLUDE_DIR}"
         "${CMAKE_SOURCE_DIR}/3rd/spdlog"
         "${CMAKE_SOURCE_DIR}/3rd/kdl"
)

if(TARGET Boost::headers)
  target_link_libraries(force_res PUBLIC Boost::headers Threads::Threads)
else()
  target_include_directories(force_res PUBLIC ${Boost_INCLUDE_DIRS})
  target_link_libraries(force_res PUBLIC Threads::Threads)
endif()

# 根工程不构建 example 内可执行文件；示例见 example/demos
