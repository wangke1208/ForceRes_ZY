cmake_minimum_required(VERSION 3.20)

get_filename_component(EXAMPLE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)

set(DEMO_INCLUDES
    "${EXAMPLE_ROOT}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${EXAMPLE_ROOT}/3rd/eigen"
    "${EXAMPLE_ROOT}/3rd/spdlog")

function(add_force_res_demo name)
  add_executable(${name} "${name}.cpp")
  target_compile_definitions(${name} PRIVATE SPDLOG_HEADER_ONLY FMT_HEADER_ONLY)
  target_include_directories(${name} PRIVATE ${DEMO_INCLUDES})
  target_link_libraries(${name} PRIVATE "${EXAMPLE_ROOT}/lib/libforce_res.a" Boost::headers Threads::Threads)
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_link_options(${name} PRIVATE -Wl,--gc-sections)
  endif()
endfunction()

add_force_res_demo(joint_drag_test)
add_force_res_demo(joint_drag_test_with_load)
