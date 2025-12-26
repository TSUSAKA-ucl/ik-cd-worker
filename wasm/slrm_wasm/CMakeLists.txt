cmake_minimum_required(VERSION 3.13)
project(SlrmWasmModule)

set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --bind")

# find_package(Eigen3 REQUIRED)
include_directories(${CMAKE_SOURCE_DIR}/../external/eigen)


# EMSDK の設定を通して emcc が使われている前提
set(CMAKE_EXECUTABLE_SUFFIX ".js")

add_executable(slrm_module
    ${CMAKE_SOURCE_DIR}/moveit_jacobian/src/cmd_vel_generator.cpp
    ${CMAKE_SOURCE_DIR}/moveit_jacobian/src/in_house_revolute_joint_model.cpp
    ${CMAKE_SOURCE_DIR}/moveit_jacobian/src/in_house_jacobian.cpp
    ${CMAKE_SOURCE_DIR}/moveit_jacobian/src/create_models_from_js.cpp
    ${CMAKE_SOURCE_DIR}/moveit_jacobian/src/log_functions_for_js.cpp
    ${CMAKE_SOURCE_DIR}/moveit_jacobian/src/bindings.cpp
)

add_custom_command(TARGET slrm_module PRE_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_SOURCE_DIR}/../../public/wasm"
    COMMENT "Creating output directory..."
)
set_target_properties(slrm_module PROPERTIES
    OUTPUT_NAME "slrm_module"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/../../public/wasm"
)

target_compile_options(slrm_module PRIVATE
    -O3
    -msimd128
    -DNDEBUG
)

target_link_options(slrm_module PRIVATE
  "-O3"
  "-msimd128"
  "-sMODULARIZE=1"
  "-sEXPORT_ES6=1"
  "-sENVIRONMENT=web,worker"
  "-sWASM=1"
  "--bind"
)
