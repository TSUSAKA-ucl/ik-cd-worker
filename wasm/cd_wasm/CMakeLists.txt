# Collision Detection WASM Module
cmake_minimum_required(VERSION 3.13)
project(CdWasmModule)

set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --bind")

# EMSDK の設定を通して emcc が使われている前提
set(CMAKE_EXECUTABLE_SUFFIX ".js")

# find_package(Eigen3 REQUIRED) externalにcloneしたものを使う
set(SLRM_SOURCE_DIR ${CMAKE_SOURCE_DIR}/../slrm_wasm/moveit_jacobian/src)
set(CD_SOURCE_DIR gjk_worker/src)		    
add_executable(cd_module
  ${CD_SOURCE_DIR}/in_house_cd.cpp
  ${SLRM_SOURCE_DIR}/in_house_revolute_joint_model.cpp
  ${SLRM_SOURCE_DIR}/create_models_from_js.cpp
  ${CD_SOURCE_DIR}/log_functions_for_js.cpp
  ${CD_SOURCE_DIR}/bindings.cpp
  ${CD_SOURCE_DIR}/link_shapes_interface.cpp
)
target_include_directories(cd_module
  PUBLIC ${CMAKE_SOURCE_DIR}/../external/eigen
  PUBLIC ${CMAKE_SOURCE_DIR}/../slrm_wasm/moveit_jacobian/src
)

add_custom_command(TARGET cd_module PRE_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_SOURCE_DIR}/../../public/wasm"
    COMMENT "Creating output directory..."
)
set_target_properties(cd_module PROPERTIES
  OUTPUT_NAME "cd_module"
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/../../public/wasm"
)


target_compile_options(cd_module PRIVATE
    -O3
    -msimd128
    -DNDEBUG
)

target_link_options(cd_module PRIVATE
  "-O3"
  "-msimd128"
  "-sINITIAL_MEMORY=1GB"
  "-sMODULARIZE=1"
  "-sEXPORT_ES6=1"
  "-sEXPORTED_RUNTIME_METHODS=['HEAPF64','HEAP32']"
  "-sENVIRONMENT=web,worker,node"
  #  "-sEXPORT_NAME=CdModule"
  "-sWASM=1"
  "--bind"
)
