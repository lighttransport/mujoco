# Copyright 2025 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/wasm/dist")

set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/wasm)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR})

link_directories(${CMAKE_BINARY_DIR}/lib)

file(GLOB MUJOCO_WASM_FILES
    "codegen/generated/*.cc"
    "unpack.cc"
)

set(MUJOCO_WASM_PHYSICS_FILES
    "physics_bindings.cc"
)

if(NOT MUJOCO_WASM_FILES)
    message(FATAL_ERROR "No source files found in codegen/generated/")
endif()

# Set Emscripten linker flags
set(EMCC_COMMON_LINKER_FLAGS
    "--bind"
    "-s ALLOW_MEMORY_GROWTH=1"
    "-s EXPORT_ES6=1"
    "-s MODULARIZE=1"
)

set(EMCC_FULL_LINKER_FLAGS
    ${EMCC_COMMON_LINKER_FLAGS}
    "-s ASSERTIONS=1"
    "-s DISABLE_EXCEPTION_CATCHING=0"
    "-s FORCE_FILESYSTEM=1"
    "-s EXPORTED_RUNTIME_METHODS=['ccall','cwrap','FS','MEMFS']"
    "-s EXPORT_NAME=loadMujoco"
    "-gsource-map"
    "-g"
    "--emit-tsd mujoco.d.ts"
)
if(MUJOCO_WASM_THREADS)
    list(APPEND EMCC_FULL_LINKER_FLAGS
        "-pthread"
        "-s PTHREAD_POOL_SIZE=navigator.hardwareConcurrency"
    )
endif()
string(REPLACE ";" " " EMCC_FULL_LINKER_FLAGS_STR "${EMCC_FULL_LINKER_FLAGS}")

set(EMCC_PHYSICS_LINKER_FLAGS
    ${EMCC_COMMON_LINKER_FLAGS}
    "-Oz"
    "-flto"
    "-s ASSERTIONS=0"
    "-s FILESYSTEM=0"
    "-s SUPPORT_LONGJMP=0"
    "-s MALLOC=emmalloc"
    "-s DISABLE_EXCEPTION_CATCHING=1"
    "-s EXPORTED_RUNTIME_METHODS=[]"
    "-s EXPORT_NAME=loadMujocoPhysics"
    "--closure=1"
    "--emit-tsd mujoco_physics.d.ts"
)
if(MUJOCO_PHYSICS_ENABLE_EXCEPTIONS)
    list(REMOVE_ITEM EMCC_PHYSICS_LINKER_FLAGS "-s DISABLE_EXCEPTION_CATCHING=1")
    list(REMOVE_ITEM EMCC_PHYSICS_LINKER_FLAGS "-s SUPPORT_LONGJMP=0")
    list(APPEND EMCC_PHYSICS_LINKER_FLAGS "-s DISABLE_EXCEPTION_CATCHING=0")
endif()
if(MUJOCO_PHYSICS_ENABLE_THREADS)
    list(APPEND EMCC_PHYSICS_LINKER_FLAGS
        "-pthread"
        "-s PTHREAD_POOL_SIZE=navigator.hardwareConcurrency"
    )
endif()
string(REPLACE ";" " " EMCC_PHYSICS_LINKER_FLAGS_STR "${EMCC_PHYSICS_LINKER_FLAGS}")

add_executable(mujoco_wasm ${MUJOCO_WASM_FILES})
target_compile_options(mujoco_wasm PRIVATE -fexceptions)
if(MUJOCO_WASM_THREADS)
    target_compile_options(mujoco_wasm PRIVATE -pthread)
endif()

# Keep the internal target name distinct to avoid colliding with the native
# `mujoco` library target, but emit artifacts named `mujoco.*` by setting the
# output name. Also apply the emscripten linker flags to the wasm target.
set_target_properties(mujoco_wasm PROPERTIES
    LINK_FLAGS "${EMCC_FULL_LINKER_FLAGS_STR}"
    OUTPUT_NAME "mujoco"
)

target_link_libraries(mujoco_wasm ccd lodepng mujoco tinyxml2 qhullstatic_r obj_decoder stl_decoder)

if(MUJOCO_BUILD_PHYSICS_WASM)
    add_executable(mujoco_physics_wasm ${MUJOCO_WASM_PHYSICS_FILES})
    target_compile_options(mujoco_physics_wasm PRIVATE -Oz -flto)
    if(MUJOCO_PHYSICS_ENABLE_EXCEPTIONS)
        target_compile_options(mujoco_physics_wasm PRIVATE -fexceptions)
    else()
        target_compile_options(mujoco_physics_wasm PRIVATE -fno-exceptions)
    endif()
    if(MUJOCO_PHYSICS_ENABLE_THREADS)
        target_compile_options(mujoco_physics_wasm PRIVATE -pthread)
    endif()
    set_target_properties(mujoco_physics_wasm PROPERTIES
        LINK_FLAGS "${EMCC_PHYSICS_LINKER_FLAGS_STR}"
        OUTPUT_NAME "mujoco_physics"
    )
    target_link_libraries(
        mujoco_physics_wasm
        ccd
        mujoco_physics_core
        qhullstatic_r
    )
    # Copy WASM artifacts to the web demo's public/ folder after build.
    set(MUJOCO_WEB_DEMO_DIR "${CMAKE_SOURCE_DIR}/web/public")
    add_custom_command(TARGET mujoco_physics_wasm POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory "${MUJOCO_WEB_DEMO_DIR}"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_SOURCE_DIR}/wasm/dist/mujoco_physics.js"
            "${CMAKE_SOURCE_DIR}/wasm/dist/mujoco_physics.wasm"
            "${MUJOCO_WEB_DEMO_DIR}"
        COMMENT "Copying mujoco_physics.{js,wasm} to web/public/"
    )
endif()

install(TARGETS mujoco_wasm DESTINATION ${DIVISIBLE_INSTALL_BIN_DIR})

if(MUJOCO_BUILD_PHYSICS_WASM)
    install(TARGETS mujoco_physics_wasm DESTINATION ${DIVISIBLE_INSTALL_BIN_DIR})
endif()
