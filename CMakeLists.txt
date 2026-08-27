# ik_service build entry point.
#
# Configures libpick_ik_core with the Python binding enabled and copies the
# resulting `pickik` module into ./dist/ so that `import pickik` works from
# this directory (tests + service use tests/conftest.py for the same).
#
#   cmake -S . -B build -G "Visual Studio 17 2022" -A x64 \
#     -Dpybind11_DIR=<(python -m pybind11 --cmake-dir)> \
#     -DFETCHCONTENT_SOURCE_DIR_EIGEN=.../.deps/eigen \
#     -DFETCHCONTENT_SOURCE_DIR_FMT=.../.deps/fmt
#   cmake --build build --config RelWithDebInfo
#
# The dependency pins are optional (FetchContent falls back to cloning when
# omitted); the service workspace keeps shallow clones under ../.deps.

cmake_minimum_required(VERSION 3.22)
project(ik_service_binding LANGUAGES CXX)

set(PICK_IK_CORE_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(PICK_IK_CORE_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(PICK_IK_CORE_BUILD_PYTHON ON CACHE BOOL "" FORCE)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../libpick_ik_core
                ${CMAKE_CURRENT_BINARY_DIR}/pick_ik_core)

# Land the module next to the service for a clean import. (A custom target,
# not add_custom_command(TARGET ...): the pickik target is defined in a
# subdirectory scope, which POST_BUILD custom commands cannot reference.)
add_custom_target(pickik_dist ALL
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_SOURCE_DIR}/dist
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:pickik>
            ${CMAKE_CURRENT_SOURCE_DIR}/dist/
    DEPENDS pickik
    COMMENT "Copying pickik module to ${CMAKE_CURRENT_SOURCE_DIR}/dist/")
