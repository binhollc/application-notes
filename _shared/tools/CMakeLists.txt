# Build the AN0013 replay utility against an installed CosmicSDK.
#
#   cmake -B build -DCOSMICSDK_ROOT=/path/to/cosmicsdk
#   cmake --build build
#   ./build/i2c_replay selfcheck
#
# COSMICSDK_ROOT is the directory holding include/ and lib/. On a default
# install that is /opt/cosmicsdk on Linux, /usr/local/cosmicsdk on macOS, and
# the install directory on Windows. Building the SDK from source instead leaves
# the same layout under its staging/ directory.

cmake_minimum_required(VERSION 3.16)
project(an0013_i2c_replay CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(COSMICSDK_ROOT "" CACHE PATH "Directory containing CosmicSDK include/ and lib/")
if(NOT COSMICSDK_ROOT)
  message(FATAL_ERROR
    "Set COSMICSDK_ROOT to the CosmicSDK install or staging directory, "
    "e.g. cmake -B build -DCOSMICSDK_ROOT=/opt/cosmicsdk")
endif()

find_path(COSMICSDK_INCLUDE_DIR CosmicSDK.hpp
          HINTS "${COSMICSDK_ROOT}/include" NO_DEFAULT_PATH REQUIRED)
find_library(COSMICSDK_LIBRARY NAMES cosmicsdk
             HINTS "${COSMICSDK_ROOT}/lib" NO_DEFAULT_PATH REQUIRED)

# CosmicSDK.hpp includes <hidapi.h>, which the SDK ships next to its own
# headers on an install and under its dependency tree in a source build.
find_path(HIDAPI_INCLUDE_DIR hidapi.h
          HINTS "${COSMICSDK_ROOT}/include" "${COSMICSDK_ROOT}/include/hidapi"
          REQUIRED)

add_executable(i2c_replay i2c_replay.cpp)
target_include_directories(i2c_replay PRIVATE
                           "${COSMICSDK_INCLUDE_DIR}" "${HIDAPI_INCLUDE_DIR}")
target_link_libraries(i2c_replay PRIVATE "${COSMICSDK_LIBRARY}")

if(UNIX AND NOT APPLE)
  # A static libcosmicsdk pulls these in itself; a shared one already has them.
  find_package(Threads REQUIRED)
  target_link_libraries(i2c_replay PRIVATE Threads::Threads)
endif()
