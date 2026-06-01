cmake_minimum_required(VERSION 3.16)
project(sumo_connection_probe LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(SUMO_SOURCE_DIR "" CACHE PATH "SUMO source or build tree containing src/libsumo/libtraci.h")
set(SUMO_LIBRARY_DIR "" CACHE PATH "Directory containing libtracicpp.so")

set(_sumo_include_hints)
if(SUMO_SOURCE_DIR)
  list(APPEND _sumo_include_hints
    "${SUMO_SOURCE_DIR}/src"
    "${SUMO_SOURCE_DIR}")
endif()
if(DEFINED ENV{SUMO_HOME})
  list(APPEND _sumo_include_hints
    "$ENV{SUMO_HOME}/src"
    "$ENV{SUMO_HOME}")
endif()

find_path(SUMO_LIBTRACI_INCLUDE_DIR
  NAMES libsumo/libtraci.h
  PATHS ${_sumo_include_hints}
  NO_DEFAULT_PATH)

if(NOT SUMO_LIBTRACI_INCLUDE_DIR)
  find_path(SUMO_LIBTRACI_INCLUDE_DIR NAMES libsumo/libtraci.h)
endif()

set(_sumo_library_hints)
if(SUMO_LIBRARY_DIR)
  list(APPEND _sumo_library_hints "${SUMO_LIBRARY_DIR}")
endif()
if(DEFINED ENV{SUMO_HOME})
  list(APPEND _sumo_library_hints
    "$ENV{SUMO_HOME}/bin"
    "$ENV{SUMO_HOME}/lib")
endif()
list(APPEND _sumo_library_hints
  /usr/lib/x86_64-linux-gnu
  /usr/lib)

find_library(SUMO_LIBTRACI_LIBRARY
  NAMES tracicpp libtracicpp
  PATHS ${_sumo_library_hints})

if(NOT SUMO_LIBTRACI_LIBRARY)
  message(FATAL_ERROR
    "Could not find libtracicpp.so. Install SUMO or pass "
    "-DSUMO_LIBRARY_DIR=/path/to/sumo/lib.")
endif()

if(NOT SUMO_LIBTRACI_INCLUDE_DIR)
  message(FATAL_ERROR
    "Found libtracicpp at '${SUMO_LIBTRACI_LIBRARY}', but could not find "
    "<libsumo/libtraci.h>. Provide a matching SUMO source/build tree with "
    "-DSUMO_SOURCE_DIR=/path/to/sumo, or set SUMO_HOME to a tree containing "
    "src/libsumo/libtraci.h.")
endif()

add_library(sumo_adapter STATIC src/sumo_adapter.cpp)
target_include_directories(sumo_adapter
  PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src"
  PRIVATE "${SUMO_LIBTRACI_INCLUDE_DIR}")
target_link_libraries(sumo_adapter PUBLIC "${SUMO_LIBTRACI_LIBRARY}")

add_executable(probe_libtraci src/probe_libtraci.cpp)
target_link_libraries(probe_libtraci PRIVATE sumo_adapter)

add_executable(probe_ego_shadow_sync src/probe_ego_shadow_sync.cpp)
target_link_libraries(probe_ego_shadow_sync PRIVATE sumo_adapter)
