cmake_minimum_required(VERSION 3.8)
project(external_control_plugin)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g -DBOOST_ALLOW_DEPRECATED_HEADERS)
endif()

option(EXTERNAL_CONTROL_PLUGIN_ENABLE_SUMO "Build prototype SUMO external NPC controller" OFF)
set(SUMO_SOURCE_DIR "" CACHE PATH "SUMO source or build tree containing src/libsumo/libtraci.h")
set(SUMO_LIBRARY_DIR "" CACHE PATH "Directory containing libtracicpp.so")
set(
  EXTERNAL_CONTROL_PLUGIN_DEFAULT_SUMOCFG
  "${CMAKE_CURRENT_SOURCE_DIR}/../sumo_connection/scenarios/straight/straight.sumocfg"
  CACHE FILEPATH "Default SUMO scenario for prototype sumo mode")

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/plugin.cpp
)

if(EXTERNAL_CONTROL_PLUGIN_ENABLE_SUMO)
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
      "EXTERNAL_CONTROL_PLUGIN_ENABLE_SUMO=ON but libtracicpp was not found. "
      "Install SUMO or pass -DSUMO_LIBRARY_DIR=/path/to/sumo/lib.")
  endif()

  if(NOT SUMO_LIBTRACI_INCLUDE_DIR)
    message(FATAL_ERROR
      "EXTERNAL_CONTROL_PLUGIN_ENABLE_SUMO=ON but <libsumo/libtraci.h> was not found. "
      "Pass -DSUMO_SOURCE_DIR=/path/to/sumo or set SUMO_HOME to a tree containing "
      "src/libsumo/libtraci.h.")
  endif()

  target_sources(${PROJECT_NAME} PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../sumo_connection/src/sumo_adapter.cpp")
  target_include_directories(${PROJECT_NAME} PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../sumo_connection/src"
    "${SUMO_LIBTRACI_INCLUDE_DIR}")
  target_link_libraries(${PROJECT_NAME} "${SUMO_LIBTRACI_LIBRARY}")
  target_compile_definitions(${PROJECT_NAME} PRIVATE
    EXTERNAL_CONTROL_PLUGIN_HAS_SUMO
    EXTERNAL_CONTROL_PLUGIN_DEFAULT_SUMOCFG="${EXTERNAL_CONTROL_PLUGIN_DEFAULT_SUMOCFG}")
endif()

pluginlib_export_plugin_description_file(traffic_simulator plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
