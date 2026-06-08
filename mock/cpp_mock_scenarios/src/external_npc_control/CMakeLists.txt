ament_auto_add_executable(external_npc_control
  external_npc_control.cpp
)
target_link_libraries(external_npc_control cpp_scenario_node)

install(TARGETS
  external_npc_control
  DESTINATION lib/cpp_mock_scenarios
)

if(BUILD_TESTING)
  include(../../cmake/add_cpp_mock_scenario_test.cmake)
  add_cpp_mock_scenario_test(${PROJECT_NAME} "external_npc_control" "5.0")
endif()
