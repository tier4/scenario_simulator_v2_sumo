ament_auto_add_executable(external_npc_control_circular
  external_npc_control_circular.cpp
)
target_link_libraries(external_npc_control_circular cpp_scenario_node)
ament_target_dependencies(external_npc_control_circular external_control_plugin)

install(TARGETS
  external_npc_control_circular
  DESTINATION lib/cpp_mock_scenarios
)

# NOTE: intentionally not registered as a colcon test. Like external_npc_control_cross,
# this is a hold-open visual demo that needs the SUMO build of the plugin
# (-DEXTERNAL_CONTROL_PLUGIN_ENABLE_SUMO=ON) and a live SUMO process, neither of which
# is available in the default CI build.
