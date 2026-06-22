ament_auto_add_executable(external_npc_control_cross
  external_npc_control_cross.cpp
)
target_link_libraries(external_npc_control_cross cpp_scenario_node)
ament_target_dependencies(external_npc_control_cross external_control_plugin)

install(TARGETS
  external_npc_control_cross
  DESTINATION lib/cpp_mock_scenarios
)

# NOTE: intentionally not registered as a colcon test. This demo needs the SUMO
# build of the plugin (-DEXTERNAL_CONTROL_PLUGIN_ENABLE_SUMO=ON) and a live SUMO
# process, neither of which is available in the default CI build. The dummy-mode
# external_npc_control test remains the CI guard for the external-control path.
