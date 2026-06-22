// Copyright 2015 TIER IV, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Circular SUMO external-NPC demo (Lanelet2 ring-road test course, for visual inspection).
//
// Runs the live SUMO path on the synthetic circular test course
// (simulation/sumo_connection/scenarios/circular). SUMO controls npc_0/npc_1/npc_2,
// which circulate the closed ring indefinitely (the route `repeat`s); the
// external_control_plugin mirrors them back into traffic_simulator so they show up
// in RViz while sumo-gui shows the same vehicles looping the ring.
//
// Unlike the cross demo there is no Ego interaction here: the course is a pure
// traffic-flow loop, so run it with EXTERNAL_CONTROL_SUMO_EGO_SHADOW=off.
//
// This is a hold-open viewing demo: it keeps running (and keeps the GUIs alive) for
// the whole window rather than asserting a pass/fail condition and tearing down.
//
// Requires the SUMO build of the plugin (-DEXTERNAL_CONTROL_PLUGIN_ENABLE_SUMO=ON)
// and these env vars (see external_control_plugin/README.md):
//   EXTERNAL_CONTROL_NPC_MODE=sumo
//   EXTERNAL_CONTROL_SUMOCFG=.../scenarios/circular/circular.sumocfg
//   EXTERNAL_CONTROL_SUMO_ROUTE_ID=loop
//   EXTERNAL_CONTROL_SUMO_EDGE_ID=lanelet_1
//   EXTERNAL_CONTROL_SUMO_EGO_SHADOW=off
// Because it needs a live SUMO it is run manually and is not a colcon test.

#include <cmath>
#include <cpp_mock_scenarios/catalogs.hpp>
#include <cpp_mock_scenarios/cpp_scenario_node.hpp>
#include <cstdlib>
#include <external_control_plugin/plugin.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <traffic_simulator/api/api.hpp>

namespace cpp_mock_scenarios
{
class ExternalNpcControlCircularScenario : public cpp_mock_scenarios::CppScenarioNode
{
public:
  explicit ExternalNpcControlCircularScenario(const rclcpp::NodeOptions & option)
  : cpp_mock_scenarios::CppScenarioNode(
      "external_npc_control_circular",
      "/home/koki/sumo_ws/src/scenario_simulator_v2/simulation/sumo_connection/scenarios/circular",
      "lanelet2_map.osm", __FILE__, false, option)
  {
    start();
  }

private:
  void onInitialize() override
  {
    entity_behavior::ExternalNpcLifecycleBridge::reset();

    // Hidden manager whose external_control behavior bootstraps the SUMO bridge.
    // Once the NPCs exist they hold the shared SUMO controller alive, so the manager
    // is despawned (see onUpdate) and does not linger as a stray car in RViz.
    api_.spawn(
      lifecycle_manager_name_, lifecycle_manager_pose_, getVehicleParameters(),
      traffic_simulator::entity::VehicleEntity::BuiltinBehavior::externalControl());

    RCLCPP_INFO(
      get_logger(),
      "spawned SUMO lifecycle manager entity=%s; npc_0/npc_1/npc_2 will be "
      "auto-spawned from SUMO vehicle appearances and circulate the ring",
      lifecycle_manager_name_.c_str());
  }

  void onUpdate() override
  {
    applySumoLifecycleRequests();

    const int npc_count = static_cast<int>(api_.isEntityExist("npc_0")) +
                          static_cast<int>(api_.isEntityExist("npc_1")) +
                          static_cast<int>(api_.isEntityExist("npc_2"));

    // Once the NPCs are up (and holding the shared SUMO controller alive), the
    // bootstrap manager is no longer needed; despawn it so it does not show up in
    // RViz as a parked car in the middle of the ring.
    if (npc_count == 3 && !manager_despawned_ && api_.isEntityExist(lifecycle_manager_name_)) {
      api_.despawn(lifecycle_manager_name_);
      manager_despawned_ = true;
      RCLCPP_INFO(
        get_logger(),
        "all 3 SUMO NPCs present and circulating; despawned bootstrap manager entity=%s "
        "(NPCs keep the shared SUMO controller alive). Holding open until time=%.0f s "
        "(watch sumo-gui + RViz)",
        lifecycle_manager_name_.c_str(), timeout_s_);
    }

    if (api_.getCurrentTime() >= timeout_s_) {
      stop(cpp_mock_scenarios::Result::SUCCESS, "circular SUMO demo hold window elapsed");
    }
  }

  static auto poseFromExternalState(const entity_behavior::ExternalVehicleState & state)
    -> geometry_msgs::msg::Pose
  {
    return geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(
        geometry_msgs::build<geometry_msgs::msg::Point>().x(state.x_m).y(state.y_m).z(state.z_m))
      .orientation(geometry_msgs::build<geometry_msgs::msg::Quaternion>()
                     .x(0.0)
                     .y(0.0)
                     .z(std::sin(state.yaw_rad * 0.5))
                     .w(std::cos(state.yaw_rad * 0.5)));
  }

  void spawnExternalControlNpc(
    const std::string & name, const geometry_msgs::msg::Pose & pose, const std::string & model)
  {
    api_.spawn(
      name, pose, getVehicleParameters(),
      traffic_simulator::entity::VehicleEntity::BuiltinBehavior::externalControl(), model);
  }

  void applySumoLifecycleRequests()
  {
    const auto snapshot = entity_behavior::ExternalNpcLifecycleBridge::getSnapshot();
    for (const auto & request : snapshot.spawn_requests) {
      if (request.scenario_entity_name == lifecycle_manager_name_) {
        continue;
      }
      if (api_.isEntityExist(request.scenario_entity_name)) {
        continue;
      }
      spawnExternalControlNpc(
        request.scenario_entity_name, poseFromExternalState(request.initial_state),
        request.default_vehicle_model);
      RCLCPP_INFO(
        get_logger(),
        "SUMO lifecycle spawned scenario NPC sumo_id=%s type_id=%s scenario_entity=%s",
        request.sumo_vehicle_id.c_str(), request.sumo_type_id.c_str(),
        request.scenario_entity_name.c_str());
    }

    for (const auto & request : snapshot.despawn_requests) {
      if (request.scenario_entity_name == lifecycle_manager_name_) {
        continue;
      }
      if (!api_.isEntityExist(request.scenario_entity_name)) {
        continue;
      }
      const auto despawned = api_.despawn(request.scenario_entity_name);
      RCLCPP_INFO(
        get_logger(), "SUMO lifecycle despawn sumo_id=%s scenario_entity=%s despawned=%s",
        request.sumo_vehicle_id.c_str(), request.scenario_entity_name.c_str(),
        despawned ? "true" : "false");
    }
  }

  bool manager_despawned_ = false;
  // Long viewing window; the launch's global_timeout bounds the run in practice.
  const double timeout_s_ = 300.0;

  const std::string lifecycle_manager_name_{"__sumo_lifecycle_manager"};

  // Ring centre (the course is centred at the SUMO origin, radius ~50 m).
  const geometry_msgs::msg::Pose lifecycle_manager_pose_ =
    geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(0.0).y(0.0).z(0.0))
      .orientation(
        geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0.0).y(0.0).z(0.0).w(1.0));
};
}  // namespace cpp_mock_scenarios

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<cpp_mock_scenarios::ExternalNpcControlCircularScenario>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
