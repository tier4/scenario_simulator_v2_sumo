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

// Cross-map SUMO external-NPC demo ("Recommended Minimum Demo", item A).
//
// Runs the live SUMO path on the converted Lanelet2 cross map (simple_cross_map):
//   * a real, moving Ego drives the route lanelet 7 -> 147 -> 154; the
//     external_control_plugin mirrors it into SUMO as the ego shadow (this is the
//     "real ego coupling" -- no static fallback, since the plugin reads the live
//     "ego" entity from OtherEntityStatus);
//   * SUMO controls npc_0 (route, major road) and npc_cross (crossing, minor road);
//   * npc_cross yields to the route traffic at the j_cross intersection and is
//     mirrored back into traffic_simulator, demonstrating NPCs reacting to the
//     intersection.
//
// Requires the SUMO build of the plugin (-DEXTERNAL_CONTROL_PLUGIN_ENABLE_SUMO=ON)
// and these env vars (see external_control_plugin/README.md):
//   EXTERNAL_CONTROL_NPC_MODE=sumo
//   EXTERNAL_CONTROL_SUMOCFG=.../lanelet2_simple_cross/intersection.sumocfg
//   EXTERNAL_CONTROL_SUMO_ROUTE_ID=straight_route
//   EXTERNAL_CONTROL_SUMO_EDGE_ID=lanelet_7
// Coordinate transform offsets are left at 0 (the SUMO net shares the Lanelet2
// local frame). Because it needs a live SUMO, this scenario is run manually and is
// not registered as a mandatory colcon test.

#include <ament_index_cpp/get_package_share_directory.hpp>
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
#include <vector>

namespace cpp_mock_scenarios
{
class ExternalNpcControlCrossScenario : public cpp_mock_scenarios::CppScenarioNode
{
public:
  explicit ExternalNpcControlCrossScenario(const rclcpp::NodeOptions & option)
  : cpp_mock_scenarios::CppScenarioNode(
      "external_npc_control_cross",
      ament_index_cpp::get_package_share_directory("simple_cross_map") + "/map", "lanelet2_map.osm",
      __FILE__, false, option)
  {
    start();
  }

private:
  void onInitialize() override
  {
    entity_behavior::ExternalNpcLifecycleBridge::reset();

    // Hidden manager whose external_control behavior drives the SUMO bridge.
    api_.spawn(
      lifecycle_manager_name_, lifecycle_manager_pose_, getVehicleParameters(),
      traffic_simulator::entity::VehicleEntity::BuiltinBehavior::externalControl());

    // Real, moving Ego on the route (lanelet 7 -> 147 -> 154). A behavior-tree
    // vehicle named "ego" is enough here: the SUMO controller reads it from
    // OtherEntityStatus by EXTERNAL_CONTROL_SUMO_EGO_ID (default "ego") and mirrors
    // its live pose into SUMO, replacing the static fallback ego.
    auto & ego_entity = api_.spawn(
      "ego", traffic_simulator::helper::constructCanonicalizedLaneletPose(7, 30.0, 0.0),
      getVehicleParameters());
    ego_entity.setLinearVelocity(2.5);
    ego_entity.requestSpeedChange(2.5, true);
    std::vector<geometry_msgs::msg::Pose> goal_poses;
    goal_poses.emplace_back(traffic_simulator::pose::toMapPose(
      traffic_simulator::helper::constructCanonicalizedLaneletPose(147, 5.0, 0.0)));
    goal_poses.emplace_back(traffic_simulator::pose::toMapPose(
      traffic_simulator::helper::constructCanonicalizedLaneletPose(154, 10.0, 0.0)));
    ego_entity.requestAssignRoute(goal_poses);

    RCLCPP_INFO(
      get_logger(),
      "spawned moving ego on lanelet 7 and SUMO lifecycle manager entity=%s; "
      "npc_0/npc_cross will be auto-spawned from SUMO vehicle appearances",
      lifecycle_manager_name_.c_str());
  }

  void onUpdate() override
  {
    applySumoLifecycleRequests();

    if (api_.isEntityExist("ego") && api_.getEntity("ego").getCurrentTwist().linear.x > 0.1) {
      ego_moved_ = true;
    }

    if (api_.isEntityExist("npc_cross")) {
      const auto speed = api_.getEntity("npc_cross").getCurrentTwist().linear.x;
      // A genuine intersection yield is: npc_cross gets underway, then decelerates
      // to (near) a stop while route traffic crosses, then proceeds. The
      // "was moving first" guard avoids a false positive from the zero velocity at
      // the spawn tick.
      if (speed > 1.5) {
        npc_cross_was_moving_ = true;
      }
      if (npc_cross_was_moving_ && speed < 0.5) {
        npc_cross_yielded_ = true;
      }
      if (npc_cross_yielded_ && speed > 1.0) {
        npc_cross_resumed_ = true;
      }
    }

    // Demo succeeds once the real ego has moved, both SUMO NPCs are present and
    // externally controlled, and npc_cross has yielded at the intersection then
    // proceeded once the route was clear.
    const bool demo_passed =
      ego_moved_ && api_.isEntityExist("npc_0") && api_.isEntityExist("npc_cross") &&
      api_.getEntity("npc_0").getCurrentAction() == "external_control" &&
      api_.getEntity("npc_cross").getCurrentAction() == "external_control" && npc_cross_yielded_ &&
      npc_cross_resumed_;

    if (demo_passed) {
      // When EXTERNAL_CONTROL_CROSS_HOLD is set, keep the simulation running after
      // the yield is observed so the GUIs (sumo-gui / RViz) stay alive for visual
      // inspection instead of tearing down ~20 s in. Otherwise stop immediately so
      // the scenario behaves as a normal pass/fail run.
      if (holdOpen()) {
        if (!hold_logged_) {
          RCLCPP_INFO(
            get_logger(),
            "cross SUMO demo PASSED (npc_cross yielded then resumed); holding open for "
            "inspection until time=%.0f s (EXTERNAL_CONTROL_CROSS_HOLD set)",
            timeout_s_);
          hold_logged_ = true;
        }
        return;
      }
      stop(cpp_mock_scenarios::Result::SUCCESS);
      return;
    }

    if (api_.getCurrentTime() >= timeout_s_) {
      stop(
        holdOpen() ? cpp_mock_scenarios::Result::SUCCESS : cpp_mock_scenarios::Result::FAILURE,
        holdOpen() ? "cross SUMO demo hold window elapsed"
                   : "cross SUMO demo did not reach the expected intersection-yield state in time");
    }
  }

  static auto holdOpen() -> bool
  {
    const char * value = std::getenv("EXTERNAL_CONTROL_CROSS_HOLD");
    return value != nullptr && std::string(value) != "" && std::string(value) != "0" &&
           std::string(value) != "false";
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

  bool ego_moved_ = false;
  bool npc_cross_was_moving_ = false;
  bool npc_cross_yielded_ = false;
  bool npc_cross_resumed_ = false;
  bool hold_logged_ = false;
  // Normal run: fail if the yield is not observed within 60 s. Hold-open run
  // (EXTERNAL_CONTROL_CROSS_HOLD): keep the sim/GUIs alive much longer for viewing.
  const double timeout_s_ = holdOpen() ? 300.0 : 60.0;

  const std::string lifecycle_manager_name_{"__sumo_lifecycle_manager"};

  // The hidden lifecycle-manager entity is placed *near the demo scene* (MGRS frame,
  // ~50 m south of the j_cross intersection), NOT off-map at (-1000,-1000). RViz's
  // default view follows the "entities" frame, which is the centroid of all spawned
  // entities; an off-map manager drags that centroid halfway to (-1000,-1000) and
  // points the camera at empty space. Keeping it on the map lets the default RViz
  // config frame the scene correctly.
  const geometry_msgs::msg::Pose lifecycle_manager_pose_ =
    geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(71690.0).y(73600.0).z(0.0))
      .orientation(
        geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0.0).y(0.0).z(0.0).w(1.0));
};
}  // namespace cpp_mock_scenarios

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<cpp_mock_scenarios::ExternalNpcControlCrossScenario>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
