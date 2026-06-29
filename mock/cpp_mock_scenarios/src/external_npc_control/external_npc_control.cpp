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

namespace cpp_mock_scenarios {
class ExternalNpcControlScenario : public cpp_mock_scenarios::CppScenarioNode {
public:
  explicit ExternalNpcControlScenario(const rclcpp::NodeOptions &option)
      : cpp_mock_scenarios::CppScenarioNode(
            "external_npc_control",
            ament_index_cpp::get_package_share_directory("kashiwanoha_map") +
                "/map",
            "lanelet2_map.osm", __FILE__, false, option) {
    start();
  }

private:
  void onUpdate() override {
    const auto mode = getExternalControlMode();
    if (mode == "sumo") {
      applySumoLifecycleRequests();
    }

    const auto check_time = mode == "sumo" ? 8.0 : 3.0;
    if (api_.getCurrentTime() >= check_time) {
      if (!api_.isEntityExist("npc_0") || !api_.isEntityExist("npc_1")) {
        stop(cpp_mock_scenarios::Result::FAILURE,
             "expected SUMO-controlled NPC entities were not spawned");
        return;
      }
      const auto &npc_0 = api_.getEntity("npc_0");
      const auto &npc_1 = api_.getEntity("npc_1");
      const auto speed_is_valid = [&](const auto &npc) {
        return mode == "sumo"
                   ? npc.getCurrentTwist().linear.x > 0.0
                   : equals(npc.getCurrentTwist().linear.x, 2.0, 0.01);
      };
      if (npc_0.getCurrentAction() == "external_control" &&
          npc_1.getCurrentAction() == "external_control" &&
          npc_0.getMapPose().position.x > spawn_pose_0_.position.x + 4.0 &&
          npc_1.getMapPose().position.x > spawn_pose_1_.position.x + 4.0 &&
          speed_is_valid(npc_0) && speed_is_valid(npc_1)) {
        stop(cpp_mock_scenarios::Result::SUCCESS);
      } else {
        stop(cpp_mock_scenarios::Result::FAILURE);
      }
    }
  }

  void onInitialize() override {
    if (getExternalControlMode() == "sumo") {
      entity_behavior::ExternalNpcLifecycleBridge::reset();
      api_.spawn(lifecycle_manager_name_, lifecycle_manager_pose_,
                 getVehicleParameters(),
                 traffic_simulator::entity::VehicleEntity::BuiltinBehavior::
                     externalControl());
      RCLCPP_INFO(get_logger(),
                  "spawned SUMO lifecycle manager entity=%s; npc_0/npc_1 "
                  "will be auto-spawned from SUMO vehicle appearances",
                  lifecycle_manager_name_.c_str());
      return;
    }

    spawnExternalControlNpc("npc_0", spawn_pose_0_, "");
    spawnExternalControlNpc("npc_1", spawn_pose_1_, "");
  }

  static auto getExternalControlMode() -> std::string {
    const char *value = std::getenv("EXTERNAL_CONTROL_NPC_MODE");
    return value == nullptr ? std::string("dummy") : std::string(value);
  }

  static auto
  poseFromExternalState(const entity_behavior::ExternalVehicleState &state)
      -> geometry_msgs::msg::Pose {
    return geometry_msgs::build<geometry_msgs::msg::Pose>()
        .position(geometry_msgs::build<geometry_msgs::msg::Point>()
                      .x(state.x_m)
                      .y(state.y_m)
                      .z(state.z_m))
        .orientation(geometry_msgs::build<geometry_msgs::msg::Quaternion>()
                         .x(0.0)
                         .y(0.0)
                         .z(std::sin(state.yaw_rad * 0.5))
                         .w(std::cos(state.yaw_rad * 0.5)));
  }

  void spawnExternalControlNpc(const std::string &name,
                               const geometry_msgs::msg::Pose &pose,
                               const std::string &model) {
    api_.spawn(name, pose, getVehicleParameters(),
               traffic_simulator::entity::VehicleEntity::BuiltinBehavior::
                   externalControl(),
               model);
  }

  static auto vehicleParametersFromExternalState(
      const entity_behavior::ExternalVehicleState &state)
      -> traffic_simulator_msgs::msg::VehicleParameters {
    auto parameters = getVehicleParameters();
    const auto reference_to_front = parameters.bounding_box.center.x +
                                    0.5 * parameters.bounding_box.dimensions.x;
    if (state.length_m > 0.0) {
      parameters.bounding_box.dimensions.x = state.length_m;
      parameters.bounding_box.center.x =
          reference_to_front - 0.5 * state.length_m;
    }
    if (state.width_m > 0.0) {
      parameters.bounding_box.dimensions.y = state.width_m;
    }
    if (state.height_m > 0.0) {
      parameters.bounding_box.dimensions.z = state.height_m;
    }
    return parameters;
  }

  void
  spawnExternalControlNpc(const std::string &name,
                          const entity_behavior::ExternalVehicleState &state,
                          const std::string &model) {
    api_.spawn(name, poseFromExternalState(state),
               vehicleParametersFromExternalState(state),
               traffic_simulator::entity::VehicleEntity::BuiltinBehavior::
                   externalControl(),
               model);
  }

  void applySumoLifecycleRequests() {
    const auto snapshot =
        entity_behavior::ExternalNpcLifecycleBridge::getSnapshot();
    for (const auto &request : snapshot.spawn_requests) {
      if (request.scenario_entity_name == lifecycle_manager_name_) {
        continue;
      }
      if (api_.isEntityExist(request.scenario_entity_name)) {
        RCLCPP_INFO(get_logger(),
                    "SUMO lifecycle binding uses existing entity sumo_id=%s "
                    "scenario_entity=%s",
                    request.sumo_vehicle_id.c_str(),
                    request.scenario_entity_name.c_str());
        continue;
      }
      spawnExternalControlNpc(request.scenario_entity_name,
                              request.initial_state,
                              request.default_vehicle_model);
      RCLCPP_INFO(get_logger(),
                  "SUMO lifecycle spawned scenario NPC sumo_id=%s type_id=%s "
                  "scenario_entity=%s model=%s",
                  request.sumo_vehicle_id.c_str(), request.sumo_type_id.c_str(),
                  request.scenario_entity_name.c_str(),
                  request.default_vehicle_model.c_str());
    }

    for (const auto &request : snapshot.despawn_requests) {
      if (request.scenario_entity_name == lifecycle_manager_name_) {
        continue;
      }
      if (!api_.isEntityExist(request.scenario_entity_name)) {
        RCLCPP_INFO(get_logger(),
                    "SUMO lifecycle despawn skipped; scenario entity already "
                    "absent sumo_id=%s scenario_entity=%s",
                    request.sumo_vehicle_id.c_str(),
                    request.scenario_entity_name.c_str());
        continue;
      }
      const auto despawned = api_.despawn(request.scenario_entity_name);
      RCLCPP_INFO(get_logger(),
                  "SUMO lifecycle despawn result sumo_id=%s "
                  "scenario_entity=%s despawned=%s",
                  request.sumo_vehicle_id.c_str(),
                  request.scenario_entity_name.c_str(),
                  despawned ? "true" : "false");
    }
  }

  const std::string lifecycle_manager_name_{"__sumo_lifecycle_manager"};

  const geometry_msgs::msg::Pose lifecycle_manager_pose_ =
      geometry_msgs::build<geometry_msgs::msg::Pose>()
          .position(geometry_msgs::build<geometry_msgs::msg::Point>()
                        .x(-1000.0)
                        .y(-1000.0)
                        .z(0.0))
          .orientation(geometry_msgs::build<geometry_msgs::msg::Quaternion>()
                           .x(0.0)
                           .y(0.0)
                           .z(0.0)
                           .w(1.0));

  const geometry_msgs::msg::Pose spawn_pose_0_ =
      geometry_msgs::build<geometry_msgs::msg::Pose>()
          .position(
              geometry_msgs::build<geometry_msgs::msg::Point>().x(0.0).y(0.0).z(
                  0.0))
          .orientation(geometry_msgs::build<geometry_msgs::msg::Quaternion>()
                           .x(0.0)
                           .y(0.0)
                           .z(0.0)
                           .w(1.0));
  const geometry_msgs::msg::Pose spawn_pose_1_ =
      geometry_msgs::build<geometry_msgs::msg::Pose>()
          .position(geometry_msgs::build<geometry_msgs::msg::Point>()
                        .x(-20.0)
                        .y(0.0)
                        .z(0.0))
          .orientation(geometry_msgs::build<geometry_msgs::msg::Quaternion>()
                           .x(0.0)
                           .y(0.0)
                           .z(0.0)
                           .w(1.0));
};
} // namespace cpp_mock_scenarios

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component =
      std::make_shared<cpp_mock_scenarios::ExternalNpcControlScenario>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
