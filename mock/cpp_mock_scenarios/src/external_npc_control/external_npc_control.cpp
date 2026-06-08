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
#include <cpp_mock_scenarios/catalogs.hpp>
#include <cpp_mock_scenarios/cpp_scenario_node.hpp>
#include <cstdlib>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <traffic_simulator/api/api.hpp>

namespace cpp_mock_scenarios
{
class ExternalNpcControlScenario : public cpp_mock_scenarios::CppScenarioNode
{
public:
  explicit ExternalNpcControlScenario(const rclcpp::NodeOptions & option)
  : cpp_mock_scenarios::CppScenarioNode(
      "external_npc_control",
      ament_index_cpp::get_package_share_directory("kashiwanoha_map") + "/map",
      "lanelet2_map.osm", __FILE__, false, option)
  {
    start();
  }

private:
  void onUpdate() override
  {
    const auto & npc = api_.getEntity("npc_0");
    if (api_.getCurrentTime() >= 3.0) {
      const auto mode = []() {
          const char * value = std::getenv("EXTERNAL_CONTROL_NPC_MODE");
          return value == nullptr ? std::string("dummy") : std::string(value);
        }();
      const auto speed_is_valid =
        mode == "sumo" ? npc.getCurrentTwist().linear.x > 0.0
                       : equals(npc.getCurrentTwist().linear.x, 2.0, 0.01);
      if (npc.getCurrentAction() == "external_control" &&
        npc.getMapPose().position.x > spawn_pose_.position.x + 4.0 &&
        speed_is_valid) {
        stop(cpp_mock_scenarios::Result::SUCCESS);
      } else {
        stop(cpp_mock_scenarios::Result::FAILURE);
      }
    }
  }

  void onInitialize() override
  {
    api_.spawn(
      "npc_0", spawn_pose_, getVehicleParameters(),
      traffic_simulator::entity::VehicleEntity::BuiltinBehavior::externalControl());
  }

  const geometry_msgs::msg::Pose spawn_pose_ =
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
  auto component = std::make_shared<cpp_mock_scenarios::ExternalNpcControlScenario>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
