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

#ifndef EXTERNAL_CONTROL_PLUGIN__PLUGIN_HPP_
#define EXTERNAL_CONTROL_PLUGIN__PLUGIN_HPP_

#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <traffic_simulator/behavior/behavior_plugin_base.hpp>
#include <vector>

namespace entity_behavior
{
struct ExternalVehicleState
{
  std::string name;
  double x_m{};
  double y_m{};
  double z_m{};
  double yaw_rad{};
  double speed_mps{};
};

class ExternalNpcController
{
public:
  virtual ~ExternalNpcController() = default;
  virtual void setEgoState(const std::optional<ExternalVehicleState> &) {}
  virtual void update(double current_time_s, double step_time_s) = 0;
  virtual std::vector<ExternalVehicleState> getVehicleStates() const = 0;
};

class DummyExternalNpcController : public ExternalNpcController
{
public:
  explicit DummyExternalNpcController(
    const ExternalVehicleState & initial_state,
    double start_time_s);

  void update(double current_time_s, double step_time_s) override;

  std::vector<ExternalVehicleState> getVehicleStates() const override;

private:
  ExternalVehicleState initial_state_;
  ExternalVehicleState current_state_;
  double start_time_s_;
};

class ExternalControlPlugin : public BehaviorPluginBase
{
public:
  void configure(const rclcpp::Logger & logger) override;

  void update(double current_time, double step_time) override;

  auto getCurrentAction() const -> const std::string & override;

#define DEFINE_DEFAULT_GETTER_SETTER(NAME, TYPE) \
public:                                          \
  TYPE get##NAME() override { return TYPE(); }   \
  void set##NAME(const TYPE &) override {}

  DEFINE_DEFAULT_GETTER_SETTER(
    DebugMarker,
    std::vector<visualization_msgs::msg::Marker>)
  DEFINE_DEFAULT_GETTER_SETTER(
    DefaultMatchingDistanceForLaneletPoseCalculation,
    double)
  DEFINE_DEFAULT_GETTER_SETTER(
    EuclideanDistancesMap,
    std::shared_ptr<EuclideanDistancesMap>)
  DEFINE_DEFAULT_GETTER_SETTER(GoalPoses, std::vector<geometry_msgs::msg::Pose>)
  DEFINE_DEFAULT_GETTER_SETTER(
    LaneChangeParameters,
    traffic_simulator::lane_change::Parameter)
  DEFINE_DEFAULT_GETTER_SETTER(LateralCollisionThreshold, std::optional<double>)
  DEFINE_DEFAULT_GETTER_SETTER(
    Obstacle, std::optional<traffic_simulator_msgs::msg::Obstacle>)
  DEFINE_DEFAULT_GETTER_SETTER(
    PedestrianParameters, traffic_simulator_msgs::msg::PedestrianParameters)
  DEFINE_DEFAULT_GETTER_SETTER(
    ReferenceTrajectory, std::shared_ptr<math::geometry::CatmullRomSpline>)
  DEFINE_DEFAULT_GETTER_SETTER(TargetSpeed, std::optional<double>)
  DEFINE_DEFAULT_GETTER_SETTER(
    TrafficLights, std::shared_ptr<traffic_simulator::TrafficLightsBase>)
  DEFINE_DEFAULT_GETTER_SETTER(
    Waypoints,
    traffic_simulator_msgs::msg::WaypointsArray)

#undef DEFINE_DEFAULT_GETTER_SETTER

#define DEFINE_STORED_GETTER_SETTER(NAME, TYPE, FIELD_NAME)            \
public:                                                                \
  TYPE get##NAME() override { return FIELD_NAME; }                     \
  void set##NAME(const TYPE & value) override { FIELD_NAME = value; }  \
                                                                       \
private:                                                               \
  TYPE FIELD_NAME;

  DEFINE_STORED_GETTER_SETTER(
    BehaviorParameter,
    traffic_simulator_msgs::msg::BehaviorParameter,
    behavior_parameter_)
  DEFINE_STORED_GETTER_SETTER(
    CanonicalizedEntityStatus,
    std::shared_ptr<traffic_simulator::CanonicalizedEntityStatus>,
    canonicalized_entity_status_)
  DEFINE_STORED_GETTER_SETTER(CurrentTime, double, current_time_)
  DEFINE_STORED_GETTER_SETTER(
    OtherEntityStatus, EntityStatusDict,
    other_entity_status_)
  DEFINE_STORED_GETTER_SETTER(
    PolylineTrajectory,
    std::shared_ptr<traffic_simulator_msgs::msg::PolylineTrajectory>,
    polyline_trajectory_)
  DEFINE_STORED_GETTER_SETTER(
    Request, traffic_simulator::behavior::Request,
    request_)
  DEFINE_STORED_GETTER_SETTER(RouteLanelets, lanelet::Ids, route_lanelets_)
  DEFINE_STORED_GETTER_SETTER(StepTime, double, step_time_)
  DEFINE_STORED_GETTER_SETTER(
    VehicleParameters,
    traffic_simulator_msgs::msg::VehicleParameters,
    vehicle_parameters_)

#undef DEFINE_STORED_GETTER_SETTER

  auto ensureControllerInitialized(double current_time, double step_time) -> bool;

  auto applyState(const ExternalVehicleState & state, double status_time) -> void;
  auto getEgoState() const -> std::optional<ExternalVehicleState>;

  std::unique_ptr<ExternalNpcController> controller_;
  rclcpp::Logger logger_{rclcpp::get_logger("external_control_plugin")};
  std::string behavior_{"external_control"};
  std::string controller_mode_{"dummy"};
  std::string ego_name_{"ego"};
  bool controller_initialization_failed_{false};
};
}  // namespace entity_behavior

#endif  // EXTERNAL_CONTROL_PLUGIN__PLUGIN_HPP_
