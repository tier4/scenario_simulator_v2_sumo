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

#include <external_control_plugin/plugin.hpp>
#include <geometry/quaternion/euler_to_quaternion.hpp>
#include <geometry/quaternion/quaternion_to_euler.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <pluginlib/class_list_macros.hpp>

#ifdef EXTERNAL_CONTROL_PLUGIN_HAS_SUMO
#include "sumo_adapter.hpp"
#endif

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_set>

namespace entity_behavior
{
namespace
{
constexpr const char * kModeEnv = "EXTERNAL_CONTROL_NPC_MODE";
constexpr const char * kLegacyModeEnv = "EXTERNAL_CONTROL_MODE";
constexpr const char * kSumocfgEnv = "EXTERNAL_CONTROL_SUMOCFG";
constexpr const char * kSumoBinaryEnv = "EXTERNAL_CONTROL_SUMO_BINARY";
constexpr const char * kSumoUseGuiEnv = "EXTERNAL_CONTROL_SUMO_USE_GUI";
constexpr const char * kSumoStepLengthEnv = "EXTERNAL_CONTROL_SUMO_STEP_LENGTH";
constexpr const char * kSumoEgoIdEnv = "EXTERNAL_CONTROL_SUMO_EGO_ID";
constexpr const char * kSumoRouteIdEnv = "EXTERNAL_CONTROL_SUMO_ROUTE_ID";
constexpr const char * kSumoEdgeIdEnv = "EXTERNAL_CONTROL_SUMO_EDGE_ID";
constexpr const char * kFallbackEgoXEnv = "EXTERNAL_CONTROL_SUMO_FALLBACK_EGO_X";
constexpr const char * kFallbackEgoYEnv = "EXTERNAL_CONTROL_SUMO_FALLBACK_EGO_Y";
constexpr const char * kFallbackEgoZEnv = "EXTERNAL_CONTROL_SUMO_FALLBACK_EGO_Z";
constexpr const char * kFallbackEgoYawEnv = "EXTERNAL_CONTROL_SUMO_FALLBACK_EGO_YAW";
constexpr const char * kFallbackEgoSpeedEnv = "EXTERNAL_CONTROL_SUMO_FALLBACK_EGO_SPEED";
constexpr const char * kDummyMode = "dummy";
constexpr const char * kSumoMode = "sumo";

#ifndef EXTERNAL_CONTROL_PLUGIN_DEFAULT_SUMOCFG
#define EXTERNAL_CONTROL_PLUGIN_DEFAULT_SUMOCFG ""
#endif

auto getEnvString(const char * name, const std::string & default_value) -> std::string
{
  const char * value = std::getenv(name);
  if (value == nullptr || std::string(value).empty()) {
    return default_value;
  }
  return value;
}

auto externalStateFromCanonicalizedStatus(
  const traffic_simulator::CanonicalizedEntityStatus & status) -> ExternalVehicleState
{
  const auto & pose = status.getMapPose();
  const auto rpy =
    math::geometry::convertQuaternionToEulerAngle(pose.orientation);
  return ExternalVehicleState{
    status.getName(), pose.position.x, pose.position.y,
    pose.position.z, rpy.z, status.getTwist().linear.x};
}

#ifdef EXTERNAL_CONTROL_PLUGIN_HAS_SUMO
auto getEnvBool(const char * name, const bool default_value) -> bool
{
  const auto value = getEnvString(name, "");
  if (value.empty()) {
    return default_value;
  }
  return value == "1" || value == "true" || value == "TRUE" || value == "on" || value == "ON";
}

auto getEnvDouble(const char * name, const double default_value) -> double
{
  const char * value = std::getenv(name);
  if (value == nullptr || std::string(value).empty()) {
    return default_value;
  }
  try {
    return std::stod(value);
  } catch (const std::exception &) {
    return default_value;
  }
}

auto externalStateFromSumoState(const VehicleState & state) -> ExternalVehicleState
{
  return ExternalVehicleState{state.id, state.x_m, state.y_m,
    state.z_m, state.yaw_rad, state.speed_mps};
}

auto egoStateFromExternalState(const ExternalVehicleState & state) -> SumoAdapter::EgoState
{
  return SumoAdapter::EgoState{
    state.x_m, state.y_m, state.z_m, state.yaw_rad, state.speed_mps};
}

auto resolveSumoBinary() -> std::string
{
  const auto configured_binary = getEnvString(kSumoBinaryEnv, "");
  if (!configured_binary.empty()) {
    return configured_binary;
  }
  return getEnvBool(kSumoUseGuiEnv, false) ? "sumo-gui" : "sumo";
}

class SumoExternalNpcController : public ExternalNpcController
{
public:
  SumoExternalNpcController(
    std::string sumo_binary,
    std::filesystem::path sumocfg,
    const double step_length_s, std::string ego_id,
    std::string route_id, std::string edge_id,
    const rclcpp::Logger & logger)
  : fallback_ego_state_{
      getEnvDouble(kFallbackEgoXEnv, 50.0),
      getEnvDouble(kFallbackEgoYEnv, 0.0),
      getEnvDouble(kFallbackEgoZEnv, 0.0),
      getEnvDouble(kFallbackEgoYawEnv, 0.0),
      getEnvDouble(kFallbackEgoSpeedEnv, 0.0)},
    ego_id_(std::move(ego_id)),
    logger_(logger)
  {
    adapter_ = std::make_unique<SumoAdapter>(
      std::move(sumo_binary), std::move(sumocfg), step_length_s, ego_id_,
      "ego_shadow", std::move(route_id), std::move(edge_id));
    adapter_->initialize();
    RCLCPP_INFO(logger_, "SumoAdapter initialized");
  }

  void setEgoState(const std::optional<ExternalVehicleState> & ego_state) override
  {
    ego_state_ = ego_state;
  }

  void setKnownEntityNames(const std::unordered_set<std::string> & known_entity_names) override
  {
    known_entity_names_ = known_entity_names;
  }

  void update(const double, const double) override
  {
    const auto ego_state = ego_state_
      ? egoStateFromExternalState(*ego_state_)
      : fallback_ego_state_;
    if (!ego_state_) {
      RCLCPP_WARN(
        logger_,
        "scenario ego state unavailable; using temporary static SUMO ego fallback "
        "x=%.3f y=%.3f z=%.3f yaw=%.3f speed=%.3f. "
        "Connect the real Ego hook through ExternalControlPlugin::getEgoState() "
        "and OtherEntityStatus.",
        fallback_ego_state_.x_m, fallback_ego_state_.y_m,
        fallback_ego_state_.z_m, fallback_ego_state_.yaw_rad,
        fallback_ego_state_.speed_mps);
    }
    if (ego_state_) {
      RCLCPP_INFO(
        logger_,
        "scenario ego state selected id=%s x=%.3f y=%.3f z=%.3f "
        "yaw=%.3f speed=%.3f",
        ego_state_->name.c_str(), ego_state_->x_m, ego_state_->y_m,
        ego_state_->z_m, ego_state_->yaw_rad, ego_state_->speed_mps);
    }
    RCLCPP_INFO(
      logger_,
      "syncing SUMO ego shadow id=%s x=%.3f y=%.3f z=%.3f yaw=%.3f speed=%.3f",
      ego_id_.c_str(), ego_state.x_m, ego_state.y_m, ego_state.z_m,
      ego_state.yaw_rad, ego_state.speed_mps);
    adapter_->syncEgo(ego_state);

    adapter_->step();
    RCLCPP_INFO(logger_, "SUMO step executed");

    const auto sumo_states = adapter_->getVehicleStates();
    RCLCPP_INFO(logger_, "number of SUMO vehicles: %zu", sumo_states.size());

    current_states_.clear();
    current_states_.reserve(sumo_states.size());
    for (const auto & state : sumo_states) {
      if (state.is_ego || state.id == ego_id_) {
        RCLCPP_INFO(logger_, "ignored SUMO ego vehicle id=%s", state.id.c_str());
        continue;
      }
      if (known_entity_names_.count(state.id) == 0) {
        RCLCPP_WARN(
          logger_,
          "missing entity mapping for SUMO vehicle id=%s speed=%.3f lane_position=%.3f "
          "lane_id=%s",
          state.id.c_str(), state.speed_mps, state.lane_position_m,
          state.lane_id.c_str());
        continue;
      }
      current_states_.push_back(externalStateFromSumoState(state));
      RCLCPP_INFO(
        logger_,
        "SUMO NPC vehicle state ready id=%s x=%.3f y=%.3f z=%.3f yaw=%.3f "
        "speed=%.3f lane_position=%.3f lane_id=%s road_id=%s",
        state.id.c_str(), state.x_m, state.y_m, state.z_m,
        state.yaw_rad, state.speed_mps, state.lane_position_m,
        state.lane_id.c_str(), state.road_id.c_str());
    }
  }

  std::vector<ExternalVehicleState> getVehicleStates() const override
  {
    return current_states_;
  }

private:
  std::unique_ptr<SumoAdapter> adapter_;
  std::optional<ExternalVehicleState> ego_state_;
  SumoAdapter::EgoState fallback_ego_state_;
  std::vector<ExternalVehicleState> current_states_;
  std::unordered_set<std::string> known_entity_names_;
  std::string ego_id_;
  rclcpp::Logger logger_;
};
#endif
}  // namespace

DummyExternalNpcController::DummyExternalNpcController(
  const ExternalVehicleState & initial_state, const double start_time_s)
: initial_state_(initial_state), current_state_(initial_state),
  start_time_s_(start_time_s) {}

void DummyExternalNpcController::update(
  const double current_time_s,
  const double step_time_s)
{
  const double elapsed_time_s =
    std::max(0.0, current_time_s + step_time_s - start_time_s_);
  current_state_ = initial_state_;
  current_state_.x_m = initial_state_.x_m + std::cos(initial_state_.yaw_rad) *
    initial_state_.speed_mps *
    elapsed_time_s;
  current_state_.y_m = initial_state_.y_m + std::sin(initial_state_.yaw_rad) *
    initial_state_.speed_mps *
    elapsed_time_s;
}

std::vector<ExternalVehicleState>
DummyExternalNpcController::getVehicleStates() const
{
  return {current_state_};
}

void ExternalControlPlugin::configure(const rclcpp::Logger & logger)
{
  logger_ = logger;
  controller_mode_ =
    getEnvString(kModeEnv, getEnvString(kLegacyModeEnv, kDummyMode));
  ego_name_ = getEnvString(kSumoEgoIdEnv, "ego");

  if (controller_mode_ != kDummyMode && controller_mode_ != kSumoMode) {
    RCLCPP_WARN(
      logger_, "unknown external control mode '%s'; using dummy",
      controller_mode_.c_str());
    controller_mode_ = kDummyMode;
  }
  RCLCPP_INFO(logger_, "external_control selected controller mode=%s", controller_mode_.c_str());
}

void ExternalControlPlugin::update(
  const double current_time,
  const double step_time)
{
  setCurrentTime(current_time);
  setStepTime(step_time);
  behavior_ = "external_control";

  if (!ensureControllerInitialized(current_time, step_time)) {
    RCLCPP_WARN(
      logger_,
      "external_control update skipped: controller is not initialized");
    return;
  }

  const auto ego_state = getEgoState();
  if (controller_mode_ == kSumoMode) {
    if (ego_state) {
      RCLCPP_INFO(
        logger_,
        "read scenario_simulator_v2 Ego state id=%s x=%.3f y=%.3f z=%.3f "
        "yaw=%.3f speed=%.3f",
        ego_state->name.c_str(), ego_state->x_m, ego_state->y_m,
        ego_state->z_m, ego_state->yaw_rad, ego_state->speed_mps);
    } else {
      RCLCPP_WARN(
        logger_,
        "scenario_simulator_v2 Ego state id=%s not found in OtherEntityStatus; "
        "SUMO mode will use the documented temporary fallback",
        ego_name_.c_str());
    }
  }
  controller_->setEgoState(ego_state);
  controller_->setKnownEntityNames(getKnownEntityNames());
  controller_->update(current_time, step_time);
  const auto states = controller_->getVehicleStates();
  const auto & entity_name = canonicalized_entity_status_->getName();
  const auto state_iter =
    std::find_if(
    states.begin(), states.end(), [&](const auto & state) {
      return state.name == entity_name;
    });

  if (state_iter == states.end()) {
    RCLCPP_WARN(
      logger_,
      "external_control time=%.3f entity=%s updated=false "
      "reason=missing_state",
      current_time, entity_name.c_str());
    return;
  }

  applyState(*state_iter, current_time + step_time);
  RCLCPP_INFO(
    logger_,
    "applied NPC vehicle state from external controller time=%.3f entity=%s "
    "x=%.3f y=%.3f z=%.3f yaw=%.3f speed=%.3f updated=true",
    current_time, state_iter->name.c_str(), state_iter->x_m,
    state_iter->y_m, state_iter->z_m, state_iter->yaw_rad,
    state_iter->speed_mps);
}

auto ExternalControlPlugin::getCurrentAction() const -> const std::string &
{
  return behavior_;
}

auto ExternalControlPlugin::ensureControllerInitialized(
  const double current_time, const double step_time) -> bool
{
  (void)step_time;

  if (controller_) {
    return true;
  }
  if (controller_initialization_failed_ || !canonicalized_entity_status_) {
    return false;
  }

  const auto & pose = canonicalized_entity_status_->getMapPose();
  const auto rpy =
    math::geometry::convertQuaternionToEulerAngle(pose.orientation);

  if (controller_mode_ == kDummyMode) {
    controller_ = std::make_unique<DummyExternalNpcController>(
      ExternalVehicleState{canonicalized_entity_status_->getName(),
        pose.position.x, pose.position.y, pose.position.z,
        rpy.z, 2.0},
      current_time);
    RCLCPP_INFO(
      logger_,
      "external_control initialized mode=dummy entity=%s x=%.3f "
      "y=%.3f yaw=%.3f speed=%.3f",
      canonicalized_entity_status_->getName().c_str(),
      pose.position.x, pose.position.y, rpy.z, 2.0);
    return true;
  }

#ifdef EXTERNAL_CONTROL_PLUGIN_HAS_SUMO
  try {
    const auto sumocfg = getEnvString(
      kSumocfgEnv, std::string(EXTERNAL_CONTROL_PLUGIN_DEFAULT_SUMOCFG));
    if (sumocfg.empty()) {
      throw std::runtime_error(
              "SUMO mode requires EXTERNAL_CONTROL_SUMOCFG or "
              "a configured default SUMO scenario");
    }
    const auto sumo_binary = resolveSumoBinary();
    controller_ = std::make_unique<SumoExternalNpcController>(
      sumo_binary, sumocfg,
      getEnvDouble(kSumoStepLengthEnv, step_time), ego_name_,
      getEnvString(kSumoRouteIdEnv, "straight_route"),
      getEnvString(kSumoEdgeIdEnv, "straight"), logger_);
    RCLCPP_INFO(
      logger_, "external_control initialized mode=sumo entity=%s binary=%s sumocfg=%s ego_id=%s",
      canonicalized_entity_status_->getName().c_str(), sumo_binary.c_str(), sumocfg.c_str(),
      ego_name_.c_str());
    return true;
  } catch (const std::exception & error) {
    controller_initialization_failed_ = true;
    RCLCPP_ERROR(
      logger_, "external_control SUMO initialization failed: %s",
      error.what());
    return false;
  }
#else
  controller_initialization_failed_ = true;
  RCLCPP_ERROR(
    logger_,
    "external_control SUMO mode requested but SUMO support was not built");
  return false;
#endif

  return true;
}

auto ExternalControlPlugin::applyState(
  const ExternalVehicleState & state,
  const double status_time) -> void
{
  auto entity_status = static_cast<traffic_simulator::EntityStatus>(
    *canonicalized_entity_status_);
  entity_status.time = status_time;
  entity_status.pose =
    geometry_msgs::build<geometry_msgs::msg::Pose>()
    .position(
    geometry_msgs::build<geometry_msgs::msg::Point>()
    .x(state.x_m)
    .y(state.y_m)
    .z(state.z_m))
    .orientation(
    math::geometry::convertEulerAngleToQuaternion(
      geometry_msgs::build<geometry_msgs::msg::Vector3>()
      .x(0.0)
      .y(0.0)
      .z(state.yaw_rad)));
  entity_status.lanelet_pose = traffic_simulator::LaneletPose();
  entity_status.lanelet_pose_valid = false;
  entity_status.action_status.current_action = behavior_;
  entity_status.action_status.twist =
    geometry_msgs::build<geometry_msgs::msg::Twist>()
    .linear(
    geometry_msgs::build<geometry_msgs::msg::Vector3>()
    .x(state.speed_mps)
    .y(0.0)
    .z(0.0))
    .angular(
    geometry_msgs::build<geometry_msgs::msg::Vector3>()
    .x(0.0)
    .y(0.0)
    .z(0.0));
  entity_status.action_status.accel =
    geometry_msgs::build<geometry_msgs::msg::Accel>()
    .linear(
    geometry_msgs::build<geometry_msgs::msg::Vector3>()
    .x(0.0)
    .y(0.0)
    .z(0.0))
    .angular(
    geometry_msgs::build<geometry_msgs::msg::Vector3>()
    .x(0.0)
    .y(0.0)
    .z(0.0));
  entity_status.action_status.linear_jerk = 0.0;
  canonicalized_entity_status_->set(
    traffic_simulator::CanonicalizedEntityStatus(
      entity_status,
      std::nullopt));
}

auto ExternalControlPlugin::getEgoState() const -> std::optional<ExternalVehicleState>
{
  const auto iter = other_entity_status_.find(ego_name_);
  if (iter == other_entity_status_.end()) {
    return std::nullopt;
  }
  return externalStateFromCanonicalizedStatus(iter->second);
}

auto ExternalControlPlugin::getKnownEntityNames() const -> std::unordered_set<std::string>
{
  std::unordered_set<std::string> entity_names;
  if (canonicalized_entity_status_) {
    entity_names.insert(canonicalized_entity_status_->getName());
  }
  for (const auto & entity : other_entity_status_) {
    entity_names.insert(entity.first);
  }
  return entity_names;
}
}  // namespace entity_behavior

PLUGINLIB_EXPORT_CLASS(
  entity_behavior::ExternalControlPlugin,
  entity_behavior::BehaviorPluginBase)
