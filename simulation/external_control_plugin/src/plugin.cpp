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
#include <traffic_simulator_msgs/msg/bounding_box.hpp>

#ifdef EXTERNAL_CONTROL_PLUGIN_HAS_SUMO
#include "sumo_adapter.hpp"
#endif

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace entity_behavior {
namespace {
constexpr const char *kModeEnv = "EXTERNAL_CONTROL_NPC_MODE";
constexpr const char *kLegacyModeEnv = "EXTERNAL_CONTROL_MODE";
constexpr const char *kSumocfgEnv = "EXTERNAL_CONTROL_SUMOCFG";
constexpr const char *kSumoBinaryEnv = "EXTERNAL_CONTROL_SUMO_BINARY";
constexpr const char *kSumoUseGuiEnv = "EXTERNAL_CONTROL_SUMO_USE_GUI";
constexpr const char *kSumoStepLengthEnv = "EXTERNAL_CONTROL_SUMO_STEP_LENGTH";
constexpr const char *kSumoEgoIdEnv = "EXTERNAL_CONTROL_SUMO_EGO_ID";
constexpr const char *kSumoIdMappingEnv = "EXTERNAL_CONTROL_SUMO_ID_MAPPING";
constexpr const char *kSumoAutoSpawnEnv = "EXTERNAL_CONTROL_SUMO_AUTO_SPAWN";
constexpr const char *kSumoAutoDespawnEnv =
    "EXTERNAL_CONTROL_SUMO_AUTO_DESPAWN";
constexpr const char *kSumoEntityPrefixEnv =
    "EXTERNAL_CONTROL_SUMO_ENTITY_PREFIX";
constexpr const char *kSumoDefaultVehicleModelEnv =
    "EXTERNAL_CONTROL_SUMO_DEFAULT_VEHICLE_MODEL";
constexpr const char *kSumoTransformEnabledEnv =
    "EXTERNAL_CONTROL_SUMO_TRANSFORM_ENABLED";
constexpr const char *kSumoTransformXOffsetEnv =
    "EXTERNAL_CONTROL_SUMO_TRANSFORM_X_OFFSET";
constexpr const char *kSumoTransformYOffsetEnv =
    "EXTERNAL_CONTROL_SUMO_TRANSFORM_Y_OFFSET";
constexpr const char *kSumoTransformZOffsetEnv =
    "EXTERNAL_CONTROL_SUMO_TRANSFORM_Z_OFFSET";
constexpr const char *kSumoTransformYawOffsetEnv =
    "EXTERNAL_CONTROL_SUMO_TRANSFORM_YAW_OFFSET";
constexpr const char *kSumoReferencePointConversionEnv =
    "EXTERNAL_CONTROL_SUMO_REFERENCE_POINT_CONVERSION";
constexpr const char *kSumoScenarioReferenceToFrontEnv =
    "EXTERNAL_CONTROL_SUMO_SCENARIO_REFERENCE_TO_FRONT";
constexpr const char *kSumoRouteIdEnv = "EXTERNAL_CONTROL_SUMO_ROUTE_ID";
constexpr const char *kSumoEdgeIdEnv = "EXTERNAL_CONTROL_SUMO_EDGE_ID";
constexpr const char *kFallbackEgoXEnv = "EXTERNAL_CONTROL_SUMO_FALLBACK_EGO_X";
constexpr const char *kFallbackEgoYEnv = "EXTERNAL_CONTROL_SUMO_FALLBACK_EGO_Y";
constexpr const char *kFallbackEgoZEnv = "EXTERNAL_CONTROL_SUMO_FALLBACK_EGO_Z";
constexpr const char *kFallbackEgoYawEnv =
    "EXTERNAL_CONTROL_SUMO_FALLBACK_EGO_YAW";
constexpr const char *kFallbackEgoSpeedEnv =
    "EXTERNAL_CONTROL_SUMO_FALLBACK_EGO_SPEED";
// When false, the plugin does not inject/move an ego shadow into SUMO at all.
// Useful for pure traffic-flow demos with no Ego (e.g. the circular ring
// course), where a pinned shadow would otherwise either block the lane or, if
// placed off-network, be discarded by SUMO after one step.
constexpr const char *kSumoEgoShadowEnv = "EXTERNAL_CONTROL_SUMO_EGO_SHADOW";
constexpr const char *kDummyMode = "dummy";
constexpr const char *kSumoMode = "sumo";

#ifndef EXTERNAL_CONTROL_PLUGIN_DEFAULT_SUMOCFG
#define EXTERNAL_CONTROL_PLUGIN_DEFAULT_SUMOCFG ""
#endif

std::mutex lifecycle_bridge_mutex;
ExternalNpcLifecycleSnapshot lifecycle_bridge_snapshot;

#ifdef EXTERNAL_CONTROL_PLUGIN_HAS_SUMO
auto setLifecycleSnapshot(const ExternalNpcLifecycleSnapshot &snapshot)
    -> void {
  const std::lock_guard<std::mutex> lock(lifecycle_bridge_mutex);
  lifecycle_bridge_snapshot = snapshot;
}
#endif

auto getEnvString(const char *name, const std::string &default_value)
    -> std::string {
  const char *value = std::getenv(name);
  if (value == nullptr || std::string(value).empty()) {
    return default_value;
  }
  return value;
}

auto externalStateFromCanonicalizedStatus(
    const traffic_simulator::CanonicalizedEntityStatus &status)
    -> ExternalVehicleState {
  const auto &pose = status.getMapPose();
  const auto rpy =
      math::geometry::convertQuaternionToEulerAngle(pose.orientation);
  const auto &bounding_box = status.getBoundingBox();
  return ExternalVehicleState{status.getName(),
                              pose.position.x,
                              pose.position.y,
                              pose.position.z,
                              rpy.z,
                              status.getTwist().linear.x,
                              bounding_box.dimensions.x,
                              bounding_box.dimensions.y,
                              bounding_box.dimensions.z};
}

#ifdef EXTERNAL_CONTROL_PLUGIN_HAS_SUMO
auto getEnvBool(const char *name, const bool default_value) -> bool {
  const auto value = getEnvString(name, "");
  if (value.empty()) {
    return default_value;
  }
  return value == "1" || value == "true" || value == "TRUE" || value == "on" ||
         value == "ON";
}

auto getEnvDouble(const char *name, const double default_value) -> double {
  const char *value = std::getenv(name);
  if (value == nullptr || std::string(value).empty()) {
    return default_value;
  }
  try {
    return std::stod(value);
  } catch (const std::exception &) {
    return default_value;
  }
}

auto getEnvOptionalDouble(const char *name) -> std::optional<double> {
  const char *value = std::getenv(name);
  if (value == nullptr || std::string(value).empty()) {
    return std::nullopt;
  }
  try {
    return std::stod(value);
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

auto trim(const std::string &value) -> std::string {
  auto begin = value.begin();
  while (begin != value.end() &&
         std::isspace(static_cast<unsigned char>(*begin))) {
    ++begin;
  }
  auto end = value.end();
  while (end != begin && std::isspace(static_cast<unsigned char>(*(end - 1)))) {
    --end;
  }
  return std::string(begin, end);
}

auto parseSumoIdMapping(const std::string &value)
    -> std::unordered_map<std::string, std::string> {
  std::unordered_map<std::string, std::string> mapping;
  std::stringstream stream(value);
  std::string item;
  while (std::getline(stream, item, ',')) {
    const auto separator = item.find_first_of(":=");
    if (separator == std::string::npos) {
      continue;
    }
    auto sumo_id = trim(item.substr(0, separator));
    auto scenario_entity = trim(item.substr(separator + 1));
    if (!sumo_id.empty() && !scenario_entity.empty()) {
      mapping[std::move(sumo_id)] = std::move(scenario_entity);
    }
  }
  return mapping;
}

auto resolveScenarioEntityName(
    const std::string &sumo_vehicle_id,
    const std::unordered_map<std::string, std::string> &id_mapping,
    const bool explicit_mapping_enabled, const std::string &entity_prefix)
    -> std::optional<std::string> {
  const auto mapping_iter = id_mapping.find(sumo_vehicle_id);
  if (explicit_mapping_enabled) {
    if (mapping_iter == id_mapping.end()) {
      return std::nullopt;
    }
    return mapping_iter->second;
  }
  return entity_prefix + sumo_vehicle_id;
}

auto normalizeYaw(const double yaw_rad) -> double {
  return std::atan2(std::sin(yaw_rad), std::cos(yaw_rad));
}

auto isPositiveFinite(const double value) -> bool {
  return std::isfinite(value) && value > 0.0;
}

struct VehicleGeometry {
  double reference_to_front_m{0.0};
  double length_m{0.0};
  double width_m{0.0};
  double height_m{0.0};
};

auto vehicleGeometryFromBoundingBox(
    const traffic_simulator_msgs::msg::BoundingBox &bounding_box)
    -> VehicleGeometry {
  const auto reference_to_front_m =
      bounding_box.center.x + 0.5 * bounding_box.dimensions.x;
  return VehicleGeometry{reference_to_front_m, bounding_box.dimensions.x,
                         bounding_box.dimensions.y, bounding_box.dimensions.z};
}

struct SumoCoordinateTransform {
  bool enabled{true};
  double x_offset_m{0.0};
  double y_offset_m{0.0};
  double z_offset_m{0.0};
  double yaw_offset_rad{0.0};
  bool reference_point_conversion_enabled{true};
  double scenario_reference_to_front_m{0.0};

  auto sumoToScenario(const ExternalVehicleState &sumo_state) const
      -> ExternalVehicleState {
    auto scenario_state = sumo_state;
    if (enabled) {
      scenario_state.x_m = sumo_state.x_m + x_offset_m;
      scenario_state.y_m = sumo_state.y_m + y_offset_m;
      scenario_state.z_m = sumo_state.z_m + z_offset_m;
      scenario_state.yaw_rad =
          normalizeYaw(sumo_state.yaw_rad + yaw_offset_rad);
    }
    if (reference_point_conversion_enabled &&
        isPositiveFinite(scenario_reference_to_front_m)) {
      scenario_state.x_m -=
          scenario_reference_to_front_m * std::cos(scenario_state.yaw_rad);
      scenario_state.y_m -=
          scenario_reference_to_front_m * std::sin(scenario_state.yaw_rad);
    }
    return scenario_state;
  }

  auto scenarioToSumo(const ExternalVehicleState &scenario_state) const
      -> ExternalVehicleState {
    auto sumo_state = scenario_state;
    if (enabled) {
      sumo_state.x_m = scenario_state.x_m - x_offset_m;
      sumo_state.y_m = scenario_state.y_m - y_offset_m;
      sumo_state.z_m = scenario_state.z_m - z_offset_m;
      sumo_state.yaw_rad =
          normalizeYaw(scenario_state.yaw_rad - yaw_offset_rad);
    }
    if (reference_point_conversion_enabled &&
        isPositiveFinite(scenario_reference_to_front_m)) {
      sumo_state.x_m +=
          scenario_reference_to_front_m * std::cos(sumo_state.yaw_rad);
      sumo_state.y_m +=
          scenario_reference_to_front_m * std::sin(sumo_state.yaw_rad);
    }
    return sumo_state;
  }
};

auto getSumoCoordinateTransformFromEnv(const VehicleGeometry &geometry)
    -> SumoCoordinateTransform {
  auto scenario_reference_to_front_m = geometry.reference_to_front_m;
  const auto configured_reference_to_front =
      getEnvString(kSumoScenarioReferenceToFrontEnv, "");
  if (!configured_reference_to_front.empty()) {
    scenario_reference_to_front_m = getEnvDouble(
        kSumoScenarioReferenceToFrontEnv, geometry.reference_to_front_m);
  }
  return SumoCoordinateTransform{
      getEnvBool(kSumoTransformEnabledEnv, true),
      getEnvDouble(kSumoTransformXOffsetEnv, 0.0),
      getEnvDouble(kSumoTransformYOffsetEnv, 0.0),
      getEnvDouble(kSumoTransformZOffsetEnv, 0.0),
      getEnvDouble(kSumoTransformYawOffsetEnv, 0.0),
      getEnvBool(kSumoReferencePointConversionEnv, true),
      scenario_reference_to_front_m};
}

auto externalStateFromSumoState(const VehicleState &state,
                                const std::string &scenario_entity_name,
                                const SumoCoordinateTransform &transform,
                                const rclcpp::Logger &logger)
    -> ExternalVehicleState {
  const auto raw_sumo_state = ExternalVehicleState{
      scenario_entity_name, state.x_m,       state.y_m,      state.z_m,
      state.yaw_rad,        state.speed_mps, state.length_m, state.width_m,
      state.height_m};
  const auto external_state = transform.sumoToScenario(raw_sumo_state);
  RCLCPP_INFO(
      logger,
      "SUMO raw pose id=%s x=%.3f y=%.3f z=%.3f yaw=%.3f; "
      "scenario transformed pose entity=%s x=%.3f y=%.3f z=%.3f yaw=%.3f "
      "length=%.3f width=%.3f height=%.3f",
      state.id.c_str(), state.x_m, state.y_m, state.z_m, state.yaw_rad,
      external_state.name.c_str(), external_state.x_m, external_state.y_m,
      external_state.z_m, external_state.yaw_rad, external_state.length_m,
      external_state.width_m, external_state.height_m);
  return external_state;
}

auto egoStateFromExternalState(const ExternalVehicleState &state,
                               const SumoCoordinateTransform &transform,
                               const rclcpp::Logger &logger)
    -> SumoAdapter::EgoState {
  const auto sumo_state = transform.scenarioToSumo(state);
  RCLCPP_INFO(
      logger,
      "scenario Ego raw pose id=%s x=%.3f y=%.3f z=%.3f yaw=%.3f speed=%.3f; "
      "SUMO transformed ego x=%.3f y=%.3f z=%.3f yaw=%.3f speed=%.3f "
      "length=%.3f width=%.3f height=%.3f",
      state.name.c_str(), state.x_m, state.y_m, state.z_m, state.yaw_rad,
      state.speed_mps, sumo_state.x_m, sumo_state.y_m, sumo_state.z_m,
      sumo_state.yaw_rad, sumo_state.speed_mps, sumo_state.length_m,
      sumo_state.width_m, sumo_state.height_m);
  return SumoAdapter::EgoState{sumo_state.x_m,       sumo_state.y_m,
                               sumo_state.z_m,       sumo_state.yaw_rad,
                               sumo_state.speed_mps, sumo_state.length_m,
                               sumo_state.width_m,   sumo_state.height_m};
}

auto resolveSumoBinary() -> std::string {
  const auto configured_binary = getEnvString(kSumoBinaryEnv, "");
  if (!configured_binary.empty()) {
    return configured_binary;
  }
  return getEnvBool(kSumoUseGuiEnv, false) ? "sumo-gui" : "sumo";
}

class SumoExternalNpcController : public ExternalNpcController {
public:
  SumoExternalNpcController(
      std::string sumo_binary, std::filesystem::path sumocfg,
      const double step_length_s, std::string ego_id, std::string route_id,
      std::string edge_id,
      std::unordered_map<std::string, std::string> id_mapping,
      SumoCoordinateTransform transform, const bool auto_spawn_enabled,
      const bool auto_despawn_enabled, std::string scenario_entity_prefix,
      std::string default_vehicle_model, const rclcpp::Logger &logger)
      : fallback_ego_state_{getEnvDouble(kFallbackEgoXEnv, 50.0),
                            getEnvDouble(kFallbackEgoYEnv, 0.0),
                            getEnvDouble(kFallbackEgoZEnv, 0.0),
                            getEnvDouble(kFallbackEgoYawEnv, 0.0),
                            getEnvDouble(kFallbackEgoSpeedEnv, 0.0)},
        ego_id_(std::move(ego_id)),
        ego_shadow_enabled_(getEnvBool(kSumoEgoShadowEnv, true)),
        id_mapping_(std::move(id_mapping)),
        explicit_mapping_enabled_(!id_mapping_.empty()),
        transform_(std::move(transform)), sumo_step_length_s_(step_length_s),
        auto_spawn_enabled_(auto_spawn_enabled),
        auto_despawn_enabled_(auto_despawn_enabled),
        scenario_entity_prefix_(std::move(scenario_entity_prefix)),
        default_vehicle_model_(std::move(default_vehicle_model)),
        logger_(logger) {
    ExternalNpcLifecycleBridge::reset();
    adapter_ = std::make_unique<SumoAdapter>(
        std::move(sumo_binary), std::move(sumocfg), step_length_s, ego_id_,
        "ego_shadow", std::move(route_id), std::move(edge_id));
    adapter_->initialize();
    RCLCPP_INFO(logger_, "SumoAdapter initialized");
    RCLCPP_INFO(logger_,
                "SUMO lifecycle config auto_spawn=%s auto_despawn=%s "
                "scenario_entity_prefix=%s default_vehicle_model=%s",
                auto_spawn_enabled_ ? "true" : "false",
                auto_despawn_enabled_ ? "true" : "false",
                scenario_entity_prefix_.c_str(),
                default_vehicle_model_.c_str());
    RCLCPP_INFO(
        logger_,
        "SUMO coordinate transform loaded enabled=%s x_offset=%.3f "
        "y_offset=%.3f z_offset=%.3f yaw_offset=%.3f "
        "reference_point_conversion=%s scenario_reference_to_front=%.3f",
        transform_.enabled ? "true" : "false", transform_.x_offset_m,
        transform_.y_offset_m, transform_.z_offset_m, transform_.yaw_offset_rad,
        transform_.reference_point_conversion_enabled ? "true" : "false",
        transform_.scenario_reference_to_front_m);
    RCLCPP_INFO(logger_, "SUMO timestep locked to simulator tick step=%.6f s",
                sumo_step_length_s_);
    RCLCPP_INFO(logger_, "SUMO id mapping table loaded mode=%s entries=%zu",
                explicit_mapping_enabled_ ? "explicit" : "direct_fallback",
                id_mapping_.size());
    if (explicit_mapping_enabled_) {
      for (const auto &[sumo_id, scenario_entity] : id_mapping_) {
        RCLCPP_INFO(logger_, "SUMO id mapping entry %s -> %s", sumo_id.c_str(),
                    scenario_entity.c_str());
      }
    } else {
      RCLCPP_INFO(logger_, "SUMO id mapping table empty; using direct fallback "
                           "SUMO id == scenario entity name");
    }
  }

  void
  setEgoState(const std::optional<ExternalVehicleState> &ego_state) override {
    ego_state_ = ego_state;
  }

  void setKnownEntityNames(
      const std::unordered_set<std::string> &known_entity_names) override {
    known_entity_names_ = known_entity_names;
  }

  void update(const double current_time_s, const double step_time_s) override {
    if (std::abs(step_time_s - sumo_step_length_s_) > 1.0e-6) {
      RCLCPP_ERROR(
          logger_,
          "SUMO step length mismatch: configured=%.6f s simulator_tick=%.6f "
          "s. Keep EXTERNAL_CONTROL_SUMO_STEP_LENGTH equal to the scenario "
          "tick.",
          sumo_step_length_s_, step_time_s);
      return;
    }

    if (last_update_time_s_ && *last_update_time_s_ == current_time_s &&
        last_step_time_s_ && *last_step_time_s_ == step_time_s) {
      RCLCPP_INFO(logger_,
                  "SUMO update reused cached states time=%.3f step=%.3f",
                  current_time_s, step_time_s);
      return;
    }
    last_update_time_s_ = current_time_s;
    last_step_time_s_ = step_time_s;

    if (!ego_shadow_enabled_) {
      // Pure traffic-flow demo: no Ego, so do not inject an ego shadow at all.
      RCLCPP_INFO(logger_,
                  "ego shadow disabled (EXTERNAL_CONTROL_SUMO_EGO_SHADOW=off); "
                  "stepping SUMO without an ego mirror");
    } else {
      const auto ego_state =
          ego_state_
              ? egoStateFromExternalState(*ego_state_, transform_, logger_)
              : fallback_ego_state_;
      if (!ego_state_) {
        RCLCPP_WARN(
            logger_,
            "scenario ego state unavailable; using temporary static SUMO "
            "ego fallback "
            "x=%.3f y=%.3f z=%.3f yaw=%.3f speed=%.3f. "
            "Connect the real Ego hook through "
            "ExternalControlPlugin::getEgoState() "
            "and OtherEntityStatus.",
            fallback_ego_state_.x_m, fallback_ego_state_.y_m,
            fallback_ego_state_.z_m, fallback_ego_state_.yaw_rad,
            fallback_ego_state_.speed_mps);
      }
      if (ego_state_) {
        RCLCPP_INFO(logger_,
                    "scenario ego state selected id=%s x=%.3f y=%.3f z=%.3f "
                    "yaw=%.3f speed=%.3f",
                    ego_state_->name.c_str(), ego_state_->x_m, ego_state_->y_m,
                    ego_state_->z_m, ego_state_->yaw_rad,
                    ego_state_->speed_mps);
      }
      RCLCPP_INFO(logger_,
                  "syncing SUMO ego shadow id=%s x=%.3f y=%.3f z=%.3f yaw=%.3f "
                  "speed=%.3f",
                  ego_id_.c_str(), ego_state.x_m, ego_state.y_m, ego_state.z_m,
                  ego_state.yaw_rad, ego_state.speed_mps);
      adapter_->syncEgo(ego_state);
    }

    adapter_->step();
    RCLCPP_INFO(logger_, "SUMO step executed");

    const auto sumo_states = adapter_->getVehicleStates();
    RCLCPP_INFO(logger_, "number of SUMO vehicles observed: %zu",
                sumo_states.size());

    current_states_.clear();
    current_states_.reserve(sumo_states.size());
    ExternalNpcLifecycleSnapshot lifecycle_snapshot;
    lifecycle_snapshot.auto_spawn_enabled = auto_spawn_enabled_;
    lifecycle_snapshot.auto_despawn_enabled = auto_despawn_enabled_;
    std::unordered_set<std::string> observed_sumo_ids;
    std::size_t ignored_vehicle_count = 0;
    std::size_t missing_mapping_count = 0;
    std::size_t missing_entity_count = 0;
    for (const auto &state : sumo_states) {
      RCLCPP_INFO(logger_,
                  "raw SUMO vehicle state id=%s type_id=%s x=%.3f y=%.3f "
                  "z=%.3f yaw=%.3f speed=%.3f length=%.3f width=%.3f "
                  "height=%.3f",
                  state.id.c_str(), state.type_id.c_str(), state.x_m, state.y_m,
                  state.z_m, state.yaw_rad, state.speed_mps, state.length_m,
                  state.width_m, state.height_m);
      if (state.is_ego || state.id == ego_id_) {
        RCLCPP_INFO(logger_, "ignored SUMO ego vehicle id=%s",
                    state.id.c_str());
        ++ignored_vehicle_count;
        continue;
      }

      observed_sumo_ids.insert(state.id);
      const bool appeared = previous_sumo_vehicle_ids_.count(state.id) == 0;
      const auto scenario_entity_name = resolveScenarioEntityName(
          state.id, id_mapping_, explicit_mapping_enabled_,
          scenario_entity_prefix_);
      if (!scenario_entity_name) {
        RCLCPP_WARN(
            logger_,
            "missing mapping for SUMO vehicle id=%s; ignored speed=%.3f "
            "lane_position=%.3f lane_id=%s",
            state.id.c_str(), state.speed_mps, state.lane_position_m,
            state.lane_id.c_str());
        ++missing_mapping_count;
        continue;
      }

      if (explicit_mapping_enabled_) {
        RCLCPP_INFO(logger_, "resolved SUMO vehicle id=%s scenario_entity=%s",
                    state.id.c_str(), scenario_entity_name->c_str());
      } else {
        RCLCPP_INFO(logger_,
                    "fallback direct mapping used sumo_id=%s "
                    "scenario_entity=%s",
                    state.id.c_str(), scenario_entity_name->c_str());
      }

      if (appeared) {
        RCLCPP_INFO(logger_,
                    "appeared SUMO vehicle id=%s type_id=%s "
                    "scenario_entity=%s",
                    state.id.c_str(), state.type_id.c_str(),
                    scenario_entity_name->c_str());
      }

      const bool entity_exists =
          known_entity_names_.count(*scenario_entity_name) != 0;
      if (active_bindings_.count(state.id) == 0) {
        active_bindings_[state.id] = *scenario_entity_name;
        if (entity_exists) {
          RCLCPP_INFO(logger_,
                      "bound SUMO vehicle to existing scenario entity "
                      "sumo_id=%s scenario_entity=%s",
                      state.id.c_str(), scenario_entity_name->c_str());
        } else if (auto_spawn_enabled_) {
          const auto initial_state = externalStateFromSumoState(
              state, *scenario_entity_name, transform_, logger_);
          lifecycle_snapshot.spawn_requests.push_back(ExternalNpcSpawnRequest{
              state.id, *scenario_entity_name, state.type_id,
              default_vehicle_model_, initial_state});
          RCLCPP_INFO(logger_,
                      "requesting scenario NPC spawn sumo_id=%s type_id=%s "
                      "scenario_entity=%s model=%s",
                      state.id.c_str(), state.type_id.c_str(),
                      scenario_entity_name->c_str(),
                      default_vehicle_model_.c_str());
        } else {
          RCLCPP_WARN(logger_,
                      "scenario entity missing and auto_spawn disabled "
                      "sumo_id=%s scenario_entity=%s",
                      state.id.c_str(), scenario_entity_name->c_str());
          ++missing_entity_count;
          continue;
        }
      } else if (!entity_exists && auto_spawn_enabled_) {
        const auto initial_state = externalStateFromSumoState(
            state, *scenario_entity_name, transform_, logger_);
        lifecycle_snapshot.spawn_requests.push_back(ExternalNpcSpawnRequest{
            state.id, *scenario_entity_name, state.type_id,
            default_vehicle_model_, initial_state});
        RCLCPP_INFO(logger_,
                    "active binding has no scenario entity yet; requesting "
                    "spawn sumo_id=%s scenario_entity=%s type_id=%s",
                    state.id.c_str(), scenario_entity_name->c_str(),
                    state.type_id.c_str());
      } else if (!entity_exists) {
        RCLCPP_WARN(
            logger_,
            "mapped scenario entity not found sumo_id=%s scenario_entity=%s "
            "speed=%.3f lane_position=%.3f lane_id=%s",
            state.id.c_str(), scenario_entity_name->c_str(), state.speed_mps,
            state.lane_position_m, state.lane_id.c_str());
        ++missing_entity_count;
        continue;
      }

      current_states_.push_back(externalStateFromSumoState(
          state, *scenario_entity_name, transform_, logger_));
      lifecycle_snapshot.active_states.push_back(current_states_.back());
      RCLCPP_INFO(logger_,
                  "SUMO NPC vehicle state ready sumo_id=%s scenario_entity=%s "
                  "x=%.3f y=%.3f z=%.3f yaw=%.3f "
                  "speed=%.3f length=%.3f width=%.3f height=%.3f "
                  "lane_position=%.3f lane_id=%s road_id=%s",
                  state.id.c_str(), scenario_entity_name->c_str(), state.x_m,
                  state.y_m, state.z_m, state.yaw_rad, state.speed_mps,
                  state.length_m, state.width_m, state.height_m,
                  state.lane_position_m, state.lane_id.c_str(),
                  state.road_id.c_str());
    }

    for (const auto &previous_id : previous_sumo_vehicle_ids_) {
      if (observed_sumo_ids.count(previous_id) != 0) {
        continue;
      }
      const auto binding_iter = active_bindings_.find(previous_id);
      if (binding_iter == active_bindings_.end()) {
        continue;
      }
      RCLCPP_INFO(logger_, "disappeared SUMO vehicle id=%s scenario_entity=%s",
                  previous_id.c_str(), binding_iter->second.c_str());
      if (auto_despawn_enabled_) {
        lifecycle_snapshot.despawn_requests.push_back(
            ExternalNpcDespawnRequest{previous_id, binding_iter->second, true});
        RCLCPP_INFO(logger_,
                    "requesting scenario NPC despawn sumo_id=%s "
                    "scenario_entity=%s",
                    previous_id.c_str(), binding_iter->second.c_str());
      } else {
        lifecycle_snapshot.stale_sumo_vehicle_ids.insert(previous_id);
        stale_sumo_vehicle_ids_.insert(previous_id);
        RCLCPP_WARN(logger_,
                    "SUMO vehicle marked stale; auto_despawn disabled "
                    "sumo_id=%s scenario_entity=%s",
                    previous_id.c_str(), binding_iter->second.c_str());
      }
      active_bindings_.erase(binding_iter);
    }

    previous_sumo_vehicle_ids_ = observed_sumo_ids;
    lifecycle_snapshot.active_bindings = active_bindings_;
    lifecycle_snapshot.stale_sumo_vehicle_ids.insert(
        stale_sumo_vehicle_ids_.begin(), stale_sumo_vehicle_ids_.end());
    setLifecycleSnapshot(lifecycle_snapshot);

    RCLCPP_INFO(
        logger_,
        "SUMO NPC updates prepared for this frame: applied=%zu ignored=%zu "
        "missing_mapping=%zu missing_entity=%zu total_sumo_vehicles=%zu "
        "active_bindings=%zu spawn_requests=%zu despawn_requests=%zu",
        current_states_.size(), ignored_vehicle_count, missing_mapping_count,
        missing_entity_count, sumo_states.size(), active_bindings_.size(),
        lifecycle_snapshot.spawn_requests.size(),
        lifecycle_snapshot.despawn_requests.size());
    if (explicit_mapping_enabled_) {
      for (const auto &[sumo_id, scenario_entity] : id_mapping_) {
        if (observed_sumo_ids.count(sumo_id) == 0) {
          RCLCPP_WARN(
              logger_,
              "mapped SUMO vehicle not observed sumo_id=%s scenario_entity=%s",
              sumo_id.c_str(), scenario_entity.c_str());
        }
        if (known_entity_names_.count(scenario_entity) == 0) {
          RCLCPP_WARN(
              logger_,
              "mapped scenario entity not found sumo_id=%s scenario_entity=%s",
              sumo_id.c_str(), scenario_entity.c_str());
        }
      }
    }
  }

  std::vector<ExternalVehicleState> getVehicleStates() const override {
    return current_states_;
  }

private:
  std::unique_ptr<SumoAdapter> adapter_;
  std::optional<ExternalVehicleState> ego_state_;
  SumoAdapter::EgoState fallback_ego_state_;
  std::vector<ExternalVehicleState> current_states_;
  std::unordered_set<std::string> known_entity_names_;
  std::string ego_id_;
  bool ego_shadow_enabled_;
  std::unordered_map<std::string, std::string> id_mapping_;
  bool explicit_mapping_enabled_;
  SumoCoordinateTransform transform_;
  double sumo_step_length_s_;
  bool auto_spawn_enabled_;
  bool auto_despawn_enabled_;
  std::string scenario_entity_prefix_;
  std::string default_vehicle_model_;
  std::unordered_set<std::string> previous_sumo_vehicle_ids_;
  std::unordered_map<std::string, std::string> active_bindings_;
  std::unordered_set<std::string> stale_sumo_vehicle_ids_;
  std::optional<double> last_update_time_s_;
  std::optional<double> last_step_time_s_;
  rclcpp::Logger logger_;
};

auto getSharedSumoExternalNpcController(
    const std::string &sumo_binary, const std::filesystem::path &sumocfg,
    const double step_length_s, const std::string &ego_id,
    const std::string &route_id, const std::string &edge_id,
    std::unordered_map<std::string, std::string> id_mapping,
    SumoCoordinateTransform transform, const bool auto_spawn_enabled,
    const bool auto_despawn_enabled, std::string scenario_entity_prefix,
    std::string default_vehicle_model, const rclcpp::Logger &logger)
    -> std::shared_ptr<SumoExternalNpcController> {
  static std::weak_ptr<SumoExternalNpcController> weak_controller;
  auto controller = weak_controller.lock();
  if (controller) {
    RCLCPP_INFO(logger, "reusing shared SUMO external NPC controller");
    return controller;
  }

  controller = std::make_shared<SumoExternalNpcController>(
      sumo_binary, sumocfg, step_length_s, ego_id, route_id, edge_id,
      std::move(id_mapping), std::move(transform), auto_spawn_enabled,
      auto_despawn_enabled, std::move(scenario_entity_prefix),
      std::move(default_vehicle_model), logger);
  weak_controller = controller;
  return controller;
}
#endif
} // namespace

auto ExternalNpcLifecycleBridge::getSnapshot() -> ExternalNpcLifecycleSnapshot {
  const std::lock_guard<std::mutex> lock(lifecycle_bridge_mutex);
  return lifecycle_bridge_snapshot;
}

auto ExternalNpcLifecycleBridge::reset() -> void {
  const std::lock_guard<std::mutex> lock(lifecycle_bridge_mutex);
  lifecycle_bridge_snapshot = ExternalNpcLifecycleSnapshot{};
}

DummyExternalNpcController::DummyExternalNpcController(
    const ExternalVehicleState &initial_state, const double start_time_s)
    : initial_state_(initial_state), current_state_(initial_state),
      start_time_s_(start_time_s) {}

void DummyExternalNpcController::update(const double current_time_s,
                                        const double step_time_s) {
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
DummyExternalNpcController::getVehicleStates() const {
  return {current_state_};
}

void ExternalControlPlugin::configure(const rclcpp::Logger &logger) {
  logger_ = logger;
  controller_mode_ =
      getEnvString(kModeEnv, getEnvString(kLegacyModeEnv, kDummyMode));
  ego_name_ = getEnvString(kSumoEgoIdEnv, "ego");

  if (controller_mode_ != kDummyMode && controller_mode_ != kSumoMode) {
    RCLCPP_WARN(logger_, "unknown external control mode '%s'; using dummy",
                controller_mode_.c_str());
    controller_mode_ = kDummyMode;
  }
  RCLCPP_INFO(logger_, "external_control selected controller mode=%s",
              controller_mode_.c_str());
}

void ExternalControlPlugin::update(const double current_time,
                                   const double step_time) {
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
      RCLCPP_WARN(logger_,
                  "scenario_simulator_v2 Ego state id=%s not found in "
                  "OtherEntityStatus; "
                  "SUMO mode will use the documented temporary fallback",
                  ego_name_.c_str());
    }
  }
  controller_->setEgoState(ego_state);
  controller_->setKnownEntityNames(getKnownEntityNames());
  controller_->update(current_time, step_time);
  const auto states = controller_->getVehicleStates();
  const auto &entity_name = canonicalized_entity_status_->getName();
  const auto state_iter =
      std::find_if(states.begin(), states.end(), [&](const auto &state) {
        return state.name == entity_name;
      });

  if (state_iter == states.end()) {
    RCLCPP_WARN(logger_,
                "external_control time=%.3f entity=%s updated=false "
                "reason=missing_state",
                current_time, entity_name.c_str());
    return;
  }

  applyState(*state_iter, current_time + step_time);
  RCLCPP_INFO(
      logger_,
      "successful entity update from external controller time=%.3f entity=%s "
      "x=%.3f y=%.3f z=%.3f yaw=%.3f speed=%.3f updated=true",
      current_time, state_iter->name.c_str(), state_iter->x_m, state_iter->y_m,
      state_iter->z_m, state_iter->yaw_rad, state_iter->speed_mps);
}

auto ExternalControlPlugin::getCurrentAction() const -> const std::string & {
  return behavior_;
}

auto ExternalControlPlugin::ensureControllerInitialized(
    const double current_time, const double step_time) -> bool {
  (void)step_time;

  if (controller_) {
    return true;
  }
  if (controller_initialization_failed_ || !canonicalized_entity_status_) {
    return false;
  }

  const auto &pose = canonicalized_entity_status_->getMapPose();
  const auto rpy =
      math::geometry::convertQuaternionToEulerAngle(pose.orientation);

  if (controller_mode_ == kDummyMode) {
    controller_ = std::make_shared<DummyExternalNpcController>(
        ExternalVehicleState{canonicalized_entity_status_->getName(),
                             pose.position.x, pose.position.y, pose.position.z,
                             rpy.z, 2.0},
        current_time);
    RCLCPP_INFO(logger_,
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
      throw std::runtime_error("SUMO mode requires EXTERNAL_CONTROL_SUMOCFG or "
                               "a configured default SUMO scenario");
    }
    const auto sumo_binary = resolveSumoBinary();
    const auto id_mapping =
        parseSumoIdMapping(getEnvString(kSumoIdMappingEnv, ""));
    const auto vehicle_geometry = vehicleGeometryFromBoundingBox(
        canonicalized_entity_status_->getBoundingBox());
    const auto transform = getSumoCoordinateTransformFromEnv(vehicle_geometry);
    const auto auto_spawn_enabled = getEnvBool(kSumoAutoSpawnEnv, true);
    const auto auto_despawn_enabled = getEnvBool(kSumoAutoDespawnEnv, true);
    const auto scenario_entity_prefix = getEnvString(kSumoEntityPrefixEnv, "");
    const auto default_vehicle_model =
        getEnvString(kSumoDefaultVehicleModelEnv, "");
    const auto configured_sumo_step_length =
        getEnvOptionalDouble(kSumoStepLengthEnv);
    const auto sumo_step_length =
        configured_sumo_step_length.value_or(step_time);
    if (std::abs(sumo_step_length - step_time) > 1.0e-6) {
      throw std::runtime_error(
          "EXTERNAL_CONTROL_SUMO_STEP_LENGTH must match the simulator "
          "step_time; SUMO and scenario_simulator_v2 are stepped in lockstep");
    }
    RCLCPP_INFO(logger_,
                "external_control SUMO config binary=%s sumocfg=%s "
                "ego_id=%s route_id=%s edge_id=%s auto_spawn=%s "
                "auto_despawn=%s entity_prefix=%s default_vehicle_model=%s "
                "step_length=%.6f scenario_reference_to_front=%.3f",
                sumo_binary.c_str(), sumocfg.c_str(), ego_name_.c_str(),
                getEnvString(kSumoRouteIdEnv, "straight_route").c_str(),
                getEnvString(kSumoEdgeIdEnv, "straight").c_str(),
                auto_spawn_enabled ? "true" : "false",
                auto_despawn_enabled ? "true" : "false",
                scenario_entity_prefix.c_str(), default_vehicle_model.c_str(),
                sumo_step_length, transform.scenario_reference_to_front_m);
    controller_ = getSharedSumoExternalNpcController(
        sumo_binary, sumocfg, sumo_step_length, ego_name_,
        getEnvString(kSumoRouteIdEnv, "straight_route"),
        getEnvString(kSumoEdgeIdEnv, "straight"), id_mapping, transform,
        auto_spawn_enabled, auto_despawn_enabled, scenario_entity_prefix,
        default_vehicle_model, logger_);
    RCLCPP_INFO(logger_,
                "external_control initialized mode=sumo entity=%s binary=%s "
                "sumocfg=%s ego_id=%s",
                canonicalized_entity_status_->getName().c_str(),
                sumo_binary.c_str(), sumocfg.c_str(), ego_name_.c_str());
    return true;
  } catch (const std::exception &error) {
    controller_initialization_failed_ = true;
    RCLCPP_ERROR(logger_, "external_control SUMO initialization failed: %s",
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

auto ExternalControlPlugin::applyState(const ExternalVehicleState &state,
                                       const double status_time) -> void {
  auto entity_status = static_cast<traffic_simulator::EntityStatus>(
      *canonicalized_entity_status_);
  entity_status.time = status_time;
  entity_status.pose =
      geometry_msgs::build<geometry_msgs::msg::Pose>()
          .position(geometry_msgs::build<geometry_msgs::msg::Point>()
                        .x(state.x_m)
                        .y(state.y_m)
                        .z(state.z_m))
          .orientation(math::geometry::convertEulerAngleToQuaternion(
              geometry_msgs::build<geometry_msgs::msg::Vector3>()
                  .x(0.0)
                  .y(0.0)
                  .z(state.yaw_rad)));
  entity_status.lanelet_pose = traffic_simulator::LaneletPose();
  entity_status.lanelet_pose_valid = false;
  entity_status.action_status.current_action = behavior_;
  entity_status.action_status.twist =
      geometry_msgs::build<geometry_msgs::msg::Twist>()
          .linear(geometry_msgs::build<geometry_msgs::msg::Vector3>()
                      .x(state.speed_mps)
                      .y(0.0)
                      .z(0.0))
          .angular(geometry_msgs::build<geometry_msgs::msg::Vector3>()
                       .x(0.0)
                       .y(0.0)
                       .z(0.0));
  entity_status.action_status.accel =
      geometry_msgs::build<geometry_msgs::msg::Accel>()
          .linear(geometry_msgs::build<geometry_msgs::msg::Vector3>()
                      .x(0.0)
                      .y(0.0)
                      .z(0.0))
          .angular(geometry_msgs::build<geometry_msgs::msg::Vector3>()
                       .x(0.0)
                       .y(0.0)
                       .z(0.0));
  entity_status.action_status.linear_jerk = 0.0;
  canonicalized_entity_status_->set(
      traffic_simulator::CanonicalizedEntityStatus(entity_status,
                                                   std::nullopt));
}

auto ExternalControlPlugin::getEgoState() const
    -> std::optional<ExternalVehicleState> {
  const auto iter = other_entity_status_.find(ego_name_);
  if (iter == other_entity_status_.end()) {
    return std::nullopt;
  }
  return externalStateFromCanonicalizedStatus(iter->second);
}

auto ExternalControlPlugin::getKnownEntityNames() const
    -> std::unordered_set<std::string> {
  std::unordered_set<std::string> entity_names;
  if (canonicalized_entity_status_) {
    entity_names.insert(canonicalized_entity_status_->getName());
  }
  for (const auto &entity : other_entity_status_) {
    entity_names.insert(entity.first);
  }
  return entity_names;
}
} // namespace entity_behavior

PLUGINLIB_EXPORT_CLASS(entity_behavior::ExternalControlPlugin,
                       entity_behavior::BehaviorPluginBase)
