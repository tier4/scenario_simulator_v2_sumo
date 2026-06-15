#include "sumo_adapter.hpp"

#include <libsumo/libtraci.h>

#include <algorithm>
#include <cmath>
#include <ostream>
#include <stdexcept>
#include <utility>

namespace
{
constexpr double kPi = 3.14159265358979323846;

double world_yaw_rad_to_sumo_angle_deg(double yaw_rad)
{
  return std::fmod(90.0 - yaw_rad * 180.0 / kPi + 360.0, 360.0);
}

double sumo_angle_deg_to_world_yaw_rad(double angle_deg)
{
  double yaw = std::fmod(90.0 - angle_deg, 360.0);
  if (yaw < 0.0) {
    yaw += 360.0;
  }
  if (yaw >= 180.0) {
    yaw -= 360.0;
  }
  return yaw * kPi / 180.0;
}

double normalize_sumo_z(double z)
{
  return z == libsumo::INVALID_DOUBLE_VALUE ? 0.0 : z;
}
}  // namespace

std::ostream & operator<<(std::ostream & os, const VehicleState & state)
{
  os << "VehicleState{id=" << state.id << ", type_id=" << state.type_id
     << ", x_m=" << state.x_m << ", y_m=" << state.y_m << ", z_m=" << state.z_m
     << ", yaw_rad=" << state.yaw_rad << ", speed_mps=" << state.speed_mps
     << ", acceleration_mps2=" << state.acceleration_mps2 << ", road_id=" << state.road_id
     << ", lane_id=" << state.lane_id << ", lane_position_m=" << state.lane_position_m
     << ", is_ego=" << (state.is_ego ? "true" : "false") << "}";
  return os;
}

SumoAdapter::SumoAdapter(
  std::string sumo_binary, std::filesystem::path sumocfg, double step_length, std::string ego_id,
  std::string ego_type_id, std::string route_id, std::string edge_id)
: sumo_binary_(std::move(sumo_binary)),
  sumocfg_(std::move(sumocfg)),
  step_length_(step_length),
  ego_id_(std::move(ego_id)),
  ego_type_id_(std::move(ego_type_id)),
  route_id_(std::move(route_id)),
  edge_id_(std::move(edge_id))
{
}

SumoAdapter::~SumoAdapter()
{
  close();
}

void SumoAdapter::initialize()
{
  if (initialized_) {
    return;
  }
  if (!std::filesystem::exists(sumocfg_)) {
    throw std::runtime_error("SUMO config does not exist: " + sumocfg_.string());
  }

  try {
    libtraci::Simulation::start(
      {sumo_binary_, "-c", sumocfg_.string(), "--step-length", std::to_string(step_length_),
       "--start", "--quit-on-end"});
    initialized_ = true;
  } catch (...) {
    close();
    throw;
  }
}

void SumoAdapter::close() noexcept
{
  if (!initialized_) {
    return;
  }
  try {
    libtraci::Simulation::close();
  } catch (...) {
  }
  initialized_ = false;
}

void SumoAdapter::step()
{
  require_initialized();
  libtraci::Simulation::step();
}

void SumoAdapter::syncEgo(const EgoState & ego_state)
{
  syncEgo(
    ego_state.x_m, ego_state.y_m, world_yaw_rad_to_sumo_angle_deg(ego_state.yaw_rad),
    ego_state.speed_mps);
}

void SumoAdapter::syncEgo(double x, double y, double angle_deg, double speed)
{
  require_initialized();
  add_ego_if_missing(x, speed);
  libtraci::Vehicle::moveToXY(ego_id_, edge_id_, 0, x, y, angle_deg, 2, 100.0);
  libtraci::Vehicle::setSpeed(ego_id_, speed);
}

double SumoAdapter::simulation_time() const
{
  require_initialized();
  return libtraci::Simulation::getTime();
}

std::vector<VehicleState> SumoAdapter::getVehicleStates() const
{
  require_initialized();
  auto vehicle_ids = libtraci::Vehicle::getIDList();
  std::sort(vehicle_ids.begin(), vehicle_ids.end());

  std::vector<VehicleState> states;
  states.reserve(vehicle_ids.size());
  for (const auto & vehicle_id : vehicle_ids) {
    const auto position = libtraci::Vehicle::getPosition3D(vehicle_id);
    const double angle = libtraci::Vehicle::getAngle(vehicle_id);
    VehicleState state;
    state.id = vehicle_id;
    state.type_id = libtraci::Vehicle::getTypeID(vehicle_id);
    state.x_m = position.x;
    state.y_m = position.y;
    state.z_m = normalize_sumo_z(position.z);
    state.yaw_rad = sumo_angle_deg_to_world_yaw_rad(angle);
    state.speed_mps = libtraci::Vehicle::getSpeed(vehicle_id);
    try {
      state.acceleration_mps2 = libtraci::Vehicle::getAcceleration(vehicle_id);
    } catch (...) {
      state.acceleration_mps2 = 0.0;
    }
    state.road_id = libtraci::Vehicle::getRoadID(vehicle_id);
    state.lane_id = libtraci::Vehicle::getLaneID(vehicle_id);
    state.lane_position_m = libtraci::Vehicle::getLanePosition(vehicle_id);
    state.is_ego = state.id == ego_id_;
    states.push_back(std::move(state));
  }
  return states;
}

void SumoAdapter::require_initialized() const
{
  if (!initialized_) {
    throw std::runtime_error("SumoAdapter::initialize() must be called first");
  }
}

void SumoAdapter::add_ego_if_missing(double depart_position, double depart_speed)
{
  const auto vehicle_ids = libtraci::Vehicle::getIDList();
  if (std::find(vehicle_ids.begin(), vehicle_ids.end(), ego_id_) != vehicle_ids.end()) {
    return;
  }

  libtraci::Vehicle::add(
    ego_id_, route_id_, ego_type_id_, "now", "0", std::to_string(depart_position),
    std::to_string(depart_speed), "current", "max", "current", "", "", "", 0, 0);
  libtraci::Vehicle::setSpeedMode(ego_id_, 0);
  libtraci::Vehicle::setLaneChangeMode(ego_id_, 0);
}
