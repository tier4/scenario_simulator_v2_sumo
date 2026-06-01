#pragma once

#include <filesystem>
#include <string>
#include <vector>

struct VehicleState
{
  std::string id;
  std::string type_id;
  double x = 0.0;
  double y = 0.0;
  double angle_deg = 0.0;
  double yaw_deg = 0.0;
  double speed = 0.0;
  std::string road_id;
  std::string lane_id;
  double lane_position = 0.0;
  bool has_leader = false;
  std::string leader_id;
  double leader_gap = 0.0;
};

class SumoAdapter
{
public:
  SumoAdapter(
    std::string sumo_binary, std::filesystem::path sumocfg, double step_length,
    std::string ego_id = "ego", std::string ego_type_id = "ego_shadow",
    std::string route_id = "straight_route", std::string edge_id = "straight");
  ~SumoAdapter();

  SumoAdapter(const SumoAdapter &) = delete;
  SumoAdapter & operator=(const SumoAdapter &) = delete;

  void initialize();
  void close() noexcept;
  void step();
  void syncEgo(double x, double y, double angle_deg, double speed);
  void sync_ego(double x, double y, double yaw_rad, double speed);

  double simulation_time() const;
  std::vector<VehicleState> get_vehicle_states() const;

private:
  void require_initialized() const;
  void add_ego_if_missing(double depart_position, double depart_speed);

  std::string sumo_binary_;
  std::filesystem::path sumocfg_;
  double step_length_;
  std::string ego_id_;
  std::string ego_type_id_;
  std::string route_id_;
  std::string edge_id_;
  bool initialized_ = false;
};
