#pragma once

#include <filesystem>
#include <ostream>
#include <string>
#include <vector>

struct VehicleState {
  // Simulator-neutral SUMO vehicle snapshot. This will later be converted to
  // OSI TrafficUpdate or traffic_simulator EntityStatus at the integration
  // boundary.
  std::string id;
  std::string type_id;
  double x_m = 0.0;
  double y_m = 0.0;
  double z_m = 0.0;
  double yaw_rad = 0.0;
  double speed_mps = 0.0;
  double acceleration_mps2 = 0.0;
  double length_m = 0.0;
  double width_m = 0.0;
  double height_m = 0.0;
  std::string road_id;
  std::string lane_id;
  double lane_position_m = 0.0;
  bool is_ego = false;
};

std::ostream &operator<<(std::ostream &os, const VehicleState &state);

class SumoAdapter {
public:
  struct EgoState {
    double x_m = 0.0;
    double y_m = 0.0;
    double z_m = 0.0;
    double yaw_rad = 0.0;
    double speed_mps = 0.0;
    double length_m = 0.0;
    double width_m = 0.0;
    double height_m = 0.0;
  };

  SumoAdapter(std::string sumo_binary, std::filesystem::path sumocfg,
              double step_length, std::string ego_id = "ego",
              std::string ego_type_id = "ego_shadow",
              std::string route_id = "straight_route",
              std::string edge_id = "straight");
  ~SumoAdapter();

  SumoAdapter(const SumoAdapter &) = delete;
  SumoAdapter &operator=(const SumoAdapter &) = delete;

  void initialize();
  void close() noexcept;
  void step();
  void syncEgo(const EgoState &ego_state);
  void syncEgo(double x, double y, double angle_deg, double speed);

  double simulation_time() const;
  std::vector<VehicleState> getVehicleStates() const;

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
