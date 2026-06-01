#include "sumo_adapter.hpp"

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
struct Options
{
  std::string sumo_binary = "sumo";
  std::filesystem::path scenario_dir = "scenarios/straight";
  int steps = 80;
  double step_length = 0.1;
  double ego_speed = 6.0;
  double ego_y = 0.0;
};

[[noreturn]] void usage(const char * program, int exit_code)
{
  std::ostream & out = exit_code == 0 ? std::cout : std::cerr;
  out << "Usage: " << program << " [options]\n"
      << "\n"
      << "Options:\n"
      << "  --sumo-binary PATH    SUMO binary to run (default: sumo)\n"
      << "  --scenario-dir PATH   Directory containing straight.sumocfg (default: scenarios/straight)\n"
      << "  --steps N             Number of simulation steps to print (default: 80)\n"
      << "  --step-length SEC     SUMO step length in seconds (default: 0.1)\n"
      << "  --ego-speed MPS       Shadow ego speed in m/s (default: 6.0)\n"
      << "  --ego-y Y             Shadow ego lateral world y coordinate (default: 0.0)\n"
      << "  -h, --help            Show this help text\n";
  std::exit(exit_code);
}

std::string require_value(int & index, int argc, char * argv[])
{
  if (index + 1 >= argc) {
    throw std::runtime_error(std::string("missing value for ") + argv[index]);
  }
  ++index;
  return argv[index];
}

Options parse_args(int argc, char * argv[])
{
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "-h" || arg == "--help") {
      usage(argv[0], 0);
    } else if (arg == "--sumo-binary") {
      options.sumo_binary = require_value(i, argc, argv);
    } else if (arg == "--scenario-dir") {
      options.scenario_dir = require_value(i, argc, argv);
    } else if (arg == "--steps") {
      options.steps = std::stoi(require_value(i, argc, argv));
    } else if (arg == "--step-length") {
      options.step_length = std::stod(require_value(i, argc, argv));
    } else if (arg == "--ego-speed") {
      options.ego_speed = std::stod(require_value(i, argc, argv));
    } else if (arg == "--ego-y") {
      options.ego_y = std::stod(require_value(i, argc, argv));
    } else {
      throw std::runtime_error("unknown option: " + arg);
    }
  }

  if (options.steps < 0) {
    throw std::runtime_error("--steps must be non-negative");
  }
  if (options.step_length <= 0.0) {
    throw std::runtime_error("--step-length must be positive");
  }
  return options;
}

void print_vehicle_states(int step, double sim_time, const std::vector<VehicleState> & states)
{
  for (const auto & state : states) {
    std::cout << step << "," << std::fixed << std::setprecision(2) << sim_time << ","
              << state.id << "," << state.type_id << "," << std::setprecision(3) << state.x
              << "," << state.y << "," << state.yaw_deg << "," << state.speed << ","
              << state.road_id << "\n";
  }
}

}  // namespace

int main(int argc, char * argv[])
{
  try {
    const Options options = parse_args(argc, argv);
    const auto sumocfg = options.scenario_dir / "straight.sumocfg";
    SumoAdapter adapter(options.sumo_binary, sumocfg, options.step_length);
    adapter.initialize();

    std::cout << "step,time,vehicle_id,type_id,x,y,yaw_deg,speed,road_id\n";
    for (int step = 0; step < options.steps; ++step) {
      const double sim_time = adapter.simulation_time();
      const double ego_x = std::min(490.0, options.ego_speed * sim_time);
      adapter.sync_ego(ego_x, options.ego_y, 0.0, options.ego_speed);
      adapter.step();
      print_vehicle_states(step, adapter.simulation_time(), adapter.get_vehicle_states());
    }

    return 0;
  } catch (const std::exception & error) {
    std::cerr << "probe_libtraci: " << error.what() << "\n";
    return 1;
  }
}
