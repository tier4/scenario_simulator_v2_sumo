#include "sumo_adapter.hpp"

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
  int steps = 120;
  double step_length = 0.1;
};

[[noreturn]] void usage(const char * program, int exit_code)
{
  std::ostream & out = exit_code == 0 ? std::cout : std::cerr;
  out << "Usage: " << program << " [options]\n"
      << "\n"
      << "Options:\n"
      << "  --sumo-binary PATH    SUMO binary to run (default: sumo)\n"
      << "  --scenario-dir PATH   Directory containing straight.sumocfg (default: scenarios/straight)\n"
      << "  --steps N             Number of simulation steps to print (default: 120)\n"
      << "  --step-length SEC     SUMO step length in seconds (default: 0.1)\n"
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
              << state.id << "," << std::setprecision(3) << state.x << "," << state.y << ","
              << state.angle_deg << "," << state.speed << "," << state.road_id << ","
              << state.lane_id << "," << state.lane_position;
    if (state.id == "npc_0") {
      std::cout << "," << (state.has_leader && state.leader_id == "ego" ? "yes" : "no") << ","
                << (state.has_leader ? state.leader_id : "") << ","
                << (state.has_leader ? state.leader_gap : 0.0);
    } else {
      std::cout << ",,,";
    }
    std::cout << "\n";
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

    std::cout << "step,time,id,x,y,angle,speed,road_id,lane_id,lane_position,"
                 "npc_0_sees_ego_as_leader,leader_id,leader_gap\n";
    for (int step = 0; step < options.steps; ++step) {
      adapter.syncEgo(50.0, 0.0, 90.0, 0.0);
      adapter.step();
      print_vehicle_states(step, adapter.simulation_time(), adapter.get_vehicle_states());
    }

    return 0;
  } catch (const std::exception & error) {
    std::cerr << "probe_ego_shadow_sync: " << error.what() << "\n";
    return 1;
  }
}
