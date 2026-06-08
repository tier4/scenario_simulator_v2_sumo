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
  const VehicleState * ego = nullptr;
  const VehicleState * npc_0 = nullptr;
  for (const auto & state : states) {
    if (state.is_ego) {
      ego = &state;
    }
    if (state.id == "npc_0") {
      npc_0 = &state;
    }
  }

  for (const auto & state : states) {
    std::cout << step << "," << std::fixed << std::setprecision(2) << sim_time << ","
              << std::setprecision(3) << state;
    if (state.id == "npc_0" && ego != nullptr) {
      std::cout << ",npc_0_gap_to_ego_m=" << (ego->lane_position_m - state.lane_position_m)
                << ",npc_0_speed_mps=" << state.speed_mps;
    }
    std::cout << "\n";
  }

  if (ego == nullptr || npc_0 == nullptr) {
    std::cerr << "missing expected vehicles: ego=" << (ego == nullptr ? "no" : "yes")
              << ", npc_0=" << (npc_0 == nullptr ? "no" : "yes") << "\n";
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

    std::cout << "step,time,vehicle_state,npc_0_gap_to_ego_m,npc_0_speed_mps\n";
    for (int step = 0; step < options.steps; ++step) {
      adapter.syncEgo(50.0, 0.0, 90.0, 0.0);
      adapter.step();
      print_vehicle_states(step, adapter.simulation_time(), adapter.getVehicleStates());
    }

    return 0;
  } catch (const std::exception & error) {
    std::cerr << "probe_ego_shadow_sync: " << error.what() << "\n";
    return 1;
  }
}
