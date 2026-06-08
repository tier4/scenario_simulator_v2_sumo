# External NPC Control Prototype

This prototype adds `external_control_plugin/ExternalControlPlugin`, a vehicle behavior plugin for
feeding externally generated NPC states into `traffic_simulator`.

The plugin replaces the default vehicle behavior tree for the entity that loads it. It supports a
dummy controller by default and an opt-in prototype SUMO controller.

## Current Prototype

The plugin defines a simulator-neutral vehicle state shape:

```cpp
struct ExternalVehicleState {
  std::string name;
  double x_m;
  double y_m;
  double z_m;
  double yaw_rad;
  double speed_mps;
};
```

It also defines an `ExternalNpcController` interface and two implementations:

- `DummyExternalNpcController` initializes from the NPC's first pose and moves that NPC forward in
  a straight line at `2.0 m/s`.
- `SumoExternalNpcController` uses the standalone `simulation/sumo_connection` `SumoAdapter` as the
  vehicle state provider. It converts `SumoAdapter` `VehicleState` values to
  `ExternalVehicleState`, filters out the SUMO ego shadow, and lets `ExternalControlPlugin` apply the
  matching NPC state through the same `EntityStatus` update path used by the dummy controller.

Direct `libtraci` calls remain isolated in `simulation/sumo_connection/src/sumo_adapter.cpp`.

## Usage

Spawn a vehicle NPC with the external-control behavior name:

```cpp
api_.spawn(
  "npc_0", spawn_pose, getVehicleParameters(),
  traffic_simulator::entity::VehicleEntity::BuiltinBehavior::externalControl());
```

The mock scenario `external_npc_control` demonstrates this setup.

Build and run the prototype scenario with:

```bash
colcon build --packages-up-to external_control_plugin cpp_mock_scenarios \
  --cmake-args -DBUILD_CPP_MOCK_SCENARIOS=ON
colcon test --packages-select cpp_mock_scenarios --ctest-args -R external_npc_control
```

This uses dummy mode because `EXTERNAL_CONTROL_NPC_MODE` defaults to `dummy`.

## SUMO Mode

Build SUMO support explicitly:

```bash
colcon build --packages-up-to external_control_plugin cpp_mock_scenarios \
  --cmake-args \
    -DBUILD_CPP_MOCK_SCENARIOS=ON \
    -DEXTERNAL_CONTROL_PLUGIN_ENABLE_SUMO=ON \
    -DSUMO_SOURCE_DIR=/path/to/sumo/source-or-build-tree \
    -DSUMO_LIBRARY_DIR=/path/to/sumo/lib
```

`SUMO_LIBRARY_DIR` can be omitted when `libtracicpp.so` is in a standard location. The straight-road
SUMO scenario is used by default:

```text
src/scenario_simulator_v2/simulation/sumo_connection/scenarios/straight/straight.sumocfg
```

Run the mock scenario in SUMO mode:

```bash
EXTERNAL_CONTROL_NPC_MODE=sumo \
EXTERNAL_CONTROL_SUMO_BINARY=sumo \
EXTERNAL_CONTROL_SUMOCFG=$PWD/src/scenario_simulator_v2/simulation/sumo_connection/scenarios/straight/straight.sumocfg \
colcon test --packages-select cpp_mock_scenarios --ctest-args -R external_npc_control
```

The SUMO route contains `npc_0` and an `ego` shadow. The plugin maps `npc_0` to the
scenario_simulator_v2 entity named `npc_0`; it does not apply SUMO's `ego` vehicle as an NPC update.
If an ego entity status is available in `other_entity_status`, the SUMO ego shadow is synchronized
before each SUMO step.

Useful environment variables:

- `EXTERNAL_CONTROL_NPC_MODE`: `dummy` or `sumo`.
- `EXTERNAL_CONTROL_SUMOCFG`: SUMO `.sumocfg` path.
- `EXTERNAL_CONTROL_SUMO_BINARY`: SUMO executable, default `sumo`.
- `EXTERNAL_CONTROL_SUMO_STEP_LENGTH`: override SUMO step length; default is the simulator step.
- `EXTERNAL_CONTROL_SUMO_EGO_ID`: SUMO/scenario ego id, default `ego`.
- `EXTERNAL_CONTROL_SUMO_ROUTE_ID`: route for the ego shadow, default `straight_route`.
- `EXTERNAL_CONTROL_SUMO_EDGE_ID`: edge for ego shadow `moveToXY`, default `straight`.

## Limitations

This is a prototype for one vehicle NPC. The plugin writes map-frame pose, yaw, and longitudinal
speed; acceleration and jerk are set to zero. Lanelet pose is marked invalid to preserve externally
supplied yaw exactly. SUMO support is intentionally opt-in at build time so the dummy controller
remains available for fallback and debugging.
