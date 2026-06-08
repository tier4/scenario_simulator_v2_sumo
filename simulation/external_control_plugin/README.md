# external_control_plugin SUMO Prototype

## Design Note

This prototype connects `external_control_plugin` to the standalone
`simulation/sumo_connection` `SumoAdapter` without changing the behavior
architecture. `DummyExternalNpcController` remains the default fallback.

Touched files:

- `simulation/external_control_plugin/include/external_control_plugin/plugin.hpp`
- `simulation/external_control_plugin/src/plugin.cpp`
- `simulation/external_control_plugin/CMakeLists.txt`
- `mock/cpp_mock_scenarios/src/external_npc_control/external_npc_control.cpp`
- `docs/developer_guide/ExternalNpcControl.md`

Update order:

1. Select controller mode from environment configuration.
2. Initialize either `DummyExternalNpcController` or `SumoExternalNpcController`.
3. In SUMO mode, read the real Ego state from `other_entity_status` when available.
4. Sync the SUMO ego shadow from the real Ego state, or from the temporary static fallback.
5. Step SUMO once.
6. Read `SumoAdapter::VehicleState` values.
7. Ignore the ego vehicle and SUMO ids with no matching scenario entity.
8. Convert matching SUMO states to `ExternalVehicleState`.
9. Apply the matching entity state through the existing `applyState()` path.

Known limitations:

- Direct id mapping only: SUMO `npc_0` maps to scenario entity `npc_0`.
- No Lanelet2-to-SUMO conversion.
- No traffic light synchronization.
- Ego synchronization uses `ExternalControlPlugin::getEgoState()`, which currently reads the
  scenario ego from `other_entity_status` by `EXTERNAL_CONTROL_SUMO_EGO_ID`.
- If the real ego is not present, SUMO mode uses a documented temporary fallback ego at
  `x=50.0`, `y=0.0`, `z=0.0`, `yaw=0.0`, `speed=0.0` so the straight-road `npc_0` can react to a
  stopped shadow vehicle ahead. Replace this fallback by ensuring the scenario ego is included in
  `OtherEntityStatus` before production use.
- SUMO support is opt-in at build time and requires C++ `libtraci` headers and library.

## Build

Dummy-only build:

```bash
colcon build --packages-up-to external_control_plugin cpp_mock_scenarios \
  --cmake-args -DBUILD_CPP_MOCK_SCENARIOS=ON
```

SUMO-enabled build:

```bash
colcon build --packages-up-to external_control_plugin cpp_mock_scenarios \
  --cmake-args \
    -DBUILD_CPP_MOCK_SCENARIOS=ON \
    -DEXTERNAL_CONTROL_PLUGIN_ENABLE_SUMO=ON \
    -DSUMO_SOURCE_DIR=/path/to/sumo/source-or-build-tree \
    -DSUMO_LIBRARY_DIR=/path/to/sumo/lib
```

`SUMO_SOURCE_DIR` must point to a tree containing `src/libsumo/libtraci.h`. `SUMO_LIBRARY_DIR` can
be omitted when `libtracicpp.so` is in a standard library path. `SUMO_HOME` can also provide search
hints when it points at a SUMO tree with the C++ headers.

## Run

Dummy mode:

```bash
EXTERNAL_CONTROL_NPC_MODE=dummy \
colcon test --packages-select cpp_mock_scenarios --ctest-args -R external_npc_control
```

SUMO mode with the straight-road scenario:

```bash
EXTERNAL_CONTROL_NPC_MODE=sumo \
EXTERNAL_CONTROL_SUMO_USE_GUI=false \
EXTERNAL_CONTROL_SUMOCFG=$PWD/src/scenario_simulator_v2/simulation/sumo_connection/scenarios/straight/straight.sumocfg \
colcon test --packages-select cpp_mock_scenarios --ctest-args -R external_npc_control
```

Use `EXTERNAL_CONTROL_SUMO_USE_GUI=true` to select `sumo-gui`, or set
`EXTERNAL_CONTROL_SUMO_BINARY` to an explicit executable path.

Runtime configuration:

- `EXTERNAL_CONTROL_NPC_MODE`: `dummy` or `sumo`, default `dummy`.
- `EXTERNAL_CONTROL_SUMOCFG`: SUMO `.sumocfg` path.
- `EXTERNAL_CONTROL_SUMO_BINARY`: explicit SUMO executable.
- `EXTERNAL_CONTROL_SUMO_USE_GUI`: `true`/`false`, default `false`.
- `EXTERNAL_CONTROL_SUMO_EGO_ID`: ego id, default `ego`.
- `EXTERNAL_CONTROL_SUMO_ROUTE_ID`: ego shadow route, default `straight_route`.
- `EXTERNAL_CONTROL_SUMO_EDGE_ID`: ego shadow edge for `moveToXY`, default `straight`.
- `EXTERNAL_CONTROL_SUMO_FALLBACK_EGO_X`: fallback ego x, default `50.0`.
- `EXTERNAL_CONTROL_SUMO_FALLBACK_EGO_Y`: fallback ego y, default `0.0`.
- `EXTERNAL_CONTROL_SUMO_FALLBACK_EGO_Z`: fallback ego z, default `0.0`.
- `EXTERNAL_CONTROL_SUMO_FALLBACK_EGO_YAW`: fallback ego yaw in radians, default `0.0`.
- `EXTERNAL_CONTROL_SUMO_FALLBACK_EGO_SPEED`: fallback ego speed, default `0.0`.
