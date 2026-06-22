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
4. Convert the scenario Ego pose with `scenarioToSumo()`.
5. Sync the SUMO ego shadow from the transformed Ego state, or from the temporary static fallback.
6. Step SUMO once.
7. Read `SumoAdapter::VehicleState` values.
8. Detect SUMO vehicle lifecycle transitions: appeared, continuing, disappeared.
9. Ignore the ego vehicle and SUMO ids with no matching scenario entity.
10. Resolve each SUMO vehicle id through the mapping table, or through direct-id fallback when the
   mapping table is empty.
11. Convert matching SUMO poses with `sumoToScenario()`.
12. Request scenario NPC spawn/despawn through the prototype lifecycle bridge.
13. Convert matching SUMO states to `ExternalVehicleState`.
14. Apply the matching entity state through the existing `applyState()` path.

Known limitations:

- The mapping format is intentionally simple and environment-driven:
  `EXTERNAL_CONTROL_SUMO_ID_MAPPING=sumo_npc_001:npc_0,sumo_npc_002:npc_1`.
  `=` is also accepted as a separator.
- This corresponds to a simple mapping config shape:
  `sumo_to_scenario_entity_map: {sumo_npc_001: npc_0, sumo_npc_002: npc_1}`.
- If the mapping variable is empty, direct-id fallback is used: SUMO `npc_0` maps to scenario
  entity `npc_0`.
- Explicit mapping mode ignores SUMO vehicles that are not present in the mapping table.
- The coordinate transform is manual and offset-based. Defaults are identity:
  `x=0`, `y=0`, `z=0`, `yaw=0`, enabled.
- No Lanelet2-to-SUMO conversion.
- No traffic light synchronization.
- The prototype uses one shared SUMO controller inside the process so multiple external-control NPC
  entities observe the same SUMO step.
- SUMO is the lifecycle authority for SUMO-controlled NPCs. Vehicle appearance creates a spawn
  request; vehicle disappearance creates a despawn request when auto-despawn is enabled.
- `ExternalControlPlugin` itself cannot call `traffic_simulator::API::spawn()` because behavior
  plugins do not receive the simulator API. The runnable mock scenario uses
  `ExternalNpcLifecycleBridge` to execute plugin-owned lifecycle requests with the normal
  `api_.spawn()` / `api_.despawn()` calls.
- SUMO `type_id` is logged and carried in spawn requests, but all SUMO vehicle types currently use
  `getVehicleParameters()` and one optional `EXTERNAL_CONTROL_SUMO_DEFAULT_VEHICLE_MODEL` fallback.
- Ego synchronization uses `ExternalControlPlugin::getEgoState()`, which currently reads the
  scenario ego from `other_entity_status` by `EXTERNAL_CONTROL_SUMO_EGO_ID`.
- If the real ego is not present, SUMO mode uses a documented temporary fallback ego at
  `x=50.0`, `y=0.0`, `z=0.0`, `yaw=0.0`, `speed=0.0` so the straight-road `npc_0` can react to a
  stopped shadow vehicle ahead. The `external_npc_control_cross` demo replaces this fallback by
  spawning a genuinely moving `ego` entity (so the plugin reads a live pose from `OtherEntityStatus`
  and logs the real Ego pose, not the fallback warning). Production use should likewise ensure the
  scenario/Autoware ego is present in `OtherEntityStatus`.
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

SUMO mode with explicit id mapping:

```bash
EXTERNAL_CONTROL_NPC_MODE=sumo \
EXTERNAL_CONTROL_SUMO_USE_GUI=false \
EXTERNAL_CONTROL_SUMOCFG=$PWD/src/scenario_simulator_v2/simulation/sumo_connection/scenarios/straight/straight_mapped.sumocfg \
EXTERNAL_CONTROL_SUMO_ID_MAPPING=sumo_npc_001:npc_0,sumo_npc_002:npc_1 \
colcon test --packages-select cpp_mock_scenarios --ctest-args -R external_npc_control
```

SUMO mode with RViz and `sumo-gui`:

```bash
source /home/koki/sumo_ws/install/setup.bash
export ROS_LOG_DIR=/tmp/scenario_ros_log

EXTERNAL_CONTROL_NPC_MODE=sumo \
EXTERNAL_CONTROL_SUMO_USE_GUI=true \
EXTERNAL_CONTROL_SUMOCFG=/home/koki/sumo_ws/src/scenario_simulator_v2/simulation/sumo_connection/scenarios/straight/straight_mapped.sumocfg \
EXTERNAL_CONTROL_SUMO_ID_MAPPING=sumo_npc_001:npc_0,sumo_npc_002:npc_1 \
ros2 launch cpp_mock_scenarios mock_test.launch.py \
  package:=cpp_mock_scenarios \
  scenario:=external_npc_control \
  launch_rviz:=true \
  timeout:=60.0
```

Use `EXTERNAL_CONTROL_SUMO_USE_GUI=true` to select `sumo-gui`, or set
`EXTERNAL_CONTROL_SUMO_BINARY` to an explicit executable path.

### Cross-map intersection demo (Recommended Minimum Demo)

`external_npc_control_cross` runs the live SUMO path on the converted Lanelet2 cross map
(`simple_cross_map`). It spawns a **real, moving Ego** on lanelet 7 (mirrored into SUMO as the
shadow — the "real ego coupling", no static fallback) and lets SUMO control `npc_0` (route, major
road) and `npc_cross` (crossing, minor road). `npc_cross` yields to the route traffic at the
`j_cross` intersection and is mirrored back into traffic_simulator. This scenario needs a live SUMO,
so it is run manually and is **not** a colcon test.

```bash
source /home/koki/sumo_ws/install/setup.bash
SUMO_CROSS=/home/koki/sumo_ws/src/scenario_simulator_v2/simulation/sumo_connection/scenarios/lanelet2_simple_cross

EXTERNAL_CONTROL_NPC_MODE=sumo \
EXTERNAL_CONTROL_SUMOCFG=$SUMO_CROSS/intersection.sumocfg \
EXTERNAL_CONTROL_SUMO_USE_GUI=true \
EXTERNAL_CONTROL_SUMO_ROUTE_ID=straight_route \
EXTERNAL_CONTROL_SUMO_EDGE_ID=lanelet_7 \
EXTERNAL_CONTROL_SUMO_TRANSFORM_X_OFFSET=67944.34 \
EXTERNAL_CONTROL_SUMO_TRANSFORM_Y_OFFSET=-75.87 \
EXTERNAL_CONTROL_SUMO_TRANSFORM_Z_OFFSET=0.757 \
ros2 launch cpp_mock_scenarios mock_test.launch.py \
  scenario:=external_npc_control_cross \
  launch_autoware:=false launch_rviz:=true global_timeout:=120
```

> **Coordinate transform (important).** traffic_simulator projects `simple_cross_map` into an
> **MGRS** frame (Ego x≈71670, y≈73652), while the SUMO net was generated from the OSM
> `local_x`/`local_y` tags (x≈3700–3815, y≈73711–73782). The two frames are related by a **pure
> translation** (headings already match, so `YAW_OFFSET=0`). The offsets above place the Ego shadow
> exactly on `lanelet_7`; without them the shadow lands ~68 km outside the network and SUMO aborts
> with a `SocketException`. If the converter is later changed to emit MGRS-projected coordinates the
> offsets become `0`. To re-derive them for another map: read one logged `scenario Ego raw pose`
> (scenario frame) for an entity whose physical SUMO position is known, then
> `offset = scenario_pose - sumo_pose`.

The mock spawns the hidden `__sumo_lifecycle_manager` **on-map near the scene** (not off at
`(-1000,-1000)`), so RViz's default view — which follows the `entities` centroid frame — frames the
demo automatically; no custom RViz config is needed. (`scenarios/lanelet2_simple_cross/cross_demo.rviz`
is kept as an optional pre-centered config.)

For visual inspection, set `EXTERNAL_CONTROL_CROSS_HOLD=1` to keep the simulation (and the GUIs)
running for up to 300 s after the intersection yield is observed, instead of stopping ~20 s in:

```bash
EXTERNAL_CONTROL_CROSS_HOLD=1 \
EXTERNAL_CONTROL_NPC_MODE=sumo \
EXTERNAL_CONTROL_SUMOCFG=$SUMO_CROSS/intersection.sumocfg \
EXTERNAL_CONTROL_SUMO_USE_GUI=true \
EXTERNAL_CONTROL_SUMO_ROUTE_ID=straight_route EXTERNAL_CONTROL_SUMO_EDGE_ID=lanelet_7 \
EXTERNAL_CONTROL_SUMO_TRANSFORM_X_OFFSET=67944.34 \
EXTERNAL_CONTROL_SUMO_TRANSFORM_Y_OFFSET=-75.87 \
EXTERNAL_CONTROL_SUMO_TRANSFORM_Z_OFFSET=0.757 \
ros2 launch cpp_mock_scenarios mock_test.launch.py \
  scenario:=external_npc_control_cross launch_autoware:=false launch_rviz:=true global_timeout:=320
```

### Circular ring-road demo (endless traffic-flow course)

`external_npc_control_circular` runs the live SUMO path on the Lanelet2-derived circular test course
(`simulation/sumo_connection/scenarios/circular`). SUMO controls `npc_0`/`npc_1`/`npc_2`, which
circulate the closed ring indefinitely (the route `repeat`s); the plugin mirrors them back into
traffic_simulator so they show up in RViz while `sumo-gui` shows the same vehicles looping. The mock
loads `scenarios/circular/lanelet2_map.osm`, so RViz shows the real lanelet road via the default
lanelet map markers. There is **no Ego** here, so the ego shadow is disabled with
`EXTERNAL_CONTROL_SUMO_EGO_SHADOW=off`. The hidden bootstrap manager is **despawned** once the NPCs
are up; the NPCs keep the shared SUMO controller alive. It holds the sim/GUIs open (~300 s) for
inspection. Needs a live SUMO, so it is run manually and is not a colcon test.

```bash
source /home/koki/sumo_ws/install/setup.bash
CIRC=/home/koki/sumo_ws/src/scenario_simulator_v2/simulation/sumo_connection/scenarios/circular

EXTERNAL_CONTROL_NPC_MODE=sumo \
EXTERNAL_CONTROL_SUMOCFG=$CIRC/circular.sumocfg \
EXTERNAL_CONTROL_SUMO_USE_GUI=true \
EXTERNAL_CONTROL_SUMO_ROUTE_ID=loop \
EXTERNAL_CONTROL_SUMO_EDGE_ID=lanelet_1 \
EXTERNAL_CONTROL_SUMO_EGO_SHADOW=off \
ros2 launch cpp_mock_scenarios mock_test.launch.py \
  scenario:=external_npc_control_circular launch_autoware:=false launch_rviz:=true global_timeout:=320
```

No coordinate transform is needed: traffic_simulator loads the Lanelet2 map through its
`map_projector_info.yaml`, and SUMO uses the same `local_x` / `local_y` coordinates copied from the
map. The road centerline radius is 50 m in both RViz and SUMO.

Runtime configuration:

- `EXTERNAL_CONTROL_NPC_MODE`: `dummy` or `sumo`, default `dummy`.
- `EXTERNAL_CONTROL_SUMOCFG`: SUMO `.sumocfg` path.
- `EXTERNAL_CONTROL_SUMO_BINARY`: explicit SUMO executable.
- `EXTERNAL_CONTROL_SUMO_USE_GUI`: `true`/`false`, default `false`.
- `EXTERNAL_CONTROL_SUMO_ID_MAPPING`: comma-separated `SUMO_ID:SCENARIO_ENTITY` pairs. Empty means
  direct-id fallback.
- `EXTERNAL_CONTROL_SUMO_EGO_ID`: ego id, default `ego`.
- `EXTERNAL_CONTROL_SUMO_AUTO_SPAWN`: request scenario NPC spawn for appeared SUMO vehicles,
  default `true`.
- `EXTERNAL_CONTROL_SUMO_AUTO_DESPAWN`: request scenario NPC despawn for disappeared SUMO vehicles,
  default `true`.
- `EXTERNAL_CONTROL_SUMO_ENTITY_PREFIX`: prefix for direct fallback scenario entity names, default
  empty. Explicit mapping entries are not prefixed.
- `EXTERNAL_CONTROL_SUMO_DEFAULT_VEHICLE_MODEL`: optional model key used by the mock lifecycle
  bridge when spawning vehicles, default empty.
- `EXTERNAL_CONTROL_SUMO_TRANSFORM_ENABLED`: `true`/`false`, default `true`.
- `EXTERNAL_CONTROL_SUMO_TRANSFORM_X_OFFSET`: scenario x = SUMO x + offset, default `0.0`.
- `EXTERNAL_CONTROL_SUMO_TRANSFORM_Y_OFFSET`: scenario y = SUMO y + offset, default `0.0`.
- `EXTERNAL_CONTROL_SUMO_TRANSFORM_Z_OFFSET`: scenario z = SUMO z + offset, default `0.0`.
- `EXTERNAL_CONTROL_SUMO_TRANSFORM_YAW_OFFSET`: scenario yaw = SUMO yaw + offset, default `0.0`.
- `EXTERNAL_CONTROL_SUMO_ROUTE_ID`: ego shadow route, default `straight_route`.
- `EXTERNAL_CONTROL_SUMO_EDGE_ID`: ego shadow edge for `moveToXY`, default `straight`.
- `EXTERNAL_CONTROL_SUMO_FALLBACK_EGO_X`: fallback ego x, default `50.0`.
- `EXTERNAL_CONTROL_SUMO_FALLBACK_EGO_Y`: fallback ego y, default `0.0`.
- `EXTERNAL_CONTROL_SUMO_FALLBACK_EGO_Z`: fallback ego z, default `0.0`.
- `EXTERNAL_CONTROL_SUMO_FALLBACK_EGO_YAW`: fallback ego yaw in radians, default `0.0`.
- `EXTERNAL_CONTROL_SUMO_FALLBACK_EGO_SPEED`: fallback ego speed, default `0.0`.
- `EXTERNAL_CONTROL_SUMO_EGO_SHADOW`: `on` (default) injects/moves the ego shadow each tick; set
  `off`/`0`/`false` to skip ego-shadow injection entirely. Use `off` for a pure traffic-flow course
  with no Ego (e.g. the circular ring): an off-network shadow is discarded by SUMO after one step
  (then the next tick aborts with `tcpip::SocketException`), and an on-network shadow blocks a
  single-lane loop.

Toy network regeneration:

```bash
cd /home/koki/sumo_ws/src/scenario_simulator_v2/simulation/sumo_connection
netconvert \
  --node-files scenarios/straight/straight.nod.xml \
  --edge-files scenarios/straight/straight.edg.xml \
  --output-file scenarios/straight/straight.net.xml
```
