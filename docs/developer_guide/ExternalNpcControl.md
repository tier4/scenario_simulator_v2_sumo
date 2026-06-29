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
  `ExternalVehicleState`, filters out the SUMO ego shadow, applies a manual coordinate transform,
  tracks SUMO vehicle lifecycle, and lets `ExternalControlPlugin` apply the matching NPC state
  through the same `EntityStatus` update path used by the dummy controller.

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
EXTERNAL_CONTROL_SUMO_USE_GUI=false \
EXTERNAL_CONTROL_SUMOCFG=$PWD/src/scenario_simulator_v2/simulation/sumo_connection/scenarios/straight/straight.sumocfg \
colcon test --packages-select cpp_mock_scenarios --ctest-args -R external_npc_control
```

The SUMO route contains `npc_0`, `npc_1`, and an `ego` shadow. With direct-id fallback, the plugin
maps those SUMO ids to scenario_simulator_v2 entities with the same names; it does not apply SUMO's
`ego` vehicle as an NPC update.
If an ego entity status is available in `other_entity_status`, the SUMO ego shadow is synchronized
before each SUMO step.

If the real ego is not present in `other_entity_status`, SUMO mode uses a temporary static fallback
ego at `x=50.0`, `y=0.0`, `z=0.0`, `yaw=0.0`, `speed=0.0`. This keeps the straight-road prototype
useful because `npc_0` starts behind the stopped shadow ego and can slow or stop in response. The
real ego hook is `ExternalControlPlugin::getEgoState()`; production use should provide the ego in
`OtherEntityStatus` instead of relying on the fallback.

The SUMO update order is:

1. read scenario Ego,
2. transform Ego with `scenarioToSumo()`,
3. sync the SUMO shadow ego,
4. step SUMO,
5. read SUMO vehicle states,
6. detect appeared/continuing/disappeared SUMO vehicles,
7. filter ego,
8. resolve the SUMO-to-scenario id mapping,
9. transform each NPC with `sumoToScenario()`,
10. request scenario NPC spawn/despawn through the prototype lifecycle bridge,
11. apply the matching scenario entity update.

SUMO is the lifecycle authority for SUMO-controlled NPCs. Vehicle appearance creates a scenario
spawn request; vehicle disappearance creates a scenario despawn request when auto-despawn is
enabled. If a scenario entity with the chosen name already exists, the controller binds the SUMO id
to that existing entity instead of requesting a new spawn, which preserves the pre-spawn workflow.

Behavior plugins do not receive `traffic_simulator::API`, so `ExternalControlPlugin` cannot call
`api_.spawn()` directly. The runnable mock scenario uses
`entity_behavior::ExternalNpcLifecycleBridge` to execute the plugin-owned lifecycle requests with
the standard scenario API:

```cpp
api_.spawn(
  scenario_entity_name, initial_pose, getVehicleParameters(),
  traffic_simulator::entity::VehicleEntity::BuiltinBehavior::externalControl(),
  default_vehicle_model);
api_.despawn(scenario_entity_name);
```

The bridge is intentionally a prototype integration point, not a final production architecture.
Future work should move lifecycle execution into a simulator-level SUMO integration surface.

SUMO vehicle ids can be mapped explicitly with:

```bash
EXTERNAL_CONTROL_SUMO_ID_MAPPING=sumo_npc_001:npc_0,sumo_npc_002:npc_1
```

The value is a comma-separated list of `SUMO_ID:SCENARIO_ENTITY` pairs. `=` is also accepted as the
separator. If the mapping variable is empty, the plugin uses direct-id fallback:
SUMO `npc_0` maps to scenario entity `npc_0`. In explicit mapping mode, SUMO vehicles that are not
in the mapping table are logged and ignored. The SUMO ego vehicle id, default `ego`, is always
ignored as an NPC update.

The mapping is equivalent to this simple configuration shape:

```yaml
sumo_to_scenario_entity_map:
  sumo_npc_001: npc_0
  sumo_npc_002: npc_1
```

The coordinate transform is offset-based and bidirectional:

```text
scenario.x   = sumo.x   + x_offset
scenario.y   = sumo.y   + y_offset
scenario.z   = sumo.z   + z_offset
scenario.yaw = sumo.yaw + yaw_offset
```

After applying that frame offset, `sumoToScenario()` shifts SUMO's front-bumper pose back to the
scenario vehicle reference point by `scenario_reference_to_front = bounding_box.center.x +
length / 2`. `scenarioToSumo()` applies the inverse frame transform and shifts the scenario Ego
reference point forward to SUMO's front-bumper point before syncing the shadow. All frame offsets
default to zero and the transform is enabled by default, so the toy straight-road scenario remains
identity apart from the vehicle reference-point correction. Override the front offset with
`EXTERNAL_CONTROL_SUMO_SCENARIO_REFERENCE_TO_FRONT`, or disable this correction with
`EXTERNAL_CONTROL_SUMO_REFERENCE_POINT_CONVERSION=off` for maps/configurations that already use a
front-bumper reference.

SUMO-reported vehicle dimensions (`length`, `width`, `height`) are copied into
`ExternalVehicleState`. The mock auto-spawn bridge applies those dimensions to
`VehicleParameters::bounding_box` for newly spawned scenario NPCs, keeping the existing
reference-to-front distance fixed so the converted pose and visual footprint stay aligned.

SUMO and scenario_simulator_v2 are stepped in lockstep. By default the SUMO `--step-length` is the
scenario `step_time`. If `EXTERNAL_CONTROL_SUMO_STEP_LENGTH` is set, it must match the same
`step_time`; otherwise SUMO initialization fails fast instead of silently drifting.

The straight-road SUMO sample contains two direct-mapped vehicles, `npc_0` and `npc_1`. The
`straight_mapped.sumocfg` sample contains `sumo_npc_001` and `sumo_npc_002` for testing explicit
mapping to the same scenario entities. The second NPC departs at `t=5.0` so the auto-spawn path has
a visible late vehicle appearance:

```bash
EXTERNAL_CONTROL_NPC_MODE=sumo \
EXTERNAL_CONTROL_SUMO_USE_GUI=false \
EXTERNAL_CONTROL_SUMOCFG=$PWD/src/scenario_simulator_v2/simulation/sumo_connection/scenarios/straight/straight_mapped.sumocfg \
EXTERNAL_CONTROL_SUMO_ID_MAPPING=sumo_npc_001:npc_0,sumo_npc_002:npc_1 \
colcon test --packages-select cpp_mock_scenarios --ctest-args -R external_npc_control
```

For a visual manual run, use `sumo-gui` and RViz:

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

Useful environment variables:

- `EXTERNAL_CONTROL_NPC_MODE`: `dummy` or `sumo`.
- `EXTERNAL_CONTROL_SUMOCFG`: SUMO `.sumocfg` path.
- `EXTERNAL_CONTROL_SUMO_BINARY`: explicit SUMO executable. If unset, defaults to `sumo` or
  `sumo-gui` based on `EXTERNAL_CONTROL_SUMO_USE_GUI`.
- `EXTERNAL_CONTROL_SUMO_USE_GUI`: `true`/`false`, default `false`.
- `EXTERNAL_CONTROL_SUMO_STEP_LENGTH`: SUMO step length; default is the simulator step and any
  explicit value must match the simulator step.
- `EXTERNAL_CONTROL_SUMO_EGO_ID`: SUMO/scenario ego id, default `ego`.
- `EXTERNAL_CONTROL_SUMO_ID_MAPPING`: comma-separated `SUMO_ID:SCENARIO_ENTITY` pairs. Empty means
  direct-id fallback.
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
- `EXTERNAL_CONTROL_SUMO_REFERENCE_POINT_CONVERSION`: `on` (default) converts SUMO front-bumper
  poses to/from the scenario vehicle reference point; set `off` only if both sides already use the
  same vehicle reference point.
- `EXTERNAL_CONTROL_SUMO_SCENARIO_REFERENCE_TO_FRONT`: override the scenario reference-point to
  front-bumper distance in meters. Default is `bounding_box.center.x + bounding_box.dimensions.x / 2`
  from the scenario entity that initializes the shared SUMO controller.
- `EXTERNAL_CONTROL_SUMO_ROUTE_ID`: route for the ego shadow, default `straight_route`.
- `EXTERNAL_CONTROL_SUMO_EDGE_ID`: edge for ego shadow `moveToXY`, default `straight`.
- `EXTERNAL_CONTROL_SUMO_FALLBACK_EGO_X`: fallback ego x, default `50.0`.
- `EXTERNAL_CONTROL_SUMO_FALLBACK_EGO_Y`: fallback ego y, default `0.0`.
- `EXTERNAL_CONTROL_SUMO_FALLBACK_EGO_Z`: fallback ego z, default `0.0`.
- `EXTERNAL_CONTROL_SUMO_FALLBACK_EGO_YAW`: fallback ego yaw in radians, default `0.0`.
- `EXTERNAL_CONTROL_SUMO_FALLBACK_EGO_SPEED`: fallback ego speed, default `0.0`.

Regenerate the toy SUMO network with:

```bash
cd /home/koki/sumo_ws/src/scenario_simulator_v2/simulation/sumo_connection
netconvert \
  --node-files scenarios/straight/straight.nod.xml \
  --edge-files scenarios/straight/straight.edg.xml \
  --output-file scenarios/straight/straight.net.xml
```

## Limitations

This is a prototype for SUMO-controlled vehicle NPCs. It does not synchronize traffic lights. It
does not implement the full Lanelet2-to-SUMO workflow. The coordinate transform is manual and
offset-based, and vehicle reference point offsets may still need correction. SUMO `type_id` is
logged during spawn, but all vehicle types currently use one default scenario vehicle parameter set;
future work should map SUMO `type_id` to scenario vehicle parameters and models. Direct or
configured id mapping is required. Despawn is supported in the mock scenario through
`traffic_simulator::API::despawn()`, but production lifecycle execution still needs a simulator-level
integration point instead of the in-process prototype bridge. The plugin writes map-frame pose, yaw,
and longitudinal speed; acceleration and jerk are set to zero. Lanelet pose is marked invalid to
preserve externally supplied yaw exactly. SUMO support is intentionally opt-in at build time so the
dummy controller remains available for fallback and debugging.
