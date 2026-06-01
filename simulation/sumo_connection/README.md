# SUMO Connection Prototype

Standalone C++ `libtraci` probe for testing SUMO-based NPC control inside
`scenario_simulator_v2`. This directory is intentionally ignored by colcon; use
plain CMake to build it.

## Files

- `scenarios/straight/`: SUMO straight-road network, route, and config files.
- `src/sumo_adapter.hpp`: minimal public adapter interface and `VehicleState` struct.
- `src/sumo_adapter.cpp`: owns direct `libtraci` calls, SUMO startup/shutdown, stepping, and shadow ego sync.
- `src/probe_libtraci.cpp`: small CLI probe using `SumoAdapter`.
- `src/probe_ego_shadow_sync.cpp`: keeps ego stopped at x=50 and checks NPC following behavior.
- `CMakeLists.txt`: standalone build file for the probe.
- `COLCON_IGNORE`: prevents this prototype from affecting `scenario_simulator_v2` colcon builds.

## Adapter Boundary

`SumoAdapter` is intentionally small. It isolates direct `libtraci` calls while
keeping the public interface independent from SUMO types:

- `initialize()` starts SUMO and creates the shadow ego vehicle.
- `close()` shuts SUMO down; the destructor also closes it.
- `step()` owns `Simulation::step()`.
- `syncEgo(x, y, angle_deg, speed)` creates the shadow ego if needed and moves it before stepping.
- `sync_ego(x, y, yaw_rad, speed)` is a compatibility helper for the original moving-ego probe.
- `get_vehicle_states()` returns `std::vector<VehicleState>`.

## Dependencies

The probe links against SUMO's C++ `libtraci` library and includes
`<libsumo/libtraci.h>`.

Best practice for C++ `libtraci` development is to build SUMO from source and
point this probe at that same source/build tree. SUMO's C++ `libtraci` API uses
headers from the SUMO source tree and links `libtracicpp`; binary packages may
install the shared library without installing the matching headers.

Recommended setup:

```bash
git clone --recursive https://github.com/eclipse-sumo/sumo.git ~/src/sumo
cd ~/src/sumo
export SUMO_HOME="$PWD"
cmake -B build .
cmake --build build -j"$(nproc)"
```

Then build this probe from `scenario_simulator_v2/simulation/sumo_connection`:

```bash
cmake -S . -B build \
  -DSUMO_SOURCE_DIR="$SUMO_HOME" \
  -DSUMO_LIBRARY_DIR="$SUMO_HOME/bin"
cmake --build build
```

If your distribution provides `libtracicpp.so` in a system library directory,
you can omit `SUMO_LIBRARY_DIR`, but still provide a matching source tree for the
headers:

```bash
cmake -S . -B build -DSUMO_SOURCE_DIR=/path/to/sumo
```

If CMake finds `libtracicpp.so` but not `<libsumo/libtraci.h>`, do not copy
random headers from another SUMO version. Use a matching SUMO source/build tree
to avoid ABI/API mismatches.

## Generate the Network

Run this from `scenario_simulator_v2/simulation/sumo_connection`:

```bash
netconvert \
  --node-files scenarios/straight/straight.nod.xml \
  --edge-files scenarios/straight/straight.edg.xml \
  --output-file scenarios/straight/straight.net.xml
```

## View the Scenario

```bash
sumo-gui -c scenarios/straight/straight.sumocfg
```

## Build the C++ Probe

From `scenario_simulator_v2/simulation/sumo_connection`:

```bash
cmake -S . -B build \
  -DSUMO_SOURCE_DIR="$SUMO_HOME" \
  -DSUMO_LIBRARY_DIR="$SUMO_HOME/bin"
cmake --build build
```

## Run the C++ Probe

```bash
./build/probe_libtraci
```

The probe prints CSV-like rows:

```text
step,time,vehicle_id,type_id,x,y,yaw_deg,speed,road_id
0,0.30,ego,ego_shadow,1.200,0.000,0.000,6.000,straight
0,0.30,npc_0,npc,6.100,-1.600,0.000,8.100,straight
```

Useful options:

```bash
./build/probe_libtraci --sumo-binary sumo-gui --steps 200 --ego-speed 10.0
```

The probe-facing yaw uses the usual world-frame convention in degrees:
`yaw_deg=0.0` points along +X. The probe handles SUMO's internal angle
convention at the boundary.

```bash
./build/probe_libtraci --help
```

## Run the Ego Shadow Sync Probe

This probe holds the externally controlled ego shadow at `x=50`, `y=0`,
`angle=90deg`, and `speed=0` before every SUMO step. `npc_0` starts behind ego on
the same route, so the CSV output can be used to confirm that the NPC slows down
and does not pass ego.

```bash
./build/probe_ego_shadow_sync --steps 120
```

The probe prints:

```text
step,time,id,x,y,angle,speed,road_id,lane_id,lane_position,npc_0_sees_ego_as_leader,leader_id,leader_gap
```
