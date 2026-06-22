# Lanelet2-to-SUMO Conversion

This phase establishes a reproducible Lanelet2-derived SUMO network workflow
without traffic light synchronization and without changes to
`external_control_plugin`.

## Tools Found

Local workspace search results:

- No local `lanelet2_converter`, `awsim2sumo`, `lanelet2_map.commonroad.xml`, or
  `lanelet2_map.net.xml` conversion artifacts were found.
- No AWSIM/SUMO conversion scripts were found under this workspace.
- `commonroad`, `commonroad-scenario-designer`, and `crdesigner` Python modules
  are not installed on this host.
- SUMO tools are installed: `netconvert`, `sumo`, `sumo-gui`, and `netedit`.
  `netconvert --help` for SUMO `1.27.0` does not list a Lanelet2 importer, so
  this workflow generates SUMO plain XML and then runs `netconvert`.
- Python Lanelet2 bindings are installed, but this workflow currently uses a
  plain XML parser because the maps in this workspace already contain
  Autoware `local_x`/`local_y` node tags.

Related upstream tools:

- CommonRoad Scenario Designer supports Lanelet2/CommonRoad/SUMO map conversion,
  but it is not available locally in this workspace:
  <https://commonroad.in.tum.de/tools/scenario-designer>
- SUMO documentation recommends generating plain XML nodes/edges/connections for
  custom network inputs and converting them with `netconvert`:
  <https://sumo.dlr.de/docs/Networks/SUMO_Road_Networks.html>

## Inputs

Reproduced input map (paths relative to the `scenario_simulator_v2` repository
root):

```text
map/simple_cross_map/map/lanelet2_map.osm
```

Related map metadata:

```text
map/simple_cross_map/map/global_map_center.pcd.yaml
map/simple_cross_map/map/lanelet2_map_provider.osm.yaml
map/simple_cross_map/map/map.map_publisher.yaml
```

The OSM nodes include `local_x`, `local_y`, `ele`, and `mgrs_code` tags. The
generated SUMO network uses `local_x`/`local_y` directly as SUMO Cartesian
coordinates.

Traffic light mapping files were not used in this phase.

## Reproduction

Run from `scenario_simulator_v2/simulation/sumo_connection`:

```bash
python3 -B scripts/convert_lanelet2_to_sumo.py
```

Equivalent explicit command:

```bash
python3 -B scripts/convert_lanelet2_to_sumo.py \
  --input-map ../../map/simple_cross_map/map/lanelet2_map.osm \
  --output-dir scenarios/lanelet2_simple_cross
```

The script writes SUMO plain XML, then runs:

```bash
netconvert \
  --node-files scenarios/lanelet2_simple_cross/converted_map.nod.xml \
  --edge-files scenarios/lanelet2_simple_cross/converted_map.edg.xml \
  --connection-files scenarios/lanelet2_simple_cross/converted_map.con.xml \
  --output-file scenarios/lanelet2_simple_cross/converted_map.net.xml \
  --offset.disable-normalization \
  --plain.extend-edge-shape \
  --log scenarios/lanelet2_simple_cross/converted_map.netconvert.log \
  --error-log scenarios/lanelet2_simple_cross/converted_map.netconvert.err.log
```

## Outputs

Generated files:

```text
scenarios/lanelet2_simple_cross/converted_map.nod.xml
scenarios/lanelet2_simple_cross/converted_map.edg.xml
scenarios/lanelet2_simple_cross/converted_map.con.xml
scenarios/lanelet2_simple_cross/converted_map.net.xml
scenarios/lanelet2_simple_cross/converted_map.rou.xml
scenarios/lanelet2_simple_cross/converted_map.sumocfg
scenarios/lanelet2_simple_cross/converted_map.netconvert.log
scenarios/lanelet2_simple_cross/converted_map.netconvert.err.log
scenarios/lanelet2_simple_cross/CONVERSION_REPORT.md
```

The route file defines one NPC vehicle:

```xml
<vehicle id="npc_0" type="npc" route="npc_route" depart="0.0" departLane="0" departPos="0.0" departSpeed="2.0" />
```

The route is:

```text
lanelet_7 lanelet_147 lanelet_154
```

`straight_route` is also defined with the same edge list so the existing
`SumoExternalNpcController` default ego route id can still be used.

## Validation

Network and route validation:

```bash
sumo -c scenarios/lanelet2_simple_cross/converted_map.sumocfg \
  --no-step-log \
  --duration-log.disable true
```

NPC run confirmation:

```bash
sumo -c scenarios/lanelet2_simple_cross/converted_map.sumocfg \
  --no-step-log \
  --duration-log.disable true \
  --tripinfo-output /tmp/lanelet2_simple_cross_tripinfo.xml \
  --fcd-output /tmp/lanelet2_simple_cross_fcd.xml
```

Observed result: `npc_0` departed on `lanelet_7_0`, arrived on
`lanelet_154_0` at `44.30s`, and traversed `128.19m`.

GUI inspection command:

```bash
sumo-gui -c scenarios/lanelet2_simple_cross/converted_map.sumocfg
```

In this headless session, direct `sumo-gui` failed with
`FXApp::openDisplay: unable to open display :1`. `xvfb-run` was also blocked by
the host `/tmp/.X11-unix` ownership/socket state. The generated network and route
are loadable by `sumo`; run the command above on a host with a working display
for visual inspection.

## Controller Use

Use the generated `.sumocfg` with the existing SUMO NPC mode:

```bash
EXTERNAL_CONTROL_NPC_MODE=sumo \
EXTERNAL_CONTROL_SUMOCFG=$PWD/src/scenario_simulator_v2/simulation/sumo_connection/scenarios/lanelet2_simple_cross/converted_map.sumocfg \
EXTERNAL_CONTROL_SUMO_ROUTE_ID=straight_route \
EXTERNAL_CONTROL_SUMO_EDGE_ID=lanelet_7
```

`npc_0` maps directly to a scenario entity named `npc_0`. The ego shadow can use
`straight_route`; its `moveToXY` edge should be `lanelet_7` unless a later phase
adds route-aware ego edge selection.

## Coordinate Handling

The generated `.net.xml` location is:

```text
netOffset="0.00,0.00"
convBoundary="3699.80,73711.52,3814.92,73782.07"
origBoundary="3699.80,73711.52,3814.92,73782.07"
projParameter="!"
```

`netconvert --offset.disable-normalization` preserves the Lanelet2 node
`local_x`/`local_y` values in SUMO. This means SUMO vehicle `x/y` are intended to
be in the same local map frame as the source Lanelet2 file for this map.

The generated SUMO edges use `spreadType="center"`. This is important for
Lanelet2 conversion: the converter writes a lanelet centerline as each SUMO edge
shape, and SUMO's default lane spread shifts a one-lane edge to the right of the
edge shape. `spreadType="center"` keeps the SUMO lane centerline on the Lanelet2
centerline, avoiding a lateral offset when SUMO vehicle poses are mirrored back
into RViz.

No intermediate CommonRoad XML is generated, so there is no
`geoTransformation` or CommonRoad translation value to extract in this workflow.

**Resolved (measured during the cross-map demo):** the SUMO net is in the OSM
`local_x`/`local_y` frame, but traffic_simulator does **not** load the map in that
frame — it projects `simple_cross_map` into **MGRS** (the OSM carries `mgrs_code`
tags). So the simulator entity poses differ from the SUMO `local_x`/`local_y`
values by a constant offset: for this map an Ego on `lanelet_7` reads
`(71670.8, 73652.1)` in the simulator vs `(3726.5, 73727.9)` on the SUMO
centerline. The relationship is a **pure translation** (headings already match —
SUMO heading of `lanelet_7` ≈ 0.476 rad equals the simulator yaw 0.478 rad), so no
rotation is needed:

```text
scenario_x = sumo_x + 67944.34
scenario_y = sumo_y - 75.87
scenario_z = sumo_z + 0.757   (SUMO is 2D; z_offset ≈ ground elevation)
scenario_yaw = sumo_yaw + 0.0
```

This offset is applied at the `SumoExternalNpcController` boundary via the
`EXTERNAL_CONTROL_SUMO_TRANSFORM_*` environment variables (see
`external_control_plugin/README.md`). It is **required** — without it the Ego
shadow is placed ~68 km outside the network and SUMO aborts with a
`SocketException`. A future converter that emits MGRS-projected coordinates (or a
CommonRoad translation) would make these offsets `0`.

## Manual Fixes And Warnings

For `simple_cross_map`:

- `netconvert` warning log: empty.
- Driveable route: yes, for `lanelet_7 -> lanelet_147 -> lanelet_154`.
- Manual route creation: yes, the script selects the longest connected lanelet
  chain and writes a minimal route.
- Manual traffic light correction: not applicable in this phase.

Known limitations:

- Only road lanelet relations are converted.
- Each lanelet becomes one SUMO edge with one lane.
- Only successor connections with matching centerline endpoints are emitted.
- Cross-intersection turns, lane changes, traffic lights, regulatory elements,
  and stop lines are not fully modeled.
- Speed limits are interpreted as km/h, matching Autoware map conventions used
  by scenario_simulator_v2 tests.
