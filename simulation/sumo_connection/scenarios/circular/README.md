# Circular Lanelet2 Test Course

A closed single-lane ring road for exercising the SUMO <-> scenario_simulator_v2
coupling with vehicles that circulate indefinitely. RViz and SUMO use the same
source geometry: `lanelet2_map.osm` is loaded by traffic_simulator, and
`converted_map.net.xml` is generated from that map by
`scripts/convert_lanelet2_to_sumo.py`.

## Layout

- 32 Lanelet2 road relations, `lanelet_1 -> ... -> lanelet_32 -> lanelet_1`.
- Centerline radius 50 m, lane width 3.6 m, centered at the local map origin `(0, 0)`.
- The map uses a sibling `map_projector_info.yaml` with
  `projector_type: TransverseMercator` and `scale_factor: 1.0`, so the
  loader's projected coordinates match the OSM `local_x` / `local_y` tags.
- SUMO conversion keeps the same metric coordinates with
  `netconvert --offset.disable-normalization`.
- Converted SUMO edges use `spreadType="center"` so the SUMO lane centerline
  stays on the Lanelet2 centerline instead of being shifted inward by SUMO's
  default right-side lane spread.

## Files

| File | Role |
| --- | --- |
| `lanelet2_map.osm` | Generated Autoware-style Lanelet2 circular map. |
| `map_projector_info.yaml` | Projector metadata used by traffic_simulator when loading the map. |
| `pointcloud_map.pcd` | Minimal placeholder point cloud required by `CppScenarioNode` map loading. |
| `converted_map.*` | SUMO plain files, net, route stub, config stub, and conversion report from the converter. |
| `circular.rou.xml` | Demo route over `lanelet_1..lanelet_32` with three looping NPCs. |
| `circular.sumocfg` | Long-running SUMO config for TraCI/coupled runs. |

## Rebuild

From the repository root:

```bash
python3 simulation/sumo_connection/scripts/generate_circular_lanelet.py
python3 simulation/sumo_connection/scripts/convert_lanelet2_to_sumo.py \
  --input-map simulation/sumo_connection/scenarios/circular/lanelet2_map.osm \
  --output-dir simulation/sumo_connection/scenarios/circular
```

The converter rewrites `converted_map.*` and `CONVERSION_REPORT.md`. Keep
`circular.rou.xml` and `circular.sumocfg` as the coupled-demo route/config.

## Run Standalone

The config's `end` is intentionally large (`100000 s`) so SUMO does not close
the TraCI socket during a coupled run. For a bounded standalone run, pass `-e`:

```bash
sumo-gui -c circular.sumocfg -e 120
sumo     -c circular.sumocfg -e 120 --fcd-output fcd.xml
```

## Run Coupled

The `external_npc_control_circular` mock loads this directory as its Lanelet2
map, so default RViz lanelet markers show the real ring road. No custom marker
topic, RViz config, or coordinate transform is required. See the circular demo
recipe in `simulation/external_control_plugin/README.md`.
