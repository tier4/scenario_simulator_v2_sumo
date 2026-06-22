# Lanelet2-to-SUMO Conversion Report

- Input map: `lanelet2_map.osm`
- Road lanelets converted: 8
- Generated route: `lanelet_7 lanelet_147 lanelet_154`
- Network coordinate bounds: x=3699.802..3814.917, y=73711.521..73782.065
- SUMO location: `{'netOffset': '0.00,0.00', 'convBoundary': '3699.80,73711.52,3814.92,73782.07', 'origBoundary': '3699.80,73711.52,3814.92,73782.07', 'projParameter': '!'}`
- Coordinate handling: node `local_x`/`local_y` tags were copied into SUMO coordinates and `netconvert --offset.disable-normalization` was used. Edges use `spreadType="center"` so single-lane SUMO lane geometry stays on the Lanelet2 centerline instead of being shifted to the right of it.
- Traffic lights: not converted or synchronized in this phase.

## Connections

- `lanelet_7` -> `lanelet_147`
- `lanelet_14` -> `lanelet_133`
- `lanelet_18` -> (none)
- `lanelet_22` -> (none)
- `lanelet_133` -> `lanelet_140`
- `lanelet_140` -> (none)
- `lanelet_147` -> `lanelet_154`
- `lanelet_154` -> (none)

## netconvert warnings

```text
(none)
```
