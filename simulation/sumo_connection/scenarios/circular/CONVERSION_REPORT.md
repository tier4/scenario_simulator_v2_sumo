# Lanelet2-to-SUMO Conversion Report

- Input map: `simulation/sumo_connection/scenarios/circular/lanelet2_map.osm`
- Road lanelets converted: 32
- Generated route: `lanelet_3 lanelet_4 lanelet_5 lanelet_6 lanelet_7 lanelet_8 lanelet_9 lanelet_10 lanelet_11 lanelet_12 lanelet_13 lanelet_14 lanelet_15 lanelet_16 lanelet_17 lanelet_18 lanelet_19 lanelet_20 lanelet_21 lanelet_22 lanelet_23 lanelet_24 lanelet_25 lanelet_26 lanelet_27 lanelet_28 lanelet_29 lanelet_30 lanelet_31 lanelet_32 lanelet_1 lanelet_2`
- Network coordinate bounds: x=-50.000..50.000, y=-50.000..50.000
- SUMO location: `{'netOffset': '0.00,0.00', 'convBoundary': '-50.00,-50.00,50.00,50.00', 'origBoundary': '-50.00,-50.00,50.00,50.00', 'projParameter': '!'}`
- Coordinate handling: node `local_x`/`local_y` tags were copied into SUMO coordinates and `netconvert --offset.disable-normalization` was used. Edges use `spreadType="center"` so single-lane SUMO lane geometry stays on the Lanelet2 centerline instead of being shifted to the right of it.
- Traffic lights: not converted or synchronized in this phase.

## Connections

- `lanelet_1` -> `lanelet_2`
- `lanelet_2` -> `lanelet_3`
- `lanelet_3` -> `lanelet_4`
- `lanelet_4` -> `lanelet_5`
- `lanelet_5` -> `lanelet_6`
- `lanelet_6` -> `lanelet_7`
- `lanelet_7` -> `lanelet_8`
- `lanelet_8` -> `lanelet_9`
- `lanelet_9` -> `lanelet_10`
- `lanelet_10` -> `lanelet_11`
- `lanelet_11` -> `lanelet_12`
- `lanelet_12` -> `lanelet_13`
- `lanelet_13` -> `lanelet_14`
- `lanelet_14` -> `lanelet_15`
- `lanelet_15` -> `lanelet_16`
- `lanelet_16` -> `lanelet_17`
- `lanelet_17` -> `lanelet_18`
- `lanelet_18` -> `lanelet_19`
- `lanelet_19` -> `lanelet_20`
- `lanelet_20` -> `lanelet_21`
- `lanelet_21` -> `lanelet_22`
- `lanelet_22` -> `lanelet_23`
- `lanelet_23` -> `lanelet_24`
- `lanelet_24` -> `lanelet_25`
- `lanelet_25` -> `lanelet_26`
- `lanelet_26` -> `lanelet_27`
- `lanelet_27` -> `lanelet_28`
- `lanelet_28` -> `lanelet_29`
- `lanelet_29` -> `lanelet_30`
- `lanelet_30` -> `lanelet_31`
- `lanelet_31` -> `lanelet_32`
- `lanelet_32` -> `lanelet_1`

## netconvert warnings

```text
(none)
```
