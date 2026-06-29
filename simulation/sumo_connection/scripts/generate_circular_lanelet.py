#!/usr/bin/env python3
"""Generate an Autoware-style Lanelet2 circular road map.

The generated OSM stores both geographic lat/lon coordinates and Autoware
`local_x` / `local_y` tags.  The sibling `map_projector_info.yaml` tells
traffic_simulator to load the lat/lon through the same TransverseMercator frame,
while the SUMO converter copies `local_x` / `local_y` into the SUMO net.
"""

from __future__ import annotations

import argparse
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

from pyproj import CRS, Transformer


DEFAULT_OUTPUT_DIR = Path(__file__).resolve().parents[1] / "scenarios" / "circular"
DEFAULT_ORIGIN_LAT = 35.681236
DEFAULT_ORIGIN_LON = 139.767125


@dataclass(frozen=True)
class Point:
    x: float
    y: float


def build_transformer(origin_lat: float, origin_lon: float, scale_factor: float) -> Transformer:
    local_crs = CRS.from_proj4(
        " ".join(
            [
                "+proj=tmerc",
                f"+lat_0={origin_lat}",
                f"+lon_0={origin_lon}",
                f"+k={scale_factor}",
                "+x_0=0",
                "+y_0=0",
                "+ellps=WGS84",
                "+units=m",
                "+no_defs",
            ]
        )
    )
    return Transformer.from_crs(local_crs, "EPSG:4326", always_xy=True)


def ring_points(radius: float, segments: int) -> list[Point]:
    # Clockwise travel makes the outer boundary the lanelet left bound and the
    # inner boundary the right bound.
    return [
        Point(
            x=radius * math.cos(-2.0 * math.pi * i / segments),
            y=radius * math.sin(-2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def add_tag(parent: ET.Element, key: str, value: str) -> None:
    ET.SubElement(parent, "tag", k=key, v=value)


def add_node(
    root: ET.Element, node_id: int, point: Point, transformer: Transformer
) -> None:
    lon, lat = transformer.transform(point.x, point.y)
    node = ET.SubElement(root, "node", id=str(node_id), lat=f"{lat:.11f}", lon=f"{lon:.11f}")
    add_tag(node, "local_x", f"{point.x:.6f}")
    add_tag(node, "local_y", f"{point.y:.6f}")
    add_tag(node, "ele", "0.000000")


def add_way(root: ET.Element, way_id: int, start_node_id: int, end_node_id: int) -> None:
    way = ET.SubElement(root, "way", id=str(way_id))
    ET.SubElement(way, "nd", ref=str(start_node_id))
    ET.SubElement(way, "nd", ref=str(end_node_id))
    add_tag(way, "type", "line_thin")
    add_tag(way, "subtype", "solid")


def add_lanelet(
    root: ET.Element,
    lanelet_id: int,
    left_way_id: int,
    right_way_id: int,
    speed_limit_kph: float,
) -> None:
    relation = ET.SubElement(root, "relation", id=str(lanelet_id))
    ET.SubElement(relation, "member", type="way", role="left", ref=str(left_way_id))
    ET.SubElement(relation, "member", type="way", role="right", ref=str(right_way_id))
    add_tag(relation, "type", "lanelet")
    add_tag(relation, "subtype", "road")
    add_tag(relation, "speed_limit", f"{speed_limit_kph:g}")


def write_lanelet_map(
    path: Path,
    *,
    radius: float,
    width: float,
    segments: int,
    speed_limit_kph: float,
    origin_lat: float,
    origin_lon: float,
    scale_factor: float,
) -> None:
    transformer = build_transformer(origin_lat, origin_lon, scale_factor)
    outer_points = ring_points(radius + width / 2.0, segments)
    inner_points = ring_points(radius - width / 2.0, segments)

    root = ET.Element("osm", generator="generate_circular_lanelet.py")
    ET.SubElement(root, "MetaInfo", format_version="1", map_version="2")

    outer_node_base = 10000
    inner_node_base = 20000
    outer_way_base = 30000
    inner_way_base = 40000

    for index, point in enumerate(outer_points):
        add_node(root, outer_node_base + index, point, transformer)
    for index, point in enumerate(inner_points):
        add_node(root, inner_node_base + index, point, transformer)

    for index in range(segments):
        next_index = (index + 1) % segments
        add_way(
            root,
            outer_way_base + index,
            outer_node_base + index,
            outer_node_base + next_index,
        )
        add_way(
            root,
            inner_way_base + index,
            inner_node_base + index,
            inner_node_base + next_index,
        )
        add_lanelet(
            root,
            index + 1,
            outer_way_base + index,
            inner_way_base + index,
            speed_limit_kph,
        )

    tree = ET.ElementTree(root)
    ET.indent(tree, space="  ")
    tree.write(path, encoding="utf-8", xml_declaration=True)


def write_projector_info(
    path: Path, *, origin_lat: float, origin_lon: float, scale_factor: float
) -> None:
    path.write_text(
        "\n".join(
            [
                "projector_type: TransverseMercator",
                "map_origin:",
                f"  latitude: {origin_lat:.11f}",
                f"  longitude: {origin_lon:.11f}",
                f"scale_factor: {scale_factor:.1f}",
                "",
            ]
        ),
        encoding="utf-8",
    )


def write_placeholder_pcd(path: Path) -> None:
    path.write_text(
        "\n".join(
            [
                "# .PCD v0.7 - Point Cloud Data file format",
                "VERSION 0.7",
                "FIELDS x y z",
                "SIZE 4 4 4",
                "TYPE F F F",
                "COUNT 1 1 1",
                "WIDTH 1",
                "HEIGHT 1",
                "VIEWPOINT 0 0 0 1 0 0 0",
                "POINTS 1",
                "DATA ascii",
                "0.0 0.0 0.0",
                "",
            ]
        ),
        encoding="utf-8",
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--radius", type=float, default=50.0)
    parser.add_argument("--width", type=float, default=3.6)
    parser.add_argument("--segments", type=int, default=32)
    parser.add_argument("--speed-limit-kph", type=float, default=50.0)
    parser.add_argument("--origin-lat", type=float, default=DEFAULT_ORIGIN_LAT)
    parser.add_argument("--origin-lon", type=float, default=DEFAULT_ORIGIN_LON)
    parser.add_argument("--scale-factor", type=float, default=1.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.segments < 3:
        raise ValueError("--segments must be at least 3")
    if args.width <= 0.0:
        raise ValueError("--width must be positive")
    if args.radius <= args.width / 2.0:
        raise ValueError("--radius must be larger than half the lane width")

    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    write_lanelet_map(
        output_dir / "lanelet2_map.osm",
        radius=args.radius,
        width=args.width,
        segments=args.segments,
        speed_limit_kph=args.speed_limit_kph,
        origin_lat=args.origin_lat,
        origin_lon=args.origin_lon,
        scale_factor=args.scale_factor,
    )
    write_projector_info(
        output_dir / "map_projector_info.yaml",
        origin_lat=args.origin_lat,
        origin_lon=args.origin_lon,
        scale_factor=args.scale_factor,
    )
    write_placeholder_pcd(output_dir / "pointcloud_map.pcd")
    print(f"Wrote circular Lanelet2 map to {output_dir}")


if __name__ == "__main__":
    main()
