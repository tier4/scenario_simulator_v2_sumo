#!/usr/bin/env python3
"""Convert an Autoware Lanelet2 OSM map into a minimal SUMO scenario.

This is a phase-1 reproducibility tool for the `sumo_connection` prototype. It
supports the Autoware-style Lanelet2 OSM files in this workspace that store
projected map coordinates on nodes as `local_x`, `local_y`, and optional `ele`
tags.
"""

from __future__ import annotations

import argparse
import math
import re
import subprocess
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


DEFAULT_INPUT = (
    Path(__file__).resolve().parents[3]
    / "map"
    / "simple_cross_map"
    / "map"
    / "lanelet2_map.osm"
)
DEFAULT_OUTPUT_DIR = Path(__file__).resolve().parents[1] / "scenarios" / "lanelet2_simple_cross"
DEFAULT_SPEED_MPS = 10.0 / 3.6
POINT_KEY_RESOLUTION_M = 0.01
BOUNDARY_ORIENTATION_EPSILON_M = 0.01


@dataclass(frozen=True)
class Point:
    x: float
    y: float
    z: float = 0.0


@dataclass(frozen=True)
class Node:
    id: str
    point: Point
    mgrs_code: str | None


@dataclass(frozen=True)
class Way:
    id: str
    node_refs: list[str]
    tags: dict[str, str]


@dataclass(frozen=True)
class Lanelet:
    id: str
    left_way_id: str
    right_way_id: str
    tags: dict[str, str]
    centerline: list[Point]
    speed_mps: float

    @property
    def edge_id(self) -> str:
        return f"lanelet_{self.id}"


def tags(element: ET.Element) -> dict[str, str]:
    return {tag.get("k", ""): tag.get("v", "") for tag in element.findall("tag")}


def parse_speed_mps(raw: str | None) -> float:
    if not raw:
        return DEFAULT_SPEED_MPS
    match = re.search(r"[-+]?\d+(?:\.\d+)?", raw)
    if not match:
        return DEFAULT_SPEED_MPS
    return float(match.group(0)) / 3.6


def parse_node(element: ET.Element) -> Node:
    node_tags = tags(element)
    missing = [key for key in ("local_x", "local_y") if key not in node_tags]
    if missing:
        raise ValueError(
            f"node {element.get('id')} is missing {', '.join(missing)}; "
            "this converter currently requires Autoware local_x/local_y tags"
        )
    return Node(
        id=require_attr(element, "id"),
        point=Point(
            x=float(node_tags["local_x"]),
            y=float(node_tags["local_y"]),
            z=float(node_tags.get("ele", "0.0")),
        ),
        mgrs_code=node_tags.get("mgrs_code"),
    )


def require_attr(element: ET.Element, name: str) -> str:
    value = element.get(name)
    if value is None:
        raise ValueError(f"<{element.tag}> is missing required attribute {name!r}")
    return value


def parse_map(path: Path) -> tuple[dict[str, Node], dict[str, Way], list[tuple[str, str, str, dict[str, str]]]]:
    root = ET.parse(path).getroot()
    nodes = {require_attr(node, "id"): parse_node(node) for node in root.findall("node")}
    ways = {
        require_attr(way, "id"): Way(
            id=require_attr(way, "id"),
            node_refs=[require_attr(nd, "ref") for nd in way.findall("nd")],
            tags=tags(way),
        )
        for way in root.findall("way")
    }

    lanelet_relations: list[tuple[str, str, str, dict[str, str]]] = []
    for relation in root.findall("relation"):
        relation_tags = tags(relation)
        if relation_tags.get("type") != "lanelet" or relation_tags.get("subtype") != "road":
            continue
        members = {
            require_attr(member, "role"): require_attr(member, "ref")
            for member in relation.findall("member")
            if member.get("type") == "way"
        }
        if "left" not in members or "right" not in members:
            raise ValueError(f"lanelet relation {relation.get('id')} is missing left/right way members")
        lanelet_relations.append((require_attr(relation, "id"), members["left"], members["right"], relation_tags))

    if not lanelet_relations:
        raise ValueError(f"no road lanelet relations found in {path}")
    return nodes, ways, lanelet_relations


def length(points: list[Point]) -> float:
    return sum(distance(a, b) for a, b in zip(points, points[1:]))


def distance(a: Point, b: Point) -> float:
    return math.hypot(b.x - a.x, b.y - a.y)


def interpolate(points: list[Point], fraction: float) -> Point:
    if len(points) == 1:
        return points[0]
    target = max(0.0, min(1.0, fraction)) * length(points)
    walked = 0.0
    for start, end in zip(points, points[1:]):
        segment = distance(start, end)
        if segment == 0.0:
            continue
        if walked + segment >= target:
            local = (target - walked) / segment
            return Point(
                x=start.x + (end.x - start.x) * local,
                y=start.y + (end.y - start.y) * local,
                z=start.z + (end.z - start.z) * local,
            )
        walked += segment
    return points[-1]


def centerline(left: list[Point], right: list[Point]) -> list[Point]:
    left, right = orient_boundaries(left, right)
    sample_count = max(len(left), len(right), 2)
    result = []
    for index in range(sample_count):
        fraction = index / (sample_count - 1)
        left_point = interpolate(left, fraction)
        right_point = interpolate(right, fraction)
        result.append(
            Point(
                x=(left_point.x + right_point.x) / 2.0,
                y=(left_point.y + right_point.y) / 2.0,
                z=(left_point.z + right_point.z) / 2.0,
            )
        )
    return result


def endpoint_width_sum(left: list[Point], right: list[Point]) -> float:
    return distance(left[0], right[0]) + distance(left[-1], right[-1])


def orient_boundaries(left: list[Point], right: list[Point]) -> tuple[list[Point], list[Point]]:
    """Orient lanelet bounds consistently before averaging them into a centerline.

    Some Autoware Lanelet2 OSM files store the left and right boundary ways in
    opposite node order. Averaging opposite-oriented bounds produces long
    diagonal artifacts instead of a lane centerline, so use the orientation that
    minimizes the start/end boundary widths.
    """
    same_direction_width = endpoint_width_sum(left, right)
    reversed_right = list(reversed(right))
    reversed_width = endpoint_width_sum(left, reversed_right)
    if reversed_width + BOUNDARY_ORIENTATION_EPSILON_M < same_direction_width:
        return left, reversed_right
    return left, right


def build_lanelets(
    nodes: dict[str, Node],
    ways: dict[str, Way],
    lanelet_relations: Iterable[tuple[str, str, str, dict[str, str]]],
) -> list[Lanelet]:
    lanelets = []
    for lanelet_id, left_way_id, right_way_id, lanelet_tags in lanelet_relations:
        left_way = ways[left_way_id]
        right_way = ways[right_way_id]
        left = [nodes[node_ref].point for node_ref in left_way.node_refs]
        right = [nodes[node_ref].point for node_ref in right_way.node_refs]
        lanelets.append(
            Lanelet(
                id=lanelet_id,
                left_way_id=left_way_id,
                right_way_id=right_way_id,
                tags=lanelet_tags,
                centerline=centerline(left, right),
                speed_mps=parse_speed_mps(lanelet_tags.get("speed_limit")),
            )
        )
    return sorted(lanelets, key=lambda lanelet: int(lanelet.id) if lanelet.id.isdigit() else lanelet.id)


def point_key(point: Point) -> tuple[int, int]:
    return (
        round(point.x / POINT_KEY_RESOLUTION_M),
        round(point.y / POINT_KEY_RESOLUTION_M),
    )


def plain_nodes(lanelets: list[Lanelet]) -> tuple[ET.Element, dict[tuple[int, int], str]]:
    root = ET.Element("nodes")
    node_ids: dict[tuple[int, int], str] = {}
    node_points: dict[tuple[int, int], Point] = {}
    for lanelet in lanelets:
        for point in (lanelet.centerline[0], lanelet.centerline[-1]):
            key = point_key(point)
            if key not in node_ids:
                node_ids[key] = f"j_{len(node_ids)}"
                node_points[key] = point

    for key, node_id in sorted(node_ids.items(), key=lambda item: item[1]):
        point = node_points[key]
        ET.SubElement(root, "node", id=node_id, x=f"{point.x:.3f}", y=f"{point.y:.3f}", type="priority")
    return root, node_ids


def plain_edges(lanelets: list[Lanelet], node_ids: dict[tuple[int, int], str]) -> ET.Element:
    root = ET.Element("edges")
    for lanelet in lanelets:
        shape = " ".join(f"{point.x:.3f},{point.y:.3f}" for point in lanelet.centerline)
        ET.SubElement(
            root,
            "edge",
            id=lanelet.edge_id,
            from_=node_ids[point_key(lanelet.centerline[0])],
            to=node_ids[point_key(lanelet.centerline[-1])],
            priority="1",
            numLanes="1",
            speed=f"{lanelet.speed_mps:.3f}",
            spreadType="center",
            shape=shape,
        )
    for edge in root.findall("edge"):
        edge.set("from", edge.attrib.pop("from_"))
    return root


def successor_map(lanelets: list[Lanelet]) -> dict[str, list[str]]:
    starts: dict[tuple[int, int], list[Lanelet]] = {}
    for lanelet in lanelets:
        starts.setdefault(point_key(lanelet.centerline[0]), []).append(lanelet)

    successors: dict[str, list[str]] = {lanelet.id: [] for lanelet in lanelets}
    for lanelet in lanelets:
        for successor in starts.get(point_key(lanelet.centerline[-1]), []):
            if successor.id != lanelet.id:
                successors[lanelet.id].append(successor.id)
    return successors


def plain_connections(lanelets: list[Lanelet], successors: dict[str, list[str]]) -> ET.Element:
    root = ET.Element("connections")
    lanelet_by_id = {lanelet.id: lanelet for lanelet in lanelets}
    for lanelet in lanelets:
        for successor_id in successors[lanelet.id]:
            ET.SubElement(root, "connection", **{"from": lanelet.edge_id, "to": lanelet_by_id[successor_id].edge_id})
    return root


def longest_route(lanelets: list[Lanelet], successors: dict[str, list[str]]) -> list[str]:
    lanelet_by_id = {lanelet.id: lanelet for lanelet in lanelets}

    def score(route: list[str]) -> tuple[int, float]:
        return (len(route), sum(length(lanelet_by_id[lanelet_id].centerline) for lanelet_id in route))

    def walk(lanelet_id: str, visited: set[str]) -> list[str]:
        candidates = []
        for successor_id in successors[lanelet_id]:
            if successor_id in visited:
                continue
            candidates.append([lanelet_id, *walk(successor_id, visited | {successor_id})])
        if not candidates:
            return [lanelet_id]
        return max(candidates, key=score)

    return max((walk(lanelet.id, {lanelet.id}) for lanelet in lanelets), key=score)


def write_xml(path: Path, root: ET.Element) -> None:
    tree = ET.ElementTree(root)
    ET.indent(tree, space="  ")
    tree.write(path, encoding="utf-8", xml_declaration=True)


def write_routes(path: Path, route_edge_ids: list[str]) -> None:
    root = ET.Element("routes")
    ET.SubElement(root, "vType", id="npc", vClass="passenger", length="4.5", accel="2.0", decel="4.5", maxSpeed="8.0")
    ET.SubElement(root, "vType", id="ego_shadow", vClass="passenger", length="4.5", accel="2.0", decel="4.5", maxSpeed="8.0")
    ET.SubElement(root, "route", id="straight_route", edges=" ".join(route_edge_ids))
    ET.SubElement(root, "route", id="npc_route", edges=" ".join(route_edge_ids))
    ET.SubElement(
        root,
        "vehicle",
        id="npc_0",
        type="npc",
        route="npc_route",
        depart="0.0",
        departLane="0",
        departPos="0.0",
        departSpeed="2.0",
    )
    write_xml(path, root)


def write_sumocfg(path: Path, include_routes: bool = True) -> None:
    root = ET.Element("configuration")
    input_element = ET.SubElement(root, "input")
    ET.SubElement(input_element, "net-file", value="converted_map.net.xml")
    if include_routes:
        ET.SubElement(input_element, "route-files", value="converted_map.rou.xml")
    time_element = ET.SubElement(root, "time")
    ET.SubElement(time_element, "begin", value="0.0")
    ET.SubElement(time_element, "end", value="60.0")
    ET.SubElement(time_element, "step-length", value="0.1")
    write_xml(path, root)


def read_location(net_path: Path) -> dict[str, str]:
    root = ET.parse(net_path).getroot()
    location = root.find("location")
    return dict(location.attrib) if location is not None else {}


def run_netconvert(output_dir: Path) -> None:
    # Run from output_dir with bare filenames so the generated net.xml records
    # portable, relative provenance paths instead of an absolute local path.
    cmd = [
        "netconvert",
        "--node-files",
        "converted_map.nod.xml",
        "--edge-files",
        "converted_map.edg.xml",
        "--connection-files",
        "converted_map.con.xml",
        "--output-file",
        "converted_map.net.xml",
        "--offset.disable-normalization",
        "--plain.extend-edge-shape",
        "--log",
        "converted_map.netconvert.log",
        "--error-log",
        "converted_map.netconvert.err.log",
    ]
    subprocess.run(cmd, check=True, cwd=output_dir)


def write_report(
    path: Path,
    input_map: Path,  # display path (repo-relative)
    lanelets: list[Lanelet],
    successors: dict[str, list[str]],
    route_ids: list[str] | None,
    location: dict[str, str],
) -> None:
    all_points = [point for lanelet in lanelets for point in lanelet.centerline]
    min_x = min(point.x for point in all_points)
    min_y = min(point.y for point in all_points)
    max_x = max(point.x for point in all_points)
    max_y = max(point.y for point in all_points)
    edge_ids = [f"lanelet_{lanelet_id}" for lanelet_id in route_ids] if route_ids else []
    warnings_path = path.parent / "converted_map.netconvert.err.log"
    warnings = warnings_path.read_text(encoding="utf-8").strip() if warnings_path.exists() else ""

    path.write_text(
        "\n".join(
            [
                "# Lanelet2-to-SUMO Conversion Report",
                "",
                f"- Input map: `{input_map}`",
                f"- Road lanelets converted: {len(lanelets)}",
                f"- Generated route: `{' '.join(edge_ids)}`" if edge_ids else "- Generated route: not generated",
                f"- Network coordinate bounds: x={min_x:.3f}..{max_x:.3f}, y={min_y:.3f}..{max_y:.3f}",
                f"- SUMO location: `{location}`",
                "- Coordinate handling: node `local_x`/`local_y` tags were copied into SUMO coordinates and "
                "`netconvert --offset.disable-normalization` was used. Edges use `spreadType=\"center\"` so "
                "single-lane SUMO lane geometry stays on the Lanelet2 centerline instead of being shifted to "
                "the right of it.",
                "- Traffic lights: not converted or synchronized in this phase.",
                "",
                "## Connections",
                "",
                *[
                    f"- `lanelet_{lanelet.id}` -> "
                    + (", ".join(f"`lanelet_{successor_id}`" for successor_id in successors[lanelet.id]) or "(none)")
                    for lanelet in lanelets
                ],
                "",
                "## netconvert warnings",
                "",
                "```text",
                warnings or "(none)",
                "```",
                "",
            ]
        ),
        encoding="utf-8",
    )


def convert(input_map: Path, output_dir: Path, generate_route: bool = True) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    nodes, ways, lanelet_relations = parse_map(input_map)
    lanelets = build_lanelets(nodes, ways, lanelet_relations)
    successors = successor_map(lanelets)
    route_ids = longest_route(lanelets, successors) if generate_route else None
    route_edge_ids = [f"lanelet_{lanelet_id}" for lanelet_id in route_ids] if route_ids else []

    nod_root, node_ids = plain_nodes(lanelets)
    write_xml(output_dir / "converted_map.nod.xml", nod_root)
    write_xml(output_dir / "converted_map.edg.xml", plain_edges(lanelets, node_ids))
    write_xml(output_dir / "converted_map.con.xml", plain_connections(lanelets, successors))
    if generate_route:
        write_routes(output_dir / "converted_map.rou.xml", route_edge_ids)
    write_sumocfg(output_dir / "converted_map.sumocfg", include_routes=generate_route)
    run_netconvert(output_dir)
    repo_root = Path(__file__).resolve().parents[3]
    try:
        input_display = input_map.relative_to(repo_root)
    except ValueError:
        input_display = Path(input_map.name)
    write_report(
        output_dir / "CONVERSION_REPORT.md",
        input_display,
        lanelets,
        successors,
        route_ids,
        read_location(output_dir / "converted_map.net.xml"),
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-map", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument(
        "--map-only",
        action="store_true",
        help="Generate the SUMO network without converted_map.rou.xml, default routes, or npc_0.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    convert(args.input_map.resolve(), args.output_dir.resolve(), generate_route=not args.map_only)
    print(f"Wrote SUMO scenario to {args.output_dir.resolve()}")


if __name__ == "__main__":
    main()
