#!/usr/bin/env python3
"""Generate race_f1_circuit_fenced.world from race_f1_circuit.world.

Places LiDAR-detectable vertical cylinder bollards along the inner/outer track
edges (8 m offset from the 16 m-wide centerline).
"""

from __future__ import annotations

import argparse
import math
import re
from pathlib import Path

HALF_WIDTH = 8.0
# Drop bollards that fold onto the asphalt at tight corners (min dist to any
# centerline sample must stay near the nominal edge offset).
MIN_EDGE_DIST = HALF_WIDTH - 0.5
# LiDAR scan plane is at z=0.72 m; keep top slightly above for pitch margin.
POST_HEIGHT = 0.80
POST_RADIUS = 0.20
# Place every Nth waypoint on straights; corners use stride 1 + extra fill.
BOLLARD_STRIDE = 2
CORNER_TURN_DEG = 15.0
CORNER_NEIGHBOR = 2
CORNER_ARC_STEPS = 6
CORNER_EXTRA_OFFSET = 1.0
MIN_POST_SPACING = 0.75

MAT = (
    '<material><ambient>0.35 0.08 0.02 1</ambient>'
    '<diffuse>0.85 0.18 0.04 1</diffuse>'
    '<specular>0.3 0.3 0.3 1</specular></material>'
)


def parse_centerline(world_text: str) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    in_road = False
    for line in world_text.splitlines():
        if '<road name="track">' in line:
            in_road = True
            continue
        if in_road and '</road>' in line:
            break
        if in_road:
            m = re.search(
                r'<point>([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)</point>', line)
            if m:
                points.append((float(m.group(1)), float(m.group(2))))

    if len(points) > 1 and math.hypot(
            points[0][0] - points[-1][0],
            points[0][1] - points[-1][1]) < 0.01:
        points = points[:-1]
    return points


def signed_area(points: list[tuple[float, float]]) -> float:
    return 0.5 * sum(
        points[i][0] * points[(i + 1) % len(points)][1]
        - points[(i + 1) % len(points)][0] * points[i][1]
        for i in range(len(points))
    )


def offset_edge(
        points: list[tuple[float, float]],
        side: str,
        area: float) -> list[tuple[float, float]]:
    out: list[tuple[float, float]] = []
    n = len(points)
    for i in range(n):
        p0, p1, p2 = points[(i - 1) % n], points[i], points[(i + 1) % n]
        tx, ty = (p1[0] - p0[0]) + (p2[0] - p1[0]), (p1[1] - p0[1]) + (p2[1] - p1[1])
        norm = math.hypot(tx, ty) or 1.0
        tx, ty = tx / norm, ty / norm
        if area < 0:
            nx, ny = (ty, -tx) if side == 'inner' else (-ty, tx)
        else:
            nx, ny = (-ty, tx) if side == 'inner' else (ty, -tx)
        out.append((p1[0] + nx * HALF_WIDTH, p1[1] + ny * HALF_WIDTH))
    return out


def _min_dist_to_polyline(px: float, py: float, centerline: list[tuple[float, float]]) -> float:
    return min(math.hypot(px - cx, py - cy) for cx, cy in centerline)


def _vertex_turn_degrees(points: list[tuple[float, float]]) -> list[float]:
    n = len(points)
    turns: list[float] = []
    for i in range(n):
        p0, p1, p2 = points[(i - 1) % n], points[i], points[(i + 1) % n]
        a1 = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
        a2 = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
        turns.append(abs(math.degrees(math.atan2(
            math.sin(a2 - a1), math.cos(a2 - a1)))))
    return turns


def _corner_vertex_set(points: list[tuple[float, float]]) -> set[int]:
    turns = _vertex_turn_degrees(points)
    corners: set[int] = set()
    n = len(points)
    for i, turn in enumerate(turns):
        if turn < CORNER_TURN_DEG:
            continue
        for d in range(-CORNER_NEIGHBOR, CORNER_NEIGHBOR + 1):
            corners.add((i + d) % n)
    return corners


def _offset_vertex(
        points: list[tuple[float, float]],
        idx: int,
        side: str,
        area: float,
        half_width: float) -> tuple[float, float]:
    n = len(points)
    p0, p1, p2 = points[(idx - 1) % n], points[idx], points[(idx + 1) % n]
    tx, ty = (p1[0] - p0[0]) + (p2[0] - p1[0]), (p1[1] - p0[1]) + (p2[1] - p1[1])
    norm = math.hypot(tx, ty) or 1.0
    tx, ty = tx / norm, ty / norm
    if area < 0:
        nx, ny = (ty, -tx) if side == 'inner' else (-ty, tx)
    else:
        nx, ny = (-ty, tx) if side == 'inner' else (ty, -tx)
    return (p1[0] + nx * half_width, p1[1] + ny * half_width)


def _dedupe_posts(
        posts: list[tuple[float, float]],
        min_spacing: float = MIN_POST_SPACING) -> list[tuple[float, float]]:
    kept: list[tuple[float, float]] = []
    for x, y in posts:
        if all(math.hypot(x - px, y - py) >= min_spacing for px, py in kept):
            kept.append((x, y))
    return kept


def _densify_corner_arc(
        edge_pts: list[tuple[float, float]],
        corner_idxs: set[int]) -> list[tuple[float, float]]:
    """Add interpolated bollards along the edge polyline around sharp corners."""
    n = len(edge_pts)
    extra: list[tuple[float, float]] = []
    for i in corner_idxs:
        p_prev = edge_pts[(i - 1) % n]
        p_curr = edge_pts[i]
        p_next = edge_pts[(i + 1) % n]
        for leg_start, leg_end in ((p_prev, p_curr), (p_curr, p_next)):
            for step in range(1, CORNER_ARC_STEPS + 1):
                t = step / (CORNER_ARC_STEPS + 1)
                extra.append((
                    leg_start[0] + t * (leg_end[0] - leg_start[0]),
                    leg_start[1] + t * (leg_end[1] - leg_start[1]),
                ))
    return extra


def _filter_edge_posts(
        edge_pts: list[tuple[float, float]],
        centerline: list[tuple[float, float]],
        corner_idxs: set[int],
        side: str,
        area: float) -> list[tuple[float, float]]:
    """Keep bollards outside the corridor; densify and boost offset at corners."""
    kept: list[tuple[float, float]] = []
    for i, (x, y) in enumerate(edge_pts):
        stride = 1 if i in corner_idxs else BOLLARD_STRIDE
        if i % stride != 0:
            continue
        if _min_dist_to_polyline(x, y, centerline) >= MIN_EDGE_DIST:
            kept.append((x, y))
            continue
        if i not in corner_idxs:
            continue
        boosted = _offset_vertex(
            centerline, i, side, area, HALF_WIDTH + CORNER_EXTRA_OFFSET)
        if _min_dist_to_polyline(boosted[0], boosted[1], centerline) >= MIN_EDGE_DIST:
            kept.append(boosted)

    for x, y in _densify_corner_arc(edge_pts, corner_idxs):
        if _min_dist_to_polyline(x, y, centerline) >= MIN_EDGE_DIST:
            kept.append((x, y))
    return _dedupe_posts(kept)


def cylinder_post(name: str, x: float, y: float) -> str:
    z = POST_HEIGHT / 2
    r, h = POST_RADIUS, POST_HEIGHT
    return (
        f'      <link name="{name}">'
        f'<pose>{x:.2f} {y:.2f} {z:.2f} 0 0 0</pose>'
        f'<collision name="col"><geometry><cylinder>'
        f'<radius>{r:.2f}</radius><length>{h:.1f}</length>'
        f'</cylinder></geometry></collision>'
        f'<visual name="vis"><geometry><cylinder>'
        f'<radius>{r:.2f}</radius><length>{h:.1f}</length>'
        f'</cylinder></geometry>{MAT}</visual>'
        f'</link>'
    )


def bollard_model(name: str, edge_pts: list[tuple[float, float]]) -> str:
    links = '\n'.join(
        cylinder_post(f'post_{i:03d}', x, y) for i, (x, y) in enumerate(edge_pts))
    return f"""    <model name="{name}">
      <static>true</static>
{links}
    </model>"""


def build_fenced_world(src_text: str) -> tuple[str, int]:
    points = parse_centerline(src_text)
    if not points:
        raise ValueError('no <road name="track"> waypoints found in source world')

    area = signed_area(points)
    corners = _corner_vertex_set(points)
    inner_edge = offset_edge(points, 'inner', area)
    outer_edge = offset_edge(points, 'outer', area)
    inner = _filter_edge_posts(inner_edge, points, corners, 'inner', area)
    outer = _filter_edge_posts(outer_edge, points, corners, 'outer', area)

    bollard_xml = f"""
    <!-- ===== Cylinder bollards along inner/outer track edges ({HALF_WIDTH:.0f} m offset, LiDAR-visible) ===== -->
{bollard_model('track_inner_bollards', inner)}

{bollard_model('track_outer_bollards', outer)}
"""

    out = src_text.replace(
        'F1-style street circuit (Albert Park inspired), ~750 m centerline',
        'F1-style street circuit (Albert Park inspired), ~750 m centerline, '
        'with LiDAR-detectable cylinder bollards',
    )
    marker = '    </road>\n\n  </world>'
    if marker not in out:
        raise ValueError('expected </road> before </world> in source world')
    out = out.replace(marker, f'    </road>\n{bollard_xml}\n  </world>', 1)
    return out, len(corners)


def main() -> None:
    pkg = Path(__file__).resolve().parents[1]
    default_src = pkg / 'worlds' / 'race_f1_circuit.world'
    default_dst = pkg / 'worlds' / 'race_f1_circuit_fenced.world'

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--src', type=Path, default=default_src)
    parser.add_argument('--dst', type=Path, default=default_dst)
    args = parser.parse_args()

    dst_text, corner_count = build_fenced_world(args.src.read_text())
    args.dst.write_text(dst_text)
    inner_count = dst_text.split('track_inner_bollards')[1].split('track_outer_bollards')[0].count('<link name="post_')
    outer_count = dst_text.split('track_outer_bollards')[1].count('<link name="post_')
    print(
        f'Wrote {args.dst} (inner={inner_count}, outer={outer_count} cylinders, '
        f'stride={BOLLARD_STRIDE}, corner_vertices={corner_count}, '
        f'corner_turn>={CORNER_TURN_DEG}deg, min_edge={MIN_EDGE_DIST}m)')


if __name__ == '__main__':
    main()
