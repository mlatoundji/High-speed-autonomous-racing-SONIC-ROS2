#!/usr/bin/env python3
"""Generate race_f1_circuit_fenced.world from race_f1_circuit.world.

Places continuous LiDAR-detectable box guardrails along inner/outer track edges
(8 m offset from the 16 m-wide centerline).  Fences aim to fully enclose the
track while keeping all geometry off the drivable surface.
"""

from __future__ import annotations

import argparse
import math
import re
from pathlib import Path

TRACK_HALF_WIDTH = 8.0
# LiDAR scan plane is at z=0.72 m; top of fence stays above that.
FENCE_HEIGHT = 1.10
FENCE_THICKNESS = 1.20
# Keep the inward fence face outside the 16 m drivable corridor.
FENCE_INNER_CLEARANCE = 0.5
FENCE_OFFSET = TRACK_HALF_WIDTH + FENCE_THICKNESS * 0.5 + FENCE_INNER_CLEARANCE
MIN_CENTER_DIST = FENCE_OFFSET
MIN_INNER_FACE_DIST = TRACK_HALF_WIDTH + FENCE_INNER_CLEARANCE
MAX_OFFSET_BOOST = 4.0
OFFSET_BOOST_STEP = 0.25
SPAN_OVERLAP = 0.12
MIN_SPAN_LENGTH = 0.05
CORNER_TURN_DEG = 12.0
CORNER_NEIGHBOR = 2
CORNER_ARC_STEPS = 2
SPAN_SAMPLES = 5
# Gazebo chokes on thousands of collision links; merge short spans into longer ones.
MAX_MERGED_LENGTH = 14.0
MAX_MERGE_ANGLE_DEG = 12.0
MAX_MERGE_GAP = 0.40

FENCE_MAT = (
    '<material><ambient>0.35 0.08 0.02 1</ambient>'
    '<diffuse>0.85 0.18 0.04 1</diffuse>'
    '<specular>0.2 0.2 0.2 1</specular></material>'
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
        out.append((p1[0] + nx * FENCE_OFFSET, p1[1] + ny * FENCE_OFFSET))
    return out


def _dist_point_to_segment(
        px: float, py: float,
        ax: float, ay: float,
        bx: float, by: float) -> float:
    dx, dy = bx - ax, by - ay
    len_sq = dx * dx + dy * dy
    if len_sq < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / len_sq))
    qx = ax + t * dx
    qy = ay + t * dy
    return math.hypot(px - qx, py - qy)


def _min_dist_to_centerline(px: float, py: float, centerline: list[tuple[float, float]]) -> float:
    n = len(centerline)
    return min(
        _dist_point_to_segment(
            px, py,
            centerline[i][0], centerline[i][1],
            centerline[(i + 1) % n][0], centerline[(i + 1) % n][1])
        for i in range(n)
    )


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


def _inward_normal(
        px: float, py: float, yaw: float,
        centerline: list[tuple[float, float]]) -> tuple[float, float]:
    """Return the box normal that points toward the track centerline."""
    candidates = (
        (-math.sin(yaw), math.cos(yaw)),
        (math.sin(yaw), -math.cos(yaw)),
    )
    return min(
        candidates,
        key=lambda n: _min_dist_to_centerline(px + n[0], py + n[1], centerline))


def _inner_face_clearance(
        px: float, py: float, yaw: float,
        centerline: list[tuple[float, float]]) -> float:
    nx, ny = _inward_normal(px, py, yaw, centerline)
    return _min_dist_to_centerline(
        px + nx * FENCE_THICKNESS * 0.5,
        py + ny * FENCE_THICKNESS * 0.5,
        centerline)


def _outward_normal_at_vertex(
        points: list[tuple[float, float]],
        idx: int,
        side: str,
        area: float) -> tuple[float, float]:
    n = len(points)
    p0, p1, p2 = points[(idx - 1) % n], points[idx], points[(idx + 1) % n]
    tx, ty = (p1[0] - p0[0]) + (p2[0] - p1[0]), (p1[1] - p0[1]) + (p2[1] - p1[1])
    norm = math.hypot(tx, ty) or 1.0
    tx, ty = tx / norm, ty / norm
    if area < 0:
        nx, ny = (ty, -tx) if side == 'inner' else (-ty, tx)
    else:
        nx, ny = (-ty, tx) if side == 'inner' else (ty, -tx)
    return nx, ny


def _resolve_edge_point(
        idx: int,
        edge_pts: list[tuple[float, float]],
        centerline: list[tuple[float, float]],
        side: str,
        area: float) -> tuple[float, float]:
    x, y = edge_pts[idx]
    if _min_dist_to_centerline(x, y, centerline) >= MIN_CENTER_DIST:
        return (x, y)

    extra = OFFSET_BOOST_STEP
    while extra <= MAX_OFFSET_BOOST + 1e-9:
        bx, by = _offset_vertex(centerline, idx, side, area, FENCE_OFFSET + extra)
        if _min_dist_to_centerline(bx, by, centerline) >= MIN_CENTER_DIST:
            return (bx, by)
        extra += OFFSET_BOOST_STEP

    nx, ny = _outward_normal_at_vertex(centerline, idx, side, area)
    return (x + nx * MAX_OFFSET_BOOST, y + ny * MAX_OFFSET_BOOST)


def _resolve_edge_polyline(
        edge_pts: list[tuple[float, float]],
        centerline: list[tuple[float, float]],
        side: str,
        area: float) -> list[tuple[float, float]]:
    return [
        _resolve_edge_point(i, edge_pts, centerline, side, area)
        for i in range(len(edge_pts))
    ]


def _span_is_safe(
        ax: float, ay: float,
        bx: float, by: float,
        centerline: list[tuple[float, float]]) -> bool:
    yaw = math.atan2(by - ay, bx - ax)
    for k in range(SPAN_SAMPLES + 1):
        t = k / SPAN_SAMPLES
        px = ax + t * (bx - ax)
        py = ay + t * (by - ay)
        if _min_dist_to_centerline(px, py, centerline) < MIN_CENTER_DIST:
            return False
        if _inner_face_clearance(px, py, yaw, centerline) < MIN_INNER_FACE_DIST:
            return False
    return True


def _lift_span_until_safe(
        ax: float, ay: float,
        bx: float, by: float,
        centerline: list[tuple[float, float]],
        side: str,
        area: float,
        vertex_idx: int) -> list[tuple[tuple[float, float], tuple[float, float]]]:
    nx, ny = _outward_normal_at_vertex(centerline, vertex_idx, side, area)
    lift = 0.0
    while lift <= MAX_OFFSET_BOOST + 1e-9:
        lax = ax + nx * lift
        lay = ay + ny * lift
        lbx = bx + nx * lift
        lby = by + ny * lift
        if _span_is_safe(lax, lay, lbx, lby, centerline):
            return [((lax, lay), (lbx, lby))]
        lift += OFFSET_BOOST_STEP
    return []


def _safe_subspans(
        ax: float, ay: float,
        bx: float, by: float,
        centerline: list[tuple[float, float]],
        side: str,
        area: float,
        vertex_idx: int,
        depth: int = 0) -> list[tuple[tuple[float, float], tuple[float, float]]]:
    if _span_is_safe(ax, ay, bx, by, centerline):
        return [((ax, ay), (bx, by))]
    if depth >= 2:
        return _lift_span_until_safe(ax, ay, bx, by, centerline, side, area, vertex_idx)
    mx, my = (ax + bx) * 0.5, (ay + by) * 0.5
    left = _safe_subspans(ax, ay, mx, my, centerline, side, area, vertex_idx, depth + 1)
    right = _safe_subspans(mx, my, bx, by, centerline, side, area, vertex_idx, depth + 1)
    return left + right


def _edge_fence_spans(
        edge_pts: list[tuple[float, float]],
        centerline: list[tuple[float, float]],
        corner_idxs: set[int],
        side: str,
        area: float) -> list[tuple[tuple[float, float], tuple[float, float]]]:
    """Return closed-chain fence spans along the full edge polyline."""
    safe_pts = _resolve_edge_polyline(edge_pts, centerline, side, area)
    n = len(safe_pts)
    spans: list[tuple[tuple[float, float], tuple[float, float]]] = []

    for i in range(n):
        p0 = safe_pts[i]
        p1 = safe_pts[(i + 1) % n]
        is_corner = i in corner_idxs or (i + 1) % n in corner_idxs
        steps = CORNER_ARC_STEPS + 1 if is_corner else 1
        for s in range(steps):
            t0 = s / steps
            t1 = (s + 1) / steps
            ax = p0[0] + t0 * (p1[0] - p0[0])
            ay = p0[1] + t0 * (p1[1] - p0[1])
            bx = p0[0] + t1 * (p1[0] - p0[0])
            by = p0[1] + t1 * (p1[1] - p0[1])
            sub = _safe_subspans(ax, ay, bx, by, centerline, side, area, i)
            if not sub:
                sub = _lift_span_until_safe(ax, ay, bx, by, centerline, side, area, i)
            spans.extend(sub)

    return _merge_spans(spans, centerline)


def _span_yaw(ax: float, ay: float, bx: float, by: float) -> float:
    return math.atan2(by - ay, bx - ax)


def _angle_diff(a: float, b: float) -> float:
    return abs(math.degrees(math.atan2(math.sin(b - a), math.cos(b - a))))


def _merge_spans(
        spans: list[tuple[tuple[float, float], tuple[float, float]]],
        centerline: list[tuple[float, float]]) -> list[tuple[tuple[float, float], tuple[float, float]]]:
    """Coalesce consecutive colinear spans to keep Gazebo link count reasonable."""
    if not spans:
        return []

    merged: list[tuple[tuple[float, float], tuple[float, float]]] = []
    sx, sy = spans[0][0]
    ex, ey = spans[0][1]
    cur_yaw = _span_yaw(sx, sy, ex, ey)

    for (ax, ay), (bx, by) in spans[1:]:
        gap = math.hypot(ax - ex, ay - ey)
        yaw = _span_yaw(ax, ay, bx, by)
        add_len = math.hypot(bx - ax, by - ay)
        combined = math.hypot(bx - sx, by - sy)
        can_merge = (
            gap <= MAX_MERGE_GAP
            and _angle_diff(cur_yaw, yaw) <= MAX_MERGE_ANGLE_DEG
            and combined <= MAX_MERGED_LENGTH
            and add_len >= MIN_SPAN_LENGTH
        )
        if can_merge and _span_is_safe(sx, sy, bx, by, centerline):
            ex, ey = bx, by
            cur_yaw = _span_yaw(sx, sy, ex, ey)
            continue

        if _span_is_safe(sx, sy, ex, ey, centerline):
            merged.append(((sx, sy), (ex, ey)))
        sx, sy = ax, ay
        ex, ey = bx, by
        cur_yaw = yaw

    if _span_is_safe(sx, sy, ex, ey, centerline):
        merged.append(((sx, sy), (ex, ey)))
    return merged


def fence_link(
        name: str,
        ax: float, ay: float,
        bx: float, by: float,
        centerline: list[tuple[float, float]]) -> str:
    dx, dy = bx - ax, by - ay
    length = math.hypot(dx, dy)
    if length < MIN_SPAN_LENGTH:
        return ''
    ux, uy = dx / length, dy / length
    half_overlap = SPAN_OVERLAP * 0.5
    e_ax = ax - ux * half_overlap
    e_ay = ay - uy * half_overlap
    e_bx = bx + ux * half_overlap
    e_by = by + uy * half_overlap
    if _span_is_safe(e_ax, e_ay, e_bx, e_by, centerline):
        ax, ay, bx, by = e_ax, e_ay, e_bx, e_by
        length += SPAN_OVERLAP
    elif not _span_is_safe(ax, ay, bx, by, centerline):
        return ''

    mx, my = (ax + bx) * 0.5, (ay + by) * 0.5
    yaw = math.atan2(by - ay, bx - ax)
    z = FENCE_HEIGHT * 0.5
    h, w, t = FENCE_HEIGHT, FENCE_THICKNESS, length
    return (
        f'      <link name="{name}">'
        f'<pose>{mx:.2f} {my:.2f} {z:.2f} 0 0 {yaw:.4f}</pose>'
        f'<collision name="col"><geometry><box>'
        f'<size>{t:.2f} {w:.2f} {h:.1f}</size>'
        f'</box></geometry></collision>'
        f'<visual name="vis"><geometry><box>'
        f'<size>{t:.2f} {w:.2f} {h:.1f}</size>'
        f'</box></geometry>{FENCE_MAT}</visual>'
        f'</link>'
    )


def guardrail_model(
        name: str,
        spans: list[tuple[tuple[float, float], tuple[float, float]]],
        centerline: list[tuple[float, float]]) -> str:
    links = []
    for i, ((ax, ay), (bx, by)) in enumerate(spans):
        link = fence_link(f'seg_{i:03d}', ax, ay, bx, by, centerline)
        if link:
            links.append(link)
    body = '\n'.join(links)
    return f"""    <model name="{name}">
      <static>true</static>
{body}
    </model>"""


def _validate_spans(
        spans: list[tuple[tuple[float, float], tuple[float, float]]],
        centerline: list[tuple[float, float]]) -> tuple[int, float]:
    unsafe = 0
    max_gap = 0.0
    for (ax, ay), (bx, by) in spans:
        if not _span_is_safe(ax, ay, bx, by, centerline):
            unsafe += 1
    for i in range(len(spans)):
        _, (ex, ey) = spans[i]
        nx, ny = spans[(i + 1) % len(spans)][0]
        max_gap = max(max_gap, math.hypot(nx - ex, ny - ey))
    return unsafe, max_gap


def build_fenced_world(src_text: str) -> tuple[str, int, int, int, float, float]:
    points = parse_centerline(src_text)
    if not points:
        raise ValueError('no <road name="track"> waypoints found in source world')

    area = signed_area(points)
    corners = _corner_vertex_set(points)
    inner_edge = offset_edge(points, 'inner', area)
    outer_edge = offset_edge(points, 'outer', area)
    inner_spans = _edge_fence_spans(inner_edge, points, corners, 'inner', area)
    outer_spans = _edge_fence_spans(outer_edge, points, corners, 'outer', area)

    fence_xml = f"""
    <!-- ===== Continuous box guardrails ({FENCE_OFFSET:.1f} m center offset, LiDAR-visible) ===== -->
{guardrail_model('track_inner_guardrail', inner_spans, points)}

{guardrail_model('track_outer_guardrail', outer_spans, points)}
"""

    out = src_text.replace(
        'F1-style street circuit (Albert Park inspired), ~750 m centerline',
        'F1-style street circuit (Albert Park inspired), ~750 m centerline, '
        'with continuous LiDAR-detectable box guardrails',
    )
    marker = '    </road>\n\n  </world>'
    if marker not in out:
        raise ValueError('expected </road> before </world> in source world')
    out = out.replace(marker, f'    </road>\n{fence_xml}\n  </world>', 1)

    _, inner_gap = _validate_spans(inner_spans, points)
    _, outer_gap = _validate_spans(outer_spans, points)
    return out, len(corners), len(inner_spans), len(outer_spans), inner_gap, outer_gap


def main() -> None:
    pkg = Path(__file__).resolve().parents[1]
    default_src = pkg / 'worlds' / 'race_f1_circuit.world'
    default_dst = pkg / 'worlds' / 'race_f1_circuit_fenced.world'

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--src', type=Path, default=default_src)
    parser.add_argument('--dst', type=Path, default=default_dst)
    args = parser.parse_args()

    dst_text, corner_count, _, _, inner_gap, outer_gap = build_fenced_world(
        args.src.read_text())
    args.dst.write_text(dst_text)
    inner_block = dst_text.split('track_inner_guardrail')[1].split('track_outer_guardrail')[0]
    outer_block = dst_text.split('track_outer_guardrail')[1]
    inner_count = inner_block.count('<link name="seg_')
    outer_count = outer_block.count('<link name="seg_')
    print(
        f'Wrote {args.dst} (inner={inner_count}, outer={outer_count} fence segments, '
        f'corner_vertices={corner_count}, max_gap=({inner_gap:.2f},{outer_gap:.2f})m, '
        f'fence_offset={FENCE_OFFSET:.1f}m)')


if __name__ == '__main__':
    main()
