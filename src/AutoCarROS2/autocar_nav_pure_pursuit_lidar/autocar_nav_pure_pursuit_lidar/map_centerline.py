"""Extract a closed-loop centerline from occupancy map via EDT + skeleton."""

from __future__ import annotations

import math
from collections import deque

import cv2
import numpy as np

from autocar_nav_pure_pursuit_lidar.map_track_geometry import refine_closed_centerline_from_map
from autocar_nav_pure_pursuit_lidar.pure_pursuit import forward_vector


def _moving_average_closed(xs: np.ndarray, ys: np.ndarray, passes: int) -> tuple[np.ndarray, np.ndarray]:
    if passes <= 0:
        return xs, ys
    out_x, out_y = xs.copy(), ys.copy()
    for _ in range(passes):
        out_x = 0.25 * np.roll(out_x, 1) + 0.5 * out_x + 0.25 * np.roll(out_x, -1)
        out_y = 0.25 * np.roll(out_y, 1) + 0.5 * out_y + 0.25 * np.roll(out_y, -1)
    return out_x, out_y


def _grid_to_world(info, row: int, col: int) -> tuple[float, float]:
    x = info.origin.position.x + (col + 0.5) * info.resolution
    y = info.origin.position.y + (row + 0.5) * info.resolution
    return float(x), float(y)


def _world_to_grid(info, x: float, y: float) -> tuple[int, int]:
    col = int((x - info.origin.position.x) / info.resolution)
    row = int((y - info.origin.position.y) / info.resolution)
    return row, col


def _build_free_mask(grid: np.ndarray) -> np.ndarray:
    # free: known and not occupied; unknown is treated as obstacle for robustness
    free = ((grid >= 0) & (grid < 50)).astype(np.uint8) * 255
    k = np.ones((3, 3), np.uint8)
    free = cv2.morphologyEx(free, cv2.MORPH_OPEN, k)
    free = cv2.morphologyEx(free, cv2.MORPH_CLOSE, k)
    return free


def _morphological_skeleton(binary: np.ndarray) -> np.ndarray:
    """Fallback skeletonization when ximgproc.thinning is unavailable."""
    img = (binary > 0).astype(np.uint8) * 255
    skel = np.zeros_like(img)
    element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
    while True:
        eroded = cv2.erode(img, element)
        opened = cv2.dilate(eroded, element)
        temp = cv2.subtract(img, opened)
        skel = cv2.bitwise_or(skel, temp)
        img = eroded
        if cv2.countNonZero(img) == 0:
            break
    return skel


def _distance_transform_m(free: np.ndarray, info) -> np.ndarray:
    dist_px = cv2.distanceTransform((free > 0).astype(np.uint8), cv2.DIST_L2, 5)
    return dist_px.astype(np.float32) * float(info.resolution)


def _corridor_mask_from_distance(
        free: np.ndarray,
        dist_m: np.ndarray,
        info,
        start_rc: tuple[int, int]) -> np.ndarray:
    h, w = free.shape
    sr, sc = start_rc
    if 0 <= sr < h and 0 <= sc < w and free[sr, sc] > 0:
        d_ref = float(dist_m[sr, sc])
    else:
        vals = dist_m[free > 0]
        d_ref = float(np.percentile(vals, 65)) if vals.size else 1.5

    # Keep a center-corridor band and avoid selecting outer free-space contours.
    low = max(0.6, 0.35 * d_ref)
    high = max(low + 0.4, 1.8 * d_ref)
    band = np.where((free > 0) & (dist_m >= low) & (dist_m <= high), 255, 0).astype(np.uint8)

    # Reconnect small gaps inside corridor band.
    k = np.ones((3, 3), np.uint8)
    band = cv2.morphologyEx(band, cv2.MORPH_CLOSE, k)

    # If corridor band collapses, fallback to a looser threshold.
    if int(cv2.countNonZero(band)) < 200:
        band = np.where((free > 0) & (dist_m >= low), 255, 0).astype(np.uint8)
    return band


def _skeleton_from_mask(mask: np.ndarray, dist_m: np.ndarray) -> np.ndarray:
    if cv2.countNonZero(mask) == 0:
        return np.zeros_like(mask)
    if hasattr(cv2, 'ximgproc') and hasattr(cv2.ximgproc, 'thinning'):
        skel = cv2.ximgproc.thinning(mask)
    else:
        skel = _morphological_skeleton(mask)

    # Remove fringe pixels that are too close to walls (in meters).
    min_clear = 0.5
    skel = np.where((skel > 0) & (dist_m >= min_clear), 255, 0).astype(np.uint8)
    return skel


def _skeleton_graph(skel: np.ndarray):
    rows, cols = np.where(skel > 0)
    nodes = {(int(r), int(c)) for r, c in zip(rows, cols)}
    nbrs: dict[tuple[int, int], list[tuple[int, int]]] = {}
    for r, c in nodes:
        ns = []
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                p = (r + dr, c + dc)
                if p in nodes:
                    ns.append(p)
        nbrs[(r, c)] = ns
    return nodes, nbrs


def _prune_spurs(nodes, nbrs, rounds: int = 200):
    nodes = set(nodes)
    nbrs = {k: list(v) for k, v in nbrs.items()}
    for _ in range(rounds):
        leaves = [p for p in nodes if len([q for q in nbrs[p] if q in nodes]) <= 1]
        if not leaves:
            break
        for p in leaves:
            nodes.discard(p)
    pruned = {}
    for p in nodes:
        pruned[p] = [q for q in nbrs[p] if q in nodes]
    return nodes, pruned


def _largest_component(nodes, nbrs):
    seen = set()
    best = set()
    for p in nodes:
        if p in seen:
            continue
        comp = set()
        dq = deque([p])
        seen.add(p)
        while dq:
            u = dq.popleft()
            comp.add(u)
            for v in nbrs[u]:
                if v not in seen:
                    seen.add(v)
                    dq.append(v)
        if len(comp) > len(best):
            best = comp
    sub = {p: [q for q in nbrs[p] if q in best] for p in best}
    return best, sub


def _all_components(nodes, nbrs) -> list[set]:
    """Return all connected components, largest first."""
    seen: set = set()
    comps: list[set] = []
    for p in nodes:
        if p in seen:
            continue
        comp: set = set()
        dq = deque([p])
        seen.add(p)
        while dq:
            u = dq.popleft()
            comp.add(u)
            for v in nbrs[u]:
                if v not in seen:
                    seen.add(v)
                    dq.append(v)
        comps.append(comp)
    comps.sort(key=len, reverse=True)
    return comps


def _best_component(
        nodes,
        nbrs,
        start_rc: tuple[int, int],
        start_search_px: float = 80.0,
) -> tuple[set, dict]:
    """Pick the best skeleton component to trace a cycle from.

    Strategy (first-principles):
    1. Collect all connected components, sorted by size (largest first).
    2. Among components that contain **at least one node within
       `start_search_px` pixels of start**, pick the **largest** one.
       This avoids small local loops that happen to be closest to the car.
    3. If no component satisfies the proximity criterion, fall back to the
       globally largest component — the car might be on a branch, but the
       main track loop is still there.
    """
    if not nodes:
        return set(), {}

    sr, sc = start_rc
    comps = _all_components(nodes, nbrs)

    # Step 2: largest component that touches the start neighbourhood.
    r2_thresh = start_search_px ** 2
    for comp in comps:
        for p in comp:
            dr = p[0] - sr
            dc = p[1] - sc
            if dr * dr + dc * dc <= r2_thresh:
                sub = {p: [q for q in nbrs[p] if q in comp] for p in comp}
                return comp, sub

    # Step 3: fallback to globally largest.
    best = comps[0]
    sub = {p: [q for q in nbrs[p] if q in best] for p in best}
    return best, sub


def _trace_cycle(component, nbrs, start_hint: tuple[int, int] | None = None):
    if not component:
        return []
    start: tuple[int, int] | None = None
    if start_hint is not None:
        best_d = float('inf')
        for p in component:
            if len(nbrs[p]) < 2:
                continue
            dr = p[0] - start_hint[0]
            dc = p[1] - start_hint[1]
            d = float(dr * dr + dc * dc)
            if d < best_d:
                best_d = d
                start = p
    if start is None:
        for p in component:
            if len(nbrs[p]) == 2:
                start = p
                break
    if start is None:
        start = next(iter(component))

    cycle = [start]
    prev = None
    cur = start
    closed = False
    visited_edges = set()
    max_steps = max(8, len(component) + 16)
    for _ in range(max_steps):
        neigh = [n for n in nbrs[cur] if n != prev]
        if not neigh:
            break
        if prev is None:
            nxt = neigh[0]
        elif len(neigh) == 1:
            nxt = neigh[0]
        else:
            # choose neighbor with smallest turn
            vx = cur[1] - prev[1]
            vy = cur[0] - prev[0]
            best = neigh[0]
            best_score = float('inf')
            for n in neigh:
                ux = n[1] - cur[1]
                uy = n[0] - cur[0]
                lv = math.hypot(vx, vy) or 1.0
                lu = math.hypot(ux, uy) or 1.0
                dot = max(-1.0, min(1.0, (vx * ux + vy * uy) / (lv * lu)))
                ang = math.acos(dot)
                if ang < best_score:
                    best_score = ang
                    best = n
            nxt = best
        edge = tuple(sorted((cur, nxt)))
        if edge in visited_edges and nxt != start:
            unvisited = [n for n in neigh if tuple(sorted((cur, n))) not in visited_edges]
            if unvisited:
                nxt = unvisited[0]
                edge = tuple(sorted((cur, nxt)))

        if nxt == start and len(cycle) >= 12:
            closed = True
            break
        cycle.append(nxt)
        visited_edges.add(edge)
        prev, cur = cur, nxt
    if not closed:
        return []
    return cycle


def _resample_closed_uniform(xs: np.ndarray, ys: np.ndarray, step: float) -> tuple[np.ndarray, np.ndarray]:
    if len(xs) < 3 or step <= 0.0:
        return xs, ys
    pts = np.column_stack([xs, ys])
    pts = np.vstack([pts, pts[0]])
    seg = np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1]))
    total = float(np.sum(seg))
    if total < 1e-6:
        return xs, ys
    n = max(8, int(total / max(step, 0.1)))
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    s = np.linspace(0.0, total, n, endpoint=False)
    rx = np.interp(s, cum, pts[:, 0])
    ry = np.interp(s, cum, pts[:, 1])
    return np.asarray(rx, dtype=float), np.asarray(ry, dtype=float)


def _rotate_and_orient(
        xs: np.ndarray,
        ys: np.ndarray,
        start_x: float,
        start_y: float,
        start_yaw: float) -> tuple[np.ndarray, np.ndarray]:
    if len(xs) < 3:
        return xs, ys
    d2 = (xs - start_x) ** 2 + (ys - start_y) ** 2
    k = int(np.argmin(d2))
    xs = np.roll(xs, -k)
    ys = np.roll(ys, -k)
    fwd_x, fwd_y = forward_vector(start_yaw)
    dx = float(xs[1] - xs[0])
    dy = float(ys[1] - ys[0])
    if dx * fwd_x + dy * fwd_y < 0.0:
        xs = xs[::-1]
        ys = ys[::-1]
    return xs, ys


def extract_loop_centerline_from_map(
        grid: np.ndarray,
        info,
        start_x: float,
        start_y: float,
        start_yaw: float,
        step: float = 2.0,
        close_dist: float = 4.0,
        min_points: int = 20,
        max_points: int = 400,
        post_smooth_passes: int = 0,
        refine_passes: int = 3) -> tuple[np.ndarray, np.ndarray]:
    """Extract loop centerline by EDT + corridor-constrained skeleton cycle."""
    free = _build_free_mask(grid)
    dist_m = _distance_transform_m(free, info)
    start_rc = _world_to_grid(info, start_x, start_y)
    corridor = _corridor_mask_from_distance(free, dist_m, info, start_rc)
    skel = _skeleton_from_mask(corridor, dist_m)
    if cv2.countNonZero(skel) < 32:
        # fallback: try skeleton on full free mask when corridor mask is too sparse
        skel = _skeleton_from_mask(free, dist_m)

    nodes, nbrs = _skeleton_graph(skel)
    if not nodes:
        return np.array([]), np.array([])

    nodes, nbrs = _prune_spurs(nodes, nbrs)
    if not nodes:
        return np.array([]), np.array([])

    # Search radius: large enough to always find the main track loop even if
    # the car is far from the corridor skeleton (e.g. near a corner).
    search_px = max(80.0, 150.0 / float(info.resolution))
    comp, comp_nbrs = _best_component(nodes, nbrs, start_rc, start_search_px=search_px)
    cycle = _trace_cycle(comp, comp_nbrs, start_hint=start_rc)
    if len(cycle) < max(min_points, 8):
        return np.array([]), np.array([])

    xs, ys = [], []
    for r, c in cycle:
        x, y = _grid_to_world(info, r, c)
        xs.append(x)
        ys.append(y)
    arr_x = np.asarray(xs, dtype=float)
    arr_y = np.asarray(ys, dtype=float)

    arr_x, arr_y = _resample_closed_uniform(arr_x, arr_y, max(0.5, step))
    arr_x, arr_y = _rotate_and_orient(arr_x, arr_y, start_x, start_y, start_yaw)

    if len(arr_x) > max_points:
        idx = np.linspace(0, len(arr_x) - 1, max_points, dtype=int)
        arr_x = arr_x[idx]
        arr_y = arr_y[idx]

    if len(arr_x) >= 3 and refine_passes > 0:
        arr_x, arr_y = refine_closed_centerline_from_map(
            grid, info, arr_x, arr_y, passes=refine_passes)
    if post_smooth_passes > 0 and len(arr_x) >= 3:
        arr_x, arr_y = _moving_average_closed(arr_x, arr_y, post_smooth_passes)

    # closure sanity: keep only plausible loops
    if len(arr_x) >= 2:
        closure = float(np.hypot(arr_x[0] - arr_x[-1], arr_y[0] - arr_y[-1]))
        if closure > max(close_dist * 2.0, 6.0):
            return np.array([]), np.array([])
    return arr_x, arr_y
