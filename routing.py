"""
routing.py — 3D collision-aware wire routing.

There's no mature off-the-shelf Python library for "route a wire through
an arbitrary mesh's free space, avoiding obstacles" — the published
approaches (see e.g. RRT-based cable routing literature) all boil down to
graph search or sampling-based planning over a collision-checked space.
Given the grid we already build in spatial.py, plain A* over 26-connected
voxel neighbors is the right tool: deterministic, reproducible, and exact
for the resolution chosen (no randomness like RRT, which matters if you
want two runs on the same design to produce the same wiring).

This is what makes wire routing "accurate" here: every returned path is a
sequence of voxel centers that the solver has actually verified sit in
free space (grid.routing_mask), not a straight line assumed to be clear.
"""
from __future__ import annotations

import heapq
import numpy as np
import trimesh

from spatial import CavityGrid

# 26-connected neighbor offsets, precomputed with their Euclidean step cost
# (so diagonal moves aren't underpriced relative to axis moves — this keeps
# the returned path length a realistic wire-length estimate, not just a
# voxel count).
_NEIGHBORS = []
for dx in (-1, 0, 1):
    for dy in (-1, 0, 1):
        for dz in (-1, 0, 1):
            if dx == dy == dz == 0:
                continue
            _NEIGHBORS.append((dx, dy, dz, (dx * dx + dy * dy + dz * dz) ** 0.5))


def route_wire(grid: CavityGrid, start_mm, goal_mm, max_expansions: int = 80_000) -> dict:
    """A* from start_mm to goal_mm through grid.routing_mask.
    Returns {"status": "routed"|"unreachable", "waypoints_mm": [[x,y,z],...],
    "length_mm": float, "reason": str|None}.

    max_expansions default lowered from an earlier 300,000: this is a pure-
    Python heapq loop (no numpy vectorization inside the search itself,
    since each expansion depends on the last), and Render's free tier is
    0.1 vCPU — a large expansion count here is real wall-clock risk on
    route_all_wires calling this once per connection in a design. The
    trade: on a genuinely fragmented cavity, a route that would eventually
    be found at 200,000 expansions now reports unreachable at 80,000
    instead — worth it for bounded latency. Raise this only after
    confirming on a real deployment that requests aren't timing out."""
    start_idx = grid.nearest_routable_index(start_mm)
    goal_idx = grid.nearest_routable_index(goal_mm)
    if start_idx is None:
        return {"status": "unreachable", "waypoints_mm": [], "length_mm": 0.0,
                "reason": f"Start point {list(np.round(start_mm, 1))}mm has no free space "
                          f"nearby — the connector may be buried inside solid material."}
    if goal_idx is None:
        return {"status": "unreachable", "waypoints_mm": [], "length_mm": 0.0,
                "reason": f"Goal point {list(np.round(goal_mm, 1))}mm has no free space "
                          f"nearby — the connector may be buried inside solid material."}

    def h(idx):
        return grid.pitch * (
            (idx[0] - goal_idx[0]) ** 2 + (idx[1] - goal_idx[1]) ** 2 + (idx[2] - goal_idx[2]) ** 2
        ) ** 0.5

    open_heap = [(h(start_idx), 0.0, start_idx)]
    came_from = {}
    g_score = {start_idx: 0.0}
    visited = set()
    expansions = 0

    while open_heap:
        expansions += 1
        if expansions > max_expansions:
            return {"status": "unreachable", "waypoints_mm": [], "length_mm": 0.0,
                    "reason": f"A* gave up after {max_expansions} node expansions without "
                              f"finding a path — the cavity may be too fragmented at this "
                              f"grid resolution ({grid.pitch}mm)."}
        _, g, current = heapq.heappop(open_heap)
        if current in visited:
            continue
        visited.add(current)
        if current == goal_idx:
            path_idx = [current]
            while current in came_from:
                current = came_from[current]
                path_idx.append(current)
            path_idx.reverse()
            waypoints = [grid.index_to_world(idx).tolist() for idx in path_idx]
            length_mm = sum(
                float(np.linalg.norm(np.array(waypoints[i]) - np.array(waypoints[i - 1])))
                for i in range(1, len(waypoints))
            )
            return {"status": "routed", "waypoints_mm": waypoints,
                    "length_mm": round(length_mm, 1), "reason": None}

        for dx, dy, dz, step_cost in _NEIGHBORS:
            nxt = (current[0] + dx, current[1] + dy, current[2] + dz)
            if not grid.in_bounds(nxt) or not grid.routing_mask[nxt] or nxt in visited:
                continue
            tentative_g = g + step_cost * grid.pitch
            if tentative_g < g_score.get(nxt, float("inf")):
                g_score[nxt] = tentative_g
                came_from[nxt] = current
                heapq.heappush(open_heap, (tentative_g + h(nxt), tentative_g, nxt))

    return {"status": "unreachable", "waypoints_mm": [], "length_mm": 0.0,
            "reason": "No path exists between these two points through the routable cavity "
                      "at the current grid resolution — they may be in disconnected pockets."}


def waypoints_to_tube_mesh(waypoints_mm: list, radius_mm: float, color_rgba) -> trimesh.Trimesh:
    """Builds a visualization-only tube (a chain of cylinders — not a
    boolean-unioned manifold solid, which isn't needed since this mesh is
    for the X-ray viewer, not for fabrication) along the routed path."""
    segments = []
    for i in range(1, len(waypoints_mm)):
        p0, p1 = np.array(waypoints_mm[i - 1]), np.array(waypoints_mm[i])
        if np.linalg.norm(p1 - p0) < 1e-6:
            continue
        cyl = trimesh.creation.cylinder(radius=radius_mm, segment=[p0, p1], sections=8)
        segments.append(cyl)
    if not segments:
        # Degenerate (start == goal) — a tiny sphere marker instead of nothing.
        tube = trimesh.creation.icosphere(radius=radius_mm, subdivisions=1)
        tube.apply_translation(waypoints_mm[0])
    else:
        tube = trimesh.util.concatenate(segments)
    tube.visual.vertex_colors = np.tile(color_rgba, (len(tube.vertices), 1))
    return tube
