"""
spatial.py — turns the incoming CAD mesh into a voxel grid the placement
and routing tools can query cheaply, and does the actual collision math.

APPROACH
--------
1. Load the mesh (STL from the existing Lumexa CAD system).
2. Compute a signed distance field on a regular grid inside the mesh's
   bounding box: trimesh.proximity.signed_distance gives, for any point,
   +distance-to-surface if the point is inside the mesh volume and
   -distance-to-surface if outside. That single call gives us everything:
     - "is this point usable at all" (must be inside: sdist > 0)
     - "how much clearance from the skin does it have" (sdist magnitude)
   which is exactly what both component placement (needs real 3D room,
   larger clearance) and wire routing (can hug closer to the wall, smaller
   clearance) need, without maintaining two different data structures.
3. Two boolean masks are derived from the same distance field:
     placement_mask[i] = sdist[i] > component_clearance_mm
     routing_mask[i]   = sdist[i] > wire_clearance_mm
   routing_mask is a superset of placement_mask (wires can go where
   components can't fit) — deliberately, since wires are thin.

RELIABILITY NOTES (read before changing constants)
----------------------------------------------------
- MAX_VOXELS is tuned against Render free tier's actual CPU allocation —
  0.1 vCPU, confirmed against Render's current published free-tier specs
  — not a memory limit (a bool array this size is a few hundred KB either
  way). 0.1 vCPU is a small fraction of a core, so this is deliberately
  conservative; raise it only if you've confirmed on a real deployment
  that requests aren't timing out.
- signed_distance is only well-defined for a closed (watertight) mesh.
  load_mesh attempts a repair, but if the mesh is still non-watertight
  afterward, results here (and everything downstream) may be silently
  wrong rather than erroring — CavityGrid.mesh_watertight tells you
  whether to trust the numbers, and main.py surfaces it in the API
  response rather than letting it fail silently.
- Every bounds check in this file goes through is_within_grid_bounds,
  which tests the RAW point against the grid's real spatial extent.
  world_to_index CLAMPS its output into valid array-index range (so you
  can always use it to index into a numpy array without an IndexError) —
  which means checking in_bounds() on an already-clamped index is always
  true and tells you nothing. That bug existed in an earlier version of
  this file (a point genuinely outside the mesh's bounding box would
  silently get checked against whatever mask value sits at the nearest
  boundary voxel instead of being correctly rejected) — fixed by keeping
  the "is this real" check and the "how do I index it" step separate.
"""
from __future__ import annotations

import numpy as np
import trimesh

MAX_VOXELS = 120_000           # compute-time cap, tuned for Render free tier's 0.1 vCPU (see above)
MIN_PITCH_MM = 1.0             # never go finer than 1mm — diminishing returns
DEFAULT_COMPONENT_CLEARANCE_MM = 3.0   # min distance from the CAD skin to place a box
DEFAULT_WIRE_CLEARANCE_MM = 1.0        # min distance from the CAD skin to route a wire


def load_mesh(path: str) -> tuple:
    """Returns (mesh, was_watertight_without_repair, is_watertight_now).
    Callers should treat is_watertight_now == False as a real warning, not
    just log noise — signed_distance's results are not reliable otherwise."""
    mesh = trimesh.load_mesh(path)
    if isinstance(mesh, trimesh.Scene):
        # Some STL/OBJ exports come back as a Scene with one geometry —
        # flatten it, same defensive pattern the CAD repo uses.
        mesh = trimesh.util.concatenate(list(mesh.geometry.values()))
    was_watertight = bool(mesh.is_watertight)
    if not was_watertight:
        mesh.fill_holes()
    return mesh, was_watertight, bool(mesh.is_watertight)


def choose_pitch_mm(mesh: trimesh.Trimesh) -> float:
    bbox_volume = float(np.prod(mesh.extents))
    if bbox_volume <= 0:
        return MIN_PITCH_MM
    ideal = (bbox_volume / MAX_VOXELS) ** (1.0 / 3.0)
    return max(MIN_PITCH_MM, round(ideal, 2))


class CavityGrid:
    """Precomputed free-space grid for one CAD shell. Build once per design
    run, then reuse for every placement/routing tool call in that run."""

    def __init__(self, mesh: trimesh.Trimesh, mesh_watertight: bool = True,
                 component_clearance_mm: float = DEFAULT_COMPONENT_CLEARANCE_MM,
                 wire_clearance_mm: float = DEFAULT_WIRE_CLEARANCE_MM):
        self.mesh = mesh
        self.mesh_watertight = mesh_watertight
        self.pitch = choose_pitch_mm(mesh)
        self.origin = mesh.bounds[0]
        extents = mesh.bounds[1] - mesh.bounds[0]
        self.shape = np.maximum(1, np.ceil(extents / self.pitch).astype(int))
        self.extent_mm = np.array(self.shape) * self.pitch   # real spatial size of the grid volume

        xs = self.origin[0] + (np.arange(self.shape[0]) + 0.5) * self.pitch
        ys = self.origin[1] + (np.arange(self.shape[1]) + 0.5) * self.pitch
        zs = self.origin[2] + (np.arange(self.shape[2]) + 0.5) * self.pitch
        gx, gy, gz = np.meshgrid(xs, ys, zs, indexing="ij")
        points = np.stack([gx.ravel(), gy.ravel(), gz.ravel()], axis=1)

        sdist = trimesh.proximity.signed_distance(mesh, points)
        self.sdist = sdist.reshape(self.shape)

        self.placement_mask = self.sdist > component_clearance_mm
        self.routing_mask = self.sdist > wire_clearance_mm

        self.usable_volume_mm3 = float(self.placement_mask.sum() * self.pitch ** 3)
        self.total_bbox_volume_mm3 = float(np.prod(extents))

    # -- coordinate helpers ------------------------------------------------
    def is_within_grid_bounds(self, points_mm) -> np.ndarray:
        """Vectorized: True where the RAW (unclamped) point actually falls
        inside the grid's spatial extent. This is the check that matters
        for "is this really outside the shell" — see the module docstring
        for why world_to_index + in_bounds is NOT equivalent to this."""
        points_mm = np.atleast_2d(points_mm)
        rel = points_mm - self.origin
        return np.all((rel >= 0) & (rel < self.extent_mm), axis=1)

    def world_to_index_array(self, points_mm) -> np.ndarray:
        """Vectorized index computation, CLAMPED to valid array range —
        only safe to use for array indexing after checking
        is_within_grid_bounds separately if "was this point real" matters."""
        points_mm = np.atleast_2d(points_mm)
        idx = np.floor((points_mm - self.origin) / self.pitch).astype(int)
        return np.clip(idx, 0, np.array(self.shape) - 1)

    def world_to_index(self, point_mm) -> tuple:
        idx = self.world_to_index_array(point_mm)[0]
        return tuple(int(v) for v in idx)

    def index_to_world(self, idx) -> np.ndarray:
        return self.origin + (np.asarray(idx) + 0.5) * self.pitch

    def in_bounds(self, idx) -> bool:
        return all(0 <= idx[d] < self.shape[d] for d in range(3))

    def nearest_routable_index(self, point_mm, max_search_voxels: int = 8):
        """A connector point sitting exactly on a component's surface can
        land just outside routing_mask due to grid discretization — search
        a small expanding cube around it for the nearest True voxel.
        Returns None both when the point is genuinely outside the grid's
        spatial extent and when nothing routable is found nearby — either
        way there's nowhere to route from/to."""
        if not self.is_within_grid_bounds(point_mm)[0]:
            return None
        center = self.world_to_index(point_mm)
        if self.routing_mask[center]:
            return center
        for r in range(1, max_search_voxels + 1):
            best, best_d2 = None, None
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    for dz in range(-r, r + 1):
                        if max(abs(dx), abs(dy), abs(dz)) != r:
                            continue  # only the new shell at this radius
                        idx = (center[0] + dx, center[1] + dy, center[2] + dz)
                        if self.in_bounds(idx) and self.routing_mask[idx]:
                            d2 = dx * dx + dy * dy + dz * dz
                            if best_d2 is None or d2 < best_d2:
                                best, best_d2 = idx, d2
            if best is not None:
                return best
        return None


def _box_check_points(lo: np.ndarray, hi: np.ndarray, pitch: float) -> np.ndarray:
    """Sample points for collision-checking a box: densely on its 6
    surfaces (at grid resolution — a corner poking past the shell will
    always show up on a face), plus a coarser interior grid (2x pitch) as
    a defense-in-depth check against a thin obstruction that runs through
    the box's core without touching its surface. This is an intentional
    trade against the earlier version's full dense-volume sampling: for a
    modest component that was thousands of points checked one at a time
    in a Python loop; on Render's 0.1 vCPU that was a real risk of a
    single placement call eating multiple seconds. Surface + coarse
    interior is a small fraction of the points, vectorized, and only
    misses a genuinely rare geometry (a thin internal obstruction fully
    enclosed by the box's surface, touching none of it, finer than the
    coarse interior grid) — worth it for the speedup.
    """
    n = np.maximum(2, np.ceil((hi - lo) / pitch).astype(int) + 1)
    xs, ys, zs = (np.linspace(lo[d], hi[d], n[d]) for d in range(3))

    faces = []
    gy, gz = np.meshgrid(ys, zs, indexing="ij")
    for x in (lo[0], hi[0]):
        faces.append(np.stack([np.full(gy.size, x), gy.ravel(), gz.ravel()], axis=1))
    gx, gz = np.meshgrid(xs, zs, indexing="ij")
    for y in (lo[1], hi[1]):
        faces.append(np.stack([gx.ravel(), np.full(gx.size, y), gz.ravel()], axis=1))
    gx, gy = np.meshgrid(xs, ys, indexing="ij")
    for z in (lo[2], hi[2]):
        faces.append(np.stack([gx.ravel(), gy.ravel(), np.full(gx.size, z)], axis=1))
    surface_points = np.concatenate(faces, axis=0)

    n_coarse = np.maximum(1, np.ceil((hi - lo) / (2 * pitch)).astype(int) + 1)
    cxs, cys, czs = (np.linspace(lo[d], hi[d], n_coarse[d]) for d in range(3))
    cgx, cgy, cgz = np.meshgrid(cxs, cys, czs, indexing="ij")
    interior_points = np.stack([cgx.ravel(), cgy.ravel(), cgz.ravel()], axis=1)

    return np.concatenate([surface_points, interior_points], axis=0)


def check_box_fits(grid: CavityGrid, center_mm, half_extents_mm, occupied_boxes: list) -> dict:
    """
    Deterministic collision check for placing an axis-aligned box (the
    agent's placement is treated as AABB even if it reports an orientation
    — see agent_tools.py docstring for why that's an intentional
    simplification). Checks two things:
      1. every sampled point of the box (surface + coarse interior — see
         _box_check_points) lies in placement_mask (i.e. is far enough
         inside the CAD shell) AND within the grid's real spatial extent
      2. the box's AABB doesn't overlap any already-placed component's AABB
    Returns {"fits": bool, "reason": str|None}.
    """
    center_mm = np.asarray(center_mm, dtype=float)
    half = np.asarray(half_extents_mm, dtype=float)
    lo, hi = center_mm - half, center_mm + half

    sample_points = _box_check_points(lo, hi, grid.pitch)
    within = grid.is_within_grid_bounds(sample_points)
    if not np.all(within):
        bad_point = sample_points[~within][0]
        return {"fits": False,
                "reason": f"Point {bad_point.round(1).tolist()}mm on the component's bounding box "
                          f"falls outside the CAD shell's bounding box entirely — this placement "
                          f"is (partly) outside the model. Move it further inside."}

    idx = grid.world_to_index_array(sample_points)
    mask_values = grid.placement_mask[idx[:, 0], idx[:, 1], idx[:, 2]]
    if not np.all(mask_values):
        bad_point = sample_points[~mask_values][0]
        return {"fits": False,
                "reason": f"Point {bad_point.round(1).tolist()}mm on the component's bounding box "
                          f"is outside the usable cavity (too close to or outside the CAD "
                          f"shell). Try moving it further from the surface or into a larger "
                          f"cavity region."}

    # 2. AABB-vs-AABB overlap against everything already placed.
    for other in occupied_boxes:
        o_lo = np.asarray(other["center_mm"]) - np.asarray(other["half_extents_mm"])
        o_hi = np.asarray(other["center_mm"]) + np.asarray(other["half_extents_mm"])
        overlap = np.all(lo < o_hi) and np.all(hi > o_lo)
        if overlap:
            return {"fits": False,
                    "reason": f"Overlaps already-placed component '{other['instance_id']}'. "
                              f"Move one of the two."}

    return {"fits": True, "reason": None}
