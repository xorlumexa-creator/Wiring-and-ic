"""
export_scene.py — combines the CAD shell, placed components, mount-area
decals, and routed wires into one GLB the viewer can load.

Node naming carries the metadata the viewer needs (category, instance id,
wire type) as plain strings in the mesh/node name, e.g.
"component::fc_main::flight_controller" or "wire::fc_main.esc_signal_1->
esc1.signal_in_1::signal" — trimesh reliably preserves geometry names as
glTF node names on export, which is a simpler and more robust contract
than depending on custom glTF 'extras' round-tripping through trimesh's
exporter. static/viewer.html parses these names back out client-side.

MOUNT-AREA DECALS
------------------
Beyond the floating 3D box for each component, build_mount_area_patch
projects that component's rough footprint onto the nearest point of the
actual CAD shell surface, colored to match the component — a flat colored
"sticker" that answers "where do I physically attach this part," which a
box floating in open space doesn't. The projection: find the point on the
shell nearest the component's center, build a tangent plane there from
the surface normal, lay out a square grid sized to the component's
footprint on that plane, then snap every grid point back onto the true
(possibly curved) surface. It assumes the STL has consistent outward-
facing winding (true for anything exported the way the existing Lumexa
CAD pipeline does it) and approximates a square footprint sized to the
larger of the component's two in-plane dimensions — for narrow/oblong
parts, that's a deliberately generous decal rather than a tight one.
"""
from __future__ import annotations

import io
import numpy as np
import trimesh

from component_library import CATEGORY_COLOR_RGBA, WIRE_CATEGORIES, COMPONENT_LIBRARY
from routing import waypoints_to_tube_mesh

XRAY_SHELL_ALPHA = 60      # 0-255; low so components/wires/decals read clearly through it
WIRE_RADIUS_MM = 0.9
MOUNT_AREA_ALPHA = 170     # more opaque than the shell — it's meant to stand out
MOUNT_AREA_GRID_N = 6
MOUNT_AREA_SURFACE_OFFSET_MM = 0.3   # avoid z-fighting with the shell mesh itself


def _orthonormal_basis(normal: np.ndarray):
    normal = normal / (np.linalg.norm(normal) + 1e-12)
    ref = np.array([1.0, 0.0, 0.0]) if abs(normal[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    t1 = np.cross(normal, ref)
    t1 /= (np.linalg.norm(t1) + 1e-12)
    t2 = np.cross(normal, t1)
    return t1, t2


def build_mount_area_patch(mesh: trimesh.Trimesh, rec: dict, grid_n: int = MOUNT_AREA_GRID_N):
    center = np.asarray(rec["center_mm"], dtype=float).reshape(1, 3)
    anchor_pts, _, tri_ids = trimesh.proximity.closest_point(mesh, center)
    anchor = anchor_pts[0]
    normal = mesh.face_normals[tri_ids[0]]
    t1, t2 = _orthonormal_basis(normal)

    half = max(rec["dims_mm"][0], rec["dims_mm"][1]) / 2.0
    us = np.linspace(-half, half, grid_n)
    vs = np.linspace(-half, half, grid_n)
    plane_points = np.array([anchor + u * t1 + v * t2 for u in us for v in vs])
    snapped_pts, _, _ = trimesh.proximity.closest_point(mesh, plane_points)
    snapped_pts = snapped_pts + normal * MOUNT_AREA_SURFACE_OFFSET_MM

    def vid(i, j):
        return i * grid_n + j

    faces = []
    for i in range(grid_n - 1):
        for j in range(grid_n - 1):
            a, b, c, d = vid(i, j), vid(i + 1, j), vid(i + 1, j + 1), vid(i, j + 1)
            faces.append([a, b, c])
            faces.append([a, c, d])

    patch = trimesh.Trimesh(vertices=snapped_pts, faces=np.array(faces), process=False)
    color = list(rec["color_rgba"][:3]) + [MOUNT_AREA_ALPHA]
    patch.visual.vertex_colors = np.tile(color, (len(patch.vertices), 1))
    return patch


def build_scene_glb(mesh: trimesh.Trimesh, design_state) -> bytes:
    scene = trimesh.Scene()

    shell = mesh.copy()
    shell.visual.vertex_colors = np.tile([200, 210, 220, XRAY_SHELL_ALPHA], (len(shell.vertices), 1))
    scene.add_geometry(shell, node_name="cad_shell::xray")

    for instance_id, rec in design_state.placed.items():
        try:
            box = trimesh.creation.box(extents=rec["dims_mm"])
            R = trimesh.transformations.euler_matrix(*np.radians(rec["rotation_deg"]))
            box.apply_transform(R)
            box.apply_translation(rec["center_mm"])
            color = rec.get("color_rgba") or CATEGORY_COLOR_RGBA.get(rec["category"], [150, 150, 150, 255])
            box.visual.vertex_colors = np.tile(color, (len(box.vertices), 1))
            scene.add_geometry(box, node_name=f"component::{instance_id}::{rec['category']}")
        except Exception as e:  # noqa: BLE001 — one bad component shouldn't sink the whole scene
            print(f"[export_scene] component box skipped for '{instance_id}': {e}")

        try:
            patch = build_mount_area_patch(mesh, rec)
            scene.add_geometry(patch, node_name=f"mountarea::{instance_id}::{rec['category']}")
        except Exception as e:  # noqa: BLE001 — a decal failure shouldn't sink the whole export
            print(f"[export_scene] mount-area decal skipped for '{instance_id}': {e}")

    for key, conn in design_state.connections.items():
        if conn["status"] != "routed" or len(conn["waypoints_mm"]) < 2:
            continue
        try:
            color = WIRE_CATEGORIES[conn["wire_category"]]["color_rgba"]
            tube = waypoints_to_tube_mesh(conn["waypoints_mm"], WIRE_RADIUS_MM, color)
            safe_key = key.replace(" ", "")
            scene.add_geometry(tube, node_name=f"wire::{safe_key}::{conn['wire_category']}")
        except Exception as e:  # noqa: BLE001 — one bad wire shouldn't sink the whole scene
            print(f"[export_scene] wire tube skipped for '{key}': {e}")

    buf = io.BytesIO()
    buf.write(scene.export(file_type="glb"))
    return buf.getvalue()


def build_design_manifest(design_state) -> dict:
    """Plain-JSON companion to the GLB — the viewer's sidebar and the API
    response both use this rather than re-deriving it from mesh names."""
    return {
        "device_type": design_state.device_type,
        "finalized": design_state.finalized,
        "final_summary": design_state.final_summary,
        "confidence_notes": design_state.confidence_notes,
        "total_mass_g": design_state.total_mass_g(),
        "wire_color_legend": {cat: v["color_rgba"] for cat, v in WIRE_CATEGORIES.items()},
        "components": [
            {"instance_id": iid, "label": r["label"], "color_rgba": r["color_rgba"],
             "component_id": r["component_id"], "category": r["category"],
             "name": COMPONENT_LIBRARY[r["component_id"]]["name"],
             "center_mm": r["center_mm"], "rotation_deg": r["rotation_deg"], "mass_g": r["mass_g"]}
            for iid, r in design_state.placed.items()
        ],
        "connections": [
            {"key": k, "label": c["label"], "from": f"{c['from_instance']}.{c['from_connector']}",
             "to": f"{c['to_instance']}.{c['to_connector']}", "wire_category": c["wire_category"],
             "color_rgba": WIRE_CATEGORIES[c["wire_category"]]["color_rgba"],
             "status": c["status"], "length_mm": c["length_mm"], "reason": c["reason"]}
            for k, c in design_state.connections.items()
        ],
    }
