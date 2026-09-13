"""
agent_tools.py — the tool-calling surface the AI agent operates through.

Same philosophy as the existing Lumexa CAD repo's engineering_agent: the
model proposes, these functions independently verify. The model can never
just assert "this fits" or "this wire is 40mm long" — every placement runs
a real collision check against the voxel grid (spatial.check_box_fits) and
every wire length comes from an actual A* search (routing.route_wire).
That's where the "high accuracy" the user asked for actually comes from —
not from prompting the model to be careful, but from not trusting it to
grade its own work.

ORIENTATION SIMPLIFICATION (read before changing rotation handling)
---------------------------------------------------------------------
Components are placed with a center + Euler rotation, but collisions are
checked against the rotated box's axis-aligned bounding box (AABB), not
its true oriented bounding box (OBB). This is conservative — it can
reject a rotated placement that would truly fit under an exact OBB-OBB
test — but it can never *accept* a placement that actually collides,
which is the safe direction to be wrong in for a first version. Swapping
in true OBB-OBB overlap (e.g. via the separating axis theorem) later is a
contained change: only check_box_fits' second loop needs it.
"""
from __future__ import annotations

import numpy as np
import trimesh

from component_library import (
    COMPONENT_LIBRARY, INSTANCE_COLOR_PALETTE, NETLIST_TEMPLATES, WIRE_CATEGORIES,
    get_component, list_components,
)
from spatial import CavityGrid, check_box_fits
from routing import route_wire


def label_for_index(n: int) -> str:
    """Spreadsheet-column-style label: 0->A, 1->B, ..., 25->Z, 26->AA, 27->AB, ...
    Used so every placed component and every wire gets a short, stable,
    human-referenceable tag (matches what the 3D viewer displays)."""
    label = ""
    n += 1
    while n > 0:
        n, rem = divmod(n - 1, 26)
        label = chr(ord("A") + rem) + label
    return label


class DesignState:
    def __init__(self, grid: CavityGrid, device_type: str):
        self.grid = grid
        self.device_type = device_type
        self.placed: dict[str, dict] = {}       # instance_id -> record
        self.connections: dict[str, dict] = {}  # key -> record
        self.finalized = False
        self.final_summary = None
        self.confidence_notes = None

    # -- internal helpers ---------------------------------------------------
    def _occupied_boxes(self, exclude_instance: str | None = None) -> list[dict]:
        return [{"instance_id": iid, "center_mm": rec["center_mm"],
                  "half_extents_mm": rec["half_extents_mm"]}
                for iid, rec in self.placed.items() if iid != exclude_instance]

    def total_mass_g(self) -> float:
        return round(sum(rec["mass_g"] for rec in self.placed.values()), 1)


# =========================================================================
# Tool implementations — each returns a small JSON-safe dict.
# =========================================================================

def tool_list_component_library(state: DesignState, category: str | None = None) -> dict:
    return {"status": "ok", "components": list_components(category)}


def tool_get_cavity_summary(state: DesignState) -> dict:
    g = state.grid
    return {
        "status": "ok",
        "bbox_extents_mm": (g.mesh.bounds[1] - g.mesh.bounds[0]).round(1).tolist(),
        "grid_pitch_mm": g.pitch,
        "usable_volume_mm3": round(g.usable_volume_mm3, 0),
        "usable_fraction_of_bbox": round(g.usable_volume_mm3 / max(g.total_bbox_volume_mm3, 1e-6), 3),
        "components_placed_so_far": len(state.placed),
        "mesh_watertight": g.mesh_watertight,
        "note": ("usable_volume assumes the incoming CAD mesh's units are millimeters "
                 "(CadQuery's default) — if the source system exports in a different "
                 "unit, rescale before uploading."
                 + ("" if g.mesh_watertight else
                    " WARNING: this mesh was not (fully) watertight — every number above may "
                    "be inaccurate; treat placement/routing results with real skepticism.")),
    }


def tool_add_component(state: DesignState, component_id: str, instance_id: str,
                        center_mm: list, rotation_deg: list | None = None) -> dict:
    if instance_id in state.placed:
        return {"status": "rejected", "reason": f"instance_id '{instance_id}' already placed. "
                                                 f"Use remove_component first to reposition it."}
    try:
        comp = get_component(component_id)
    except KeyError as e:
        return {"status": "rejected", "reason": str(e)}

    rotation_deg = rotation_deg or [0.0, 0.0, 0.0]
    center = np.asarray(center_mm, dtype=float)
    R = trimesh.transformations.euler_matrix(*np.radians(rotation_deg))[:3, :3]

    dims = np.asarray(comp["dims_mm"], dtype=float)
    padded_extents = dims + 2.0 * comp["clearance_mm"]
    box = trimesh.creation.box(extents=padded_extents)
    box.apply_transform(np.vstack([np.hstack([R, np.zeros((3, 1))]), [0, 0, 0, 1]]))
    box.apply_translation(center)
    lo, hi = box.bounds
    half_extents = (hi - lo) / 2.0

    check = check_box_fits(state.grid, center, half_extents, state._occupied_boxes())
    if not check["fits"]:
        return {"status": "rejected", "reason": check["reason"]}

    connectors_world = {}
    for c in comp["connectors"]:
        local = np.asarray(c["local_pos_mm"], dtype=float)
        connectors_world[c["id"]] = (R @ local + center).round(2).tolist()

    # Each placed instance gets a stable short label (A, B, C, ...) and its
    # own distinct color (not shared by category) — this is what lets the
    # 3D viewer and the legend show "area B is the GPS, put it here" even
    # when two components share a category.
    label = label_for_index(len(state.placed))
    color_rgba = INSTANCE_COLOR_PALETTE[len(state.placed) % len(INSTANCE_COLOR_PALETTE)]

    state.placed[instance_id] = {
        "instance_id": instance_id, "component_id": component_id, "category": comp["category"],
        "mass_g": comp["mass_g"], "center_mm": center.tolist(), "rotation_deg": rotation_deg,
        "half_extents_mm": half_extents.tolist(), "dims_mm": comp["dims_mm"],
        "connectors_world": connectors_world,
        "requires_sky_view": comp["requires_sky_view"],
        "label": label, "color_rgba": color_rgba,
    }
    return {"status": "placed", "instance_id": instance_id, "label": label,
            "connectors_world": connectors_world, "running_total_mass_g": state.total_mass_g(),
            "sky_view_advisory": (f"'{instance_id}' (area {label}) is a {comp['category']} that "
                                   f"typically needs exterior/sky-facing placement for signal — "
                                   f"double check its position is near the airframe's top or an "
                                   f"open face." if comp["requires_sky_view"] else None)}


def tool_remove_component(state: DesignState, instance_id: str) -> dict:
    if instance_id not in state.placed:
        return {"status": "rejected", "reason": f"No placed component with instance_id '{instance_id}'."}
    del state.placed[instance_id]
    dropped = [k for k, c in state.connections.items()
               if c["from_instance"] == instance_id or c["to_instance"] == instance_id]
    for k in dropped:
        del state.connections[k]
    return {"status": "removed", "instance_id": instance_id,
            "connections_dropped": dropped}


def tool_add_connection(state: DesignState, from_instance: str, from_connector: str,
                         to_instance: str, to_connector: str, wire_category: str) -> dict:
    if wire_category not in WIRE_CATEGORIES:
        return {"status": "rejected", "reason": f"wire_category must be one of {list(WIRE_CATEGORIES)}"}
    for iid, conn in ((from_instance, from_connector), (to_instance, to_connector)):
        if iid not in state.placed:
            return {"status": "rejected", "reason": f"'{iid}' has not been placed yet."}
        if conn not in state.placed[iid]["connectors_world"]:
            valid = list(state.placed[iid]["connectors_world"])
            return {"status": "rejected",
                    "reason": f"'{iid}' has no connector '{conn}'. Valid connectors: {valid}"}
    key = f"{from_instance}.{from_connector}->{to_instance}.{to_connector}"
    if key in state.connections:
        return {"status": "rejected", "reason": f"Connection '{key}' already exists."}
    label = f"W{len(state.connections) + 1}"
    state.connections[key] = {
        "key": key, "label": label, "from_instance": from_instance, "from_connector": from_connector,
        "to_instance": to_instance, "to_connector": to_connector, "wire_category": wire_category,
        "status": "pending", "waypoints_mm": [], "length_mm": 0.0, "reason": None,
    }
    return {"status": "added", "key": key, "label": label}


def tool_remove_connection(state: DesignState, key: str) -> dict:
    if key not in state.connections:
        return {"status": "rejected", "reason": f"No connection with key '{key}'."}
    del state.connections[key]
    return {"status": "removed", "key": key}


def tool_auto_populate_standard_netlist(state: DesignState, device_type: str | None = None) -> dict:
    device_type = device_type or state.device_type
    template = NETLIST_TEMPLATES.get(device_type)
    if template is None:
        return {"status": "rejected",
                "reason": f"No netlist template for device_type '{device_type}'. "
                          f"Available: {list(NETLIST_TEMPLATES)}. Use add_connection to wire manually."}

    added, skipped = [], []
    for from_cat, from_ctype, to_cat, to_ctype, wire_cat in template:
        from_insts = [iid for iid, r in state.placed.items() if r["category"] == from_cat]
        to_insts = [iid for iid, r in state.placed.items() if r["category"] == to_cat]
        if not from_insts or not to_insts:
            skipped.append(f"{from_cat} -> {to_cat}: no placed component in "
                            f"{'from' if not from_insts else 'to'}-category yet")
            continue
        for from_iid in from_insts:
            for to_iid in to_insts:
                from_comp = COMPONENT_LIBRARY[state.placed[from_iid]["component_id"]]
                to_comp = COMPONENT_LIBRARY[state.placed[to_iid]["component_id"]]
                from_conns = [c["id"] for c in from_comp["connectors"] if c["type"] == from_ctype]
                to_conns = [c["id"] for c in to_comp["connectors"] if c["type"] == to_ctype]
                for i in range(min(len(from_conns), len(to_conns))):
                    result = tool_add_connection(state, from_iid, from_conns[i],
                                                  to_iid, to_conns[i], wire_cat)
                    if result["status"] == "added":
                        added.append(result["key"])
    return {"status": "ok", "connections_added": added, "template_rows_skipped": skipped}


def tool_route_all_wires(state: DesignState) -> dict:
    if not state.connections:
        return {"status": "rejected", "reason": "No connections registered yet — call "
                                                 "auto_populate_standard_netlist or add_connection first."}
    routed, unreachable, total_length = 0, [], 0.0
    for key, conn in state.connections.items():
        start = state.placed[conn["from_instance"]]["connectors_world"][conn["from_connector"]]
        goal = state.placed[conn["to_instance"]]["connectors_world"][conn["to_connector"]]
        result = route_wire(state.grid, start, goal)
        conn["status"] = result["status"]
        conn["waypoints_mm"] = result["waypoints_mm"]
        conn["length_mm"] = result["length_mm"]
        conn["reason"] = result["reason"]
        if result["status"] == "routed":
            routed += 1
            total_length += result["length_mm"]
        else:
            unreachable.append({"key": key, "reason": result["reason"]})
    return {"status": "ok", "total_connections": len(state.connections), "routed": routed,
            "unreachable": unreachable, "total_wire_length_mm": round(total_length, 1)}


def tool_get_design_status(state: DesignState) -> dict:
    unrouted = [k for k, c in state.connections.items() if c["status"] != "routed"]
    return {
        "status": "ok",
        "components_placed": [{"instance_id": iid, "label": r["label"], "component_id": r["component_id"],
                                "category": r["category"], "mass_g": r["mass_g"]}
                               for iid, r in state.placed.items()],
        "connections_summary": [{"key": k, "label": c["label"], "status": c["status"],
                                  "length_mm": c["length_mm"]} for k, c in state.connections.items()],
        "total_mass_g": state.total_mass_g(),
        "total_connections": len(state.connections),
        "routed_connections": len(state.connections) - len(unrouted),
        "unrouted_connections": unrouted,
        "ready_to_finalize": bool(state.connections) and not unrouted,
    }


def tool_finalize_design(state: DesignState, summary: str, confidence_notes: str) -> dict:
    status = tool_get_design_status(state)
    if not status["ready_to_finalize"]:
        return {"status": "rejected",
                "reason": f"Not ready: {len(status['unrouted_connections'])} connection(s) still "
                          f"unrouted ({status['unrouted_connections']}). Reposition the components "
                          f"involved and re-run route_all_wires before finalizing."}
    state.finalized = True
    state.final_summary = summary
    state.confidence_notes = confidence_notes
    return {"status": "finalized", "summary": summary, "confidence_notes": confidence_notes}


TOOL_DISPATCH = {
    "list_component_library": tool_list_component_library,
    "get_cavity_summary": tool_get_cavity_summary,
    "add_component": tool_add_component,
    "remove_component": tool_remove_component,
    "add_connection": tool_add_connection,
    "remove_connection": tool_remove_connection,
    "auto_populate_standard_netlist": tool_auto_populate_standard_netlist,
    "route_all_wires": tool_route_all_wires,
    "get_design_status": tool_get_design_status,
    "finalize_design": tool_finalize_design,
}


# =========================================================================
# OpenAI-style tool specs (same shape ai_client._to_openai_tools expects)
# =========================================================================
ELECTRONICS_AGENT_TOOLS = [
    {"name": "list_component_library",
     "description": "Browse available electronics parts, optionally filtered by category.",
     "parameters": {"type": "object", "properties": {
         "category": {"type": "string", "description": "One of the CATEGORIES, or omit for all."}}}},

    {"name": "get_cavity_summary",
     "description": "Get the usable interior volume of the CAD shell and how many components are placed so far. Call this first.",
     "parameters": {"type": "object", "properties": {}}},

    {"name": "add_component",
     "description": "Place one component instance inside the CAD shell at a given center point and rotation. "
                     "Runs a real collision check against the shell and every other placed component — a "
                     "'rejected' result means it does not fit there; read the reason and try a different position.",
     "parameters": {"type": "object", "properties": {
         "component_id": {"type": "string", "description": "id from list_component_library"},
         "instance_id": {"type": "string", "description": "your own unique label for this instance, e.g. 'fc_main'"},
         "center_mm": {"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3},
         "rotation_deg": {"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3,
                           "description": "Euler XYZ degrees, optional, default [0,0,0]"},
     }, "required": ["component_id", "instance_id", "center_mm"]}},

    {"name": "remove_component",
     "description": "Remove a previously placed component instance (also drops any connections referencing it).",
     "parameters": {"type": "object", "properties": {
         "instance_id": {"type": "string"}}, "required": ["instance_id"]}},

    {"name": "add_connection",
     "description": "Manually register one wire between two placed components' connectors. Prefer "
                     "auto_populate_standard_netlist for the standard topology; use this for anything "
                     "the template doesn't cover.",
     "parameters": {"type": "object", "properties": {
         "from_instance": {"type": "string"}, "from_connector": {"type": "string"},
         "to_instance": {"type": "string"}, "to_connector": {"type": "string"},
         "wire_category": {"type": "string", "enum": list(WIRE_CATEGORIES)},
     }, "required": ["from_instance", "from_connector", "to_instance", "to_connector", "wire_category"]}},

    {"name": "remove_connection",
     "description": "Remove a previously registered connection by its key.",
     "parameters": {"type": "object", "properties": {"key": {"type": "string"}}, "required": ["key"]}},

    {"name": "auto_populate_standard_netlist",
     "description": "Deterministically wires up the standard topology for this device_type between "
                     "whichever components are currently placed (e.g. FC->ESC signal, battery->PDB->ESC "
                     "power, FC->GPS/receiver/VTX data). Call this once most components are placed, "
                     "rather than hand-wiring the whole netlist yourself.",
     "parameters": {"type": "object", "properties": {
         "device_type": {"type": "string", "description": "Defaults to the run's device_type if omitted."}}}},

    {"name": "route_all_wires",
     "description": "Runs collision-aware 3D pathfinding for every registered connection through the "
                     "shell's free space. Re-run this after repositioning components — it always "
                     "re-routes everything, so results always reflect the current layout.",
     "parameters": {"type": "object", "properties": {}}},

    {"name": "get_design_status",
     "description": "Full status: placed components, total mass, routed/unrouted connection counts, "
                     "and whether the design is ready to finalize.",
     "parameters": {"type": "object", "properties": {}}},

    {"name": "finalize_design",
     "description": "Locks in the design. Only succeeds if every registered connection is routed — "
                     "call get_design_status first to confirm.",
     "parameters": {"type": "object", "properties": {
         "summary": {"type": "string", "description": "Plain-language summary of the final layout."},
         "confidence_notes": {"type": "string",
                               "description": "Anything you're unsure about — e.g. GPS placement "
                                               "relative to sky view, tight clearances worth a human "
                                               "double-check, parts library entries that should be "
                                               "swapped for exact datasheet values."},
     }, "required": ["summary", "confidence_notes"]}},
]

ELECTRONICS_AGENT_SYSTEM_PROMPT = """You are an electronics integration engineer. You are given a 3D CAD \
shell (already designed by a separate mechanical-design system) and a device's electronics requirements. \
Your job is to select real components, place them collision-free inside the shell, wire them up, and \
verify every wire actually routes through free space.

RULES:
- Only use component_ids returned by list_component_library — never invent one.
- Call get_cavity_summary first to see how much usable room you have.
- add_component runs a REAL collision check. A 'rejected' result means the placement is physically \
invalid — do not repeat the same position, move it based on the reason given.
- Prefer auto_populate_standard_netlist over hand-wiring the whole netlist yourself; only use \
add_connection for anything it reports as skipped.
- After placing components and populating the netlist, call route_all_wires. If any connection comes \
back unreachable, reposition the components involved (the cavity may be disconnected between them, or \
too tight) and route_all_wires again.
- Respect sky_view_advisory hints (GPS, receiver, VTX, telemetry, camera) by placing those parts near \
the shell's exterior/top where reasonable — this is advisory, not collision-checked, so use judgment.
- Call get_design_status before finalize_design and only finalize once ready_to_finalize is true.
- Be honest in confidence_notes about anything a human should double-check before this design is trusted \
for fabrication — especially parts-library dimensions that are representative rather than your exact BOM.
- Every placed component gets an automatic short label (area A, B, C, ...) and every wire gets one too \
(W1, W2, ...) — these are for the human viewing the final 3D result, not something you assign yourself. \
Feel free to refer to components by their label in your summary (e.g. "area B (GPS) sits near the top").
"""
