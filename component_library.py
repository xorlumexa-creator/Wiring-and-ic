"""
component_library.py — the electronics parts catalog.

WHAT THIS IS
------------
A plain-Python database of the electronics that go *inside* a device (a
drone, initially) — flight controller, ESCs, GPS, receiver, VTX, camera,
telemetry radio, PDB, battery, buzzer/antenna — with real-world bounding
box dimensions, mass, and connector layouts.

ACCURACY NOTE — read this before trusting the numbers for fabrication
----------------------------------------------------------------------
The dimensions below are the *industry-standard, well-established form
factors* for each category (30.5mm/20x20mm FPV mounting patterns, generic
GPS module footprints, etc.) — they're good enough to drive genuinely
correct collision-free placement and wire-length math. They are NOT pulled
from a specific vendor's datasheet. Before you fabricate anything, replace
the relevant entries with the exact dimensions/connector positions from
YOUR actual bill of materials' datasheets. That's a deliberate design
choice: the geometric engine (collision checking, A* routing) is what
gives you real accuracy; the parts data is what you should own and keep
current, because it's your BOM, not a generic one.

Adding a real part: copy an existing entry with a matching category, edit
dims_mm / connectors / mass_g, give it a new unique id, done. Nothing else
in the codebase needs to change.

SCHEMA
------
Component:
    id            str   unique catalog key
    category      str   one of CATEGORIES below
    name          str   human label
    dims_mm       [L, W, H]   local bounding box before rotation (mm)
    mass_g        float
    clearance_mm  float  extra keep-out margin added around the part on
                          every side (connectors, heat, wiggle room)
    connectors    list of:
        id          str  unique within the component, e.g. "esc_signal_1"
        type        str  one of CONNECTOR_TYPES below
        local_pos_mm [x, y, z]  position relative to the component's own
                          center, BEFORE the component's placement
                          rotation is applied (agent_tools applies it)
    requires_sky_view  bool   True for GPS/antenna-type parts that need to
                          go near the top/exterior for signal — advisory
                          only, surfaced in status, never hard-enforced
    notes         str
"""

CATEGORIES = [
    "flight_controller", "esc_4in1", "esc_individual", "gps", "receiver",
    "vtx", "camera", "telemetry_radio", "pdb", "battery", "buzzer", "antenna",
]

CONNECTOR_TYPES = [
    "power_in", "power_out", "signal_out", "signal_in", "uart", "usb", "antenna_feed",
]

# Wire colors, chosen to match real-world hobby-electronics convention
# rather than an arbitrary palette:
#   power  = RED    — the one color code that's genuinely universal for DC
#                      positive/power, across electronics, automotive, RC.
#   ground = BLACK   — likewise universal for DC negative/ground.
#   signal = WHITE   — the dominant convention for a 3-wire ESC/servo PWM
#                      signal lead (this is what ArduPilot/Pixhawk docs and
#                      Futaba-style servos use). Brand variance exists —
#                      Airtronics/JR-style gear commonly uses yellow/orange
#                      instead — so treat this as "most common," not a law.
#   data_bus = BLUE  — UART/telemetry cables (GPS, receiver, VTX links) do
#                      NOT have a real universal color standard; connector
#                      color coding is manufacturer-specific (e.g. Matek vs
#                      Holybro GH1.25 cables differ). Blue here is a
#                      visualization choice, not a real-life match — swap
#                      per-wire to your actual cable's colors if that
#                      matters for your use case.
WIRE_CATEGORIES = {
    "power":     {"color_rgba": [225, 30, 30, 255], "min_clearance_mm": 1.0},
    "ground":    {"color_rgba": [20, 20, 20, 255], "min_clearance_mm": 1.0},
    "signal":    {"color_rgba": [240, 240, 235, 255], "min_clearance_mm": 0.5},
    "data_bus":  {"color_rgba": [40, 120, 220, 255], "min_clearance_mm": 0.5},
}

# One visually-distinct color per placed COMPONENT INSTANCE (not per
# category) — cycled through in placement order, so instance #1 and
# instance #2 of the same category (e.g. two ESCs) are still trivially
# tellable apart in the 3D view and the legend. 16 colors chosen for
# maximum pairwise contrast (a standard qualitative-palette approach);
# beyond 16 placed components it cycles, which is a reasonable point to
# stop trusting color alone and go by the A/B/C label instead.
INSTANCE_COLOR_PALETTE = [
    [230, 25, 75, 255], [60, 180, 75, 255], [255, 225, 25, 255], [0, 130, 200, 255],
    [245, 130, 48, 255], [145, 30, 180, 255], [70, 240, 240, 255], [240, 50, 230, 255],
    [210, 245, 60, 255], [250, 190, 212, 255], [0, 128, 128, 255], [220, 190, 255, 255],
    [170, 110, 40, 255], [255, 250, 200, 255], [128, 0, 0, 255], [170, 255, 195, 255],
]

# Kept as a reference/fallback (e.g. for tools that want "the" color for a
# category in the abstract) — the 3D scene and legend use per-instance
# colors above, not this, so that same-category duplicates stay distinct.
CATEGORY_COLOR_RGBA = {
    "flight_controller": [80, 200, 120, 255],
    "esc_4in1":          [230, 140, 40, 255],
    "esc_individual":    [230, 140, 40, 255],
    "gps":               [160, 80, 220, 255],
    "receiver":          [80, 160, 230, 255],
    "vtx":               [230, 80, 160, 255],
    "camera":            [40, 40, 40, 255],
    "telemetry_radio":   [100, 200, 220, 255],
    "pdb":               [200, 60, 60, 255],
    "battery":           [230, 200, 30, 255],
    "buzzer":            [180, 180, 180, 255],
    "antenna":           [120, 120, 120, 255],
}

COMPONENT_LIBRARY = {
    "fc_f7_generic": {
        "id": "fc_f7_generic", "category": "flight_controller",
        "name": "Generic F7 Flight Controller (30.5mm standard mount)",
        "dims_mm": [36.0, 36.0, 8.0], "mass_g": 9.0, "clearance_mm": 2.0,
        "connectors": [
            {"id": "esc_signal_1", "type": "signal_out", "local_pos_mm": [15, 15, -4]},
            {"id": "esc_signal_2", "type": "signal_out", "local_pos_mm": [-15, 15, -4]},
            {"id": "esc_signal_3", "type": "signal_out", "local_pos_mm": [-15, -15, -4]},
            {"id": "esc_signal_4", "type": "signal_out", "local_pos_mm": [15, -15, -4]},
            {"id": "battery_power", "type": "power_in", "local_pos_mm": [0, 18, -4]},
            {"id": "gps_uart", "type": "uart", "local_pos_mm": [0, -18, 4]},
            {"id": "rx_uart", "type": "uart", "local_pos_mm": [10, -18, 4]},
            {"id": "vtx_uart", "type": "uart", "local_pos_mm": [-10, -18, 4]},
        ],
        "requires_sky_view": False,
        "notes": "30.5mm hole-to-hole mount is the FPV-industry standard; "
                 "swap dims/connector count if your BOM's FC differs.",
    },
    "esc_4in1_30a": {
        "id": "esc_4in1_30a", "category": "esc_4in1",
        "name": "Generic 4-in-1 ESC, 30A/motor (30.5mm standard mount)",
        "dims_mm": [36.0, 36.0, 6.0], "mass_g": 12.0, "clearance_mm": 2.0,
        "connectors": [
            {"id": "signal_in_1", "type": "signal_in", "local_pos_mm": [15, 15, 3]},
            {"id": "signal_in_2", "type": "signal_in", "local_pos_mm": [-15, 15, 3]},
            {"id": "signal_in_3", "type": "signal_in", "local_pos_mm": [-15, -15, 3]},
            {"id": "signal_in_4", "type": "signal_in", "local_pos_mm": [15, -15, 3]},
            {"id": "motor_out_1", "type": "power_out", "local_pos_mm": [18, 18, 0]},
            {"id": "motor_out_2", "type": "power_out", "local_pos_mm": [-18, 18, 0]},
            {"id": "motor_out_3", "type": "power_out", "local_pos_mm": [-18, -18, 0]},
            {"id": "motor_out_4", "type": "power_out", "local_pos_mm": [18, -18, 0]},
            {"id": "battery_power", "type": "power_in", "local_pos_mm": [0, 18, -3]},
        ],
        "requires_sky_view": False, "notes": "",
    },
    "gps_generic": {
        "id": "gps_generic", "category": "gps",
        "name": "Generic GPS + compass module",
        "dims_mm": [24.0, 24.0, 10.0], "mass_g": 7.0, "clearance_mm": 1.0,
        "connectors": [
            {"id": "uart_out", "type": "uart", "local_pos_mm": [0, -10, 0]},
            {"id": "antenna_feed", "type": "antenna_feed", "local_pos_mm": [0, 0, 5]},
        ],
        "requires_sky_view": True,
        "notes": "Compass is magnetically sensitive — keep away from power wiring "
                 "and motors in the final placement (not automatically enforced here).",
    },
    "rx_generic": {
        "id": "rx_generic", "category": "receiver",
        "name": "Generic RC receiver",
        "dims_mm": [20.0, 15.0, 5.0], "mass_g": 3.0, "clearance_mm": 1.0,
        "connectors": [
            {"id": "uart_out", "type": "uart", "local_pos_mm": [0, -7, 0]},
            {"id": "antenna_feed", "type": "antenna_feed", "local_pos_mm": [0, 7, 0]},
        ],
        "requires_sky_view": True, "notes": "",
    },
    "vtx_generic": {
        "id": "vtx_generic", "category": "vtx",
        "name": "Generic analog/digital VTX",
        "dims_mm": [28.0, 28.0, 6.0], "mass_g": 8.0, "clearance_mm": 2.0,
        "connectors": [
            {"id": "uart_in", "type": "uart", "local_pos_mm": [0, -14, 0]},
            {"id": "camera_signal_in", "type": "signal_in", "local_pos_mm": [-10, 14, 0]},
            {"id": "power_in", "type": "power_in", "local_pos_mm": [10, 14, 0]},
            {"id": "antenna_feed", "type": "antenna_feed", "local_pos_mm": [0, 0, 3]},
        ],
        "requires_sky_view": True, "notes": "VTX runs hot — leave clearance for airflow.",
    },
    "camera_fpv_generic": {
        "id": "camera_fpv_generic", "category": "camera",
        "name": "Generic FPV camera",
        "dims_mm": [19.0, 19.0, 20.0], "mass_g": 6.0, "clearance_mm": 1.0,
        "connectors": [{"id": "signal_out", "type": "signal_out", "local_pos_mm": [0, 0, -10]}],
        "requires_sky_view": True, "notes": "Must face forward/outward — placement should be at the airframe's front face.",
    },
    "telemetry_radio_generic": {
        "id": "telemetry_radio_generic", "category": "telemetry_radio",
        "name": "Generic telemetry radio module",
        "dims_mm": [22.0, 13.0, 6.0], "mass_g": 4.0, "clearance_mm": 1.0,
        "connectors": [
            {"id": "uart_in", "type": "uart", "local_pos_mm": [0, -6, 0]},
            {"id": "antenna_feed", "type": "antenna_feed", "local_pos_mm": [0, 6, 0]},
        ],
        "requires_sky_view": True, "notes": "",
    },
    "pdb_generic": {
        "id": "pdb_generic", "category": "pdb",
        "name": "Generic power distribution board",
        "dims_mm": [30.0, 30.0, 3.0], "mass_g": 5.0, "clearance_mm": 1.0,
        "connectors": [
            {"id": "battery_in", "type": "power_in", "local_pos_mm": [0, 15, 0]},
            {"id": "esc_power_out", "type": "power_out", "local_pos_mm": [0, -15, 0]},
        ],
        "requires_sky_view": False, "notes": "",
    },
    "battery_generic_4s": {
        "id": "battery_generic_4s", "category": "battery",
        "name": "Generic 4S LiPo pack (edit dims_mm to your exact pack)",
        "dims_mm": [70.0, 35.0, 35.0], "mass_g": 190.0, "clearance_mm": 1.0,
        "connectors": [{"id": "power_out", "type": "power_out", "local_pos_mm": [0, -17.5, 0]}],
        "requires_sky_view": False,
        "notes": "Battery dims vary a LOT by capacity/brand — this is the entry most "
                 "worth replacing with your exact pack's measured dimensions.",
    },
    "buzzer_generic": {
        "id": "buzzer_generic", "category": "buzzer",
        "name": "Generic buzzer",
        "dims_mm": [12.0, 12.0, 6.0], "mass_g": 2.0, "clearance_mm": 0.5,
        "connectors": [{"id": "power_in", "type": "power_in", "local_pos_mm": [0, -6, 0]}],
        "requires_sky_view": False, "notes": "",
    },
}


# ---------------------------------------------------------------------
# Standard netlist templates — deterministic, by design.
#
# WHY: wiring topology (what connects to what) is safety-relevant. Having
# the LLM invent a wiring diagram from scratch on every run is the wrong
# place to spend "creativity" — instead, the netlist is a fixed, reviewed
# template per device type, and the AI agent's real job is (a) choosing
# WHICH parts fill each slot and (b) finding a collision-free, sky-view-
# respecting 3D placement for them inside the given CAD shell. The
# auto_populate_standard_netlist tool in agent_tools.py expands this
# template against whichever components the agent actually placed.
# ---------------------------------------------------------------------
NETLIST_TEMPLATES = {
    "quadcopter_4motor": [
        # (from_category, from_connector_type, to_category, to_connector_type, wire_category)
        ("flight_controller", "signal_out", "esc_4in1", "signal_in", "signal"),
        ("battery", "power_out", "pdb", "power_in", "power"),
        ("pdb", "power_out", "esc_4in1", "power_in", "power"),
        ("battery", "power_out", "flight_controller", "power_in", "power"),
        ("flight_controller", "uart", "gps", "uart", "data_bus"),
        ("flight_controller", "uart", "receiver", "uart", "data_bus"),
        ("flight_controller", "uart", "vtx", "uart", "data_bus"),
        ("flight_controller", "uart", "telemetry_radio", "uart", "data_bus"),
        ("camera", "signal_out", "vtx", "signal_in", "signal"),
        ("flight_controller", "power_in", "buzzer", "power_in", "signal"),
    ],
}


def get_component(component_id: str) -> dict:
    if component_id not in COMPONENT_LIBRARY:
        raise KeyError(f"Unknown component id '{component_id}'. "
                        f"Call list_component_library to see valid ids.")
    return COMPONENT_LIBRARY[component_id]


def list_components(category: str | None = None) -> list[dict]:
    items = list(COMPONENT_LIBRARY.values())
    if category:
        items = [c for c in items if c["category"] == category]
    # Return a slim view — the agent doesn't need connector local_pos_mm at
    # browse time, only at placement/wiring time (kept in the full record).
    return [{"id": c["id"], "category": c["category"], "name": c["name"],
              "dims_mm": c["dims_mm"], "mass_g": c["mass_g"],
              "requires_sky_view": c["requires_sky_view"], "notes": c["notes"]}
             for c in items]
