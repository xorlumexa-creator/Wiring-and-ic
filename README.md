# Lumexa Electronics Design Service

A separate service from the existing Lumexa CAD/FEA backend. It takes a
CAD shell (an STL that backend already generated) and a device's
electronics requirements, and produces a collision-checked, fully-wired
electronics layout inside that shell — viewable as an interactive X-ray
3D scene.

## Pipeline

```
CAD shell (.stl, from the existing Lumexa system)
        │
        ▼
spatial.py    → voxelizes the shell's interior into a free-space grid
                (signed-distance based: every voxel knows how far it is
                from the shell's skin)
        │
        ▼
agent_tools.py + ai_client.py
    → Nemotron 3 Ultra (via OpenRouter) runs a tool-calling loop:
        1. browse component_library.py for real parts (FC, ESC, GPS,
           receiver, VTX, camera, telemetry radio, PDB, battery...)
        2. place them with add_component — every placement is
           collision-checked against the grid AND every other placed
           part, for real, before it's accepted
        3. wire them with auto_populate_standard_netlist (deterministic
           topology template) + add_connection for anything extra
        4. route_all_wires — real A* pathfinding through the free-space
           grid for every wire, not a straight-line guess
        5. iterate on anything rejected/unrouted, then finalize_design
        │
        ▼
export_scene.py → one GLB: the shell (translucent, X-ray) + a colored box
                   per component (one distinct color per instance, not
                   per category) + a matching colored "mount-area" decal
                   projected onto the shell surface where that part
                   actually sits + wire tubes colored to real-life
                   convention (see below)
        │
        ▼
static/viewer.html → three.js viewer: opacity slider, cross-section
                      clipping plane, per-category wire toggles, floating
                      "A/B/C" area labels over each component, a
                      component sidebar + wire-color legend sourced from
                      manifest.json
```

## Labeling & color scheme

Every placed component gets a short, stable tag — **A, B, C, ...** — in
placement order, plus its own distinct color (not shared with other
components of the same category, so two ESCs or two GPS modules stay
visually distinguishable). Every wire gets a tag too — **W1, W2, ...**.
Both appear in the viewer's sidebar, as floating billboard labels in the
3D scene, and in `manifest.json` / the API response, so "area B" means
the same thing everywhere.

Each component renders as **two things** in the 3D scene, not one:
1. its actual 3D bounding box (so the collision/placement geometry is
   still inspectable), and
2. a **mount-area decal** — that component's color, painted onto the
   nearest patch of the actual CAD shell surface — which is the "here's
   where to physically put this part" answer the box alone doesn't give
   you. See the docstring at the top of `export_scene.py` for exactly how
   that projection works and its one real limitation (it assumes the
   nearest shell surface is the intended mounting surface, which is a
   sound default but won't be right 100% of the time for oddly-shaped
   cavities).

**Wire colors are matched to real-world convention where one actually
exists, and clearly flagged where it doesn't:**

| Category | Color | Basis |
|---|---|---|
| power | red | Universal DC positive/power convention |
| ground | black | Universal DC negative/ground convention |
| signal | white | Dominant ESC/servo PWM-signal convention (ArduPilot/Pixhawk docs, Futaba-style gear) — some brands (JR/Airtronics-style) use yellow or orange instead |
| data_bus | blue | **Not** a real-world standard — UART/telemetry cable coloring is manufacturer/connector-specific. This is a visualization choice; swap it if your actual cable's colors matter to you |

This table is also in `component_library.py`'s `WIRE_CATEGORIES` and
surfaced live in the viewer's legend.

## Why it's built this way (the accuracy question)

You asked for high accuracy. The place that actually comes from is **the
AI never grades its own work** — every tool call in `agent_tools.py`
either passes a real geometric check or it doesn't:

- **Placement** (`spatial.check_box_fits`): samples the component's
  actual bounding box against the voxel grid and against every other
  placed part's AABB. A "rejected" result is a genuine collision, not a
  guess.
- **Routing** (`routing.route_wire`): A* over the same voxel grid — the
  returned wire length is the length of a path that was actually verified
  to avoid the shell and every placed component, not a straight-line
  estimate.
- **Wiring topology** is a fixed template (`NETLIST_TEMPLATES` in
  `component_library.py`), not something the model improvises per run.
  That's deliberate: which pin connects to which is safety-relevant, so
  it's reviewed once as data, and the AI's real job is component choice +
  3D placement, not inventing a wiring diagram from scratch.

What is **not** independently verified, and is worth your attention
before trusting a design for fabrication:

- **Part dimensions** in `component_library.py` are representative
  industry-standard form factors (30.5mm/20x20mm FPV mounts, etc.), not
  pulled from your exact BOM's datasheets. Swap in your real numbers —
  it's a plain Python dict, no other code needs to change. The
  `finalize_design` tool asks the model to flag exactly this in its
  `confidence_notes`.
- **Collisions use AABB, not true oriented-box overlap** (see the
  docstring at the top of `agent_tools.py`) — conservative (can reject a
  placement that would truly fit), never permissive (can't accept a real
  collision).

## Setup

```bash
pip install -r requirements.txt
export OPENROUTER_API_KEY=sk-or-...
uvicorn main:app --reload
```

Then either:

```bash
curl -X POST http://localhost:8000/design-electronics \
  -F "cad_file=@drone_body.stl" \
  -F "device_type=quadcopter_4motor" \
  -F "requirements=500g AUW, GPS return-to-home, analog FPV, 20min flight time" \
  -F "max_iterations=6"
```

or open `http://localhost:8000/viewer` after a design has been generated
(it needs `?design_id=<id>` from the response above — the JSON response
gives you `viewer_url` directly).

## Environment variables

| Var | Default | Notes |
|---|---|---|
| `AI_PROVIDER` | `openrouter` | or `groq` |
| `OPENROUTER_API_KEY` | — | required if using OpenRouter |
| `OPENROUTER_MODEL` | `nvidia/nemotron-3-ultra-550b-a55b` | verify current id/pricing on OpenRouter before deploying |
| `GROQ_API_KEY` | — | only if `AI_PROVIDER=groq` |
| `AGENT_TURN_MAX_TOKENS` | `2000` | per-turn token cap for the tool-calling loop |

## Deploying to Render (free tier)

`render.yaml` is included — connect the repo in the Render dashboard,
it'll pick it up, then set `OPENROUTER_API_KEY` in the dashboard (kept out
of the yaml on purpose). Render's free tier is 512MB RAM **and 0.1 vCPU**
— the CPU allocation is the tighter constraint in practice, which is why
`spatial.py` and `routing.py` are tuned the way they are (see next
section).

## Reliability pass — what changed and why

This section exists because "trust me, it's reliable" isn't worth much —
here's exactly what was checked and fixed:

- **Fixed a real bounds-check bug**: `check_box_fits` and
  `nearest_routable_index` used to check `in_bounds()` on an index that
  `world_to_index()` had already clamped into valid range — which means
  the check was always true, even for a point genuinely outside the
  mesh's bounding box (e.g. a component whose padded clearance box pokes
  past the shell entirely). That point would silently get checked against
  whichever mask value happened to sit at the nearest boundary voxel
  instead of being rejected. Fixed by adding `is_within_grid_bounds`,
  which tests the raw (unclamped) point against the grid's real spatial
  extent — kept structurally separate from "how do I index this into the
  array," which still clamps (on purpose, for safe array access). Caught
  a real invalid placement during testing (see `agent_tools.py`'s test
  history) that the old code would have silently accepted.
- **check_box_fits is now vectorized and samples the box's surface + a
  coarse interior grid instead of a dense full-volume fill.** The old
  version sampled every point on a 3D grid filling the component's entire
  padded volume, one Python `world_to_index()` call at a time — for a
  modest component that's thousands of individual numpy calls in a tight
  Python loop, real latency risk on Render's 0.1 vCPU. The trade this
  makes: a thin obstruction fully enclosed inside a component's volume,
  touching none of its 6 faces and finer than the coarse interior grid,
  could theoretically be missed. Tested against exactly that case (see
  `test_spatial_logic.py`'s Test 6) — the coarse interior grid catches it.
- **Lowered `MAX_VOXELS` (400k → 120k) and A*'s `max_expansions` (300k →
  80k)** once I'd actually confirmed Render free tier is 0.1 vCPU, not
  just 512MB — both were tuned before that number was confirmed. The
  trade: on a very large or very fragmented cavity, a route/grid that
  would eventually resolve at the old, higher limits now fails faster
  instead — bounded worst-case latency over a small chance of a
  successful-but-slow result.
- **Mesh watertightness is now tracked and surfaced, not silently
  assumed.** `signed_distance` (everything in `spatial.py` depends on it)
  is only mathematically well-defined for a closed mesh. `load_mesh` now
  reports whether the upload needed repair and whether that repair fully
  succeeded; if it didn't, the agent gets an explicit warning in its
  prompt telling it to treat every placement/routing result with real
  skepticism, and the API response includes `mesh_watertight: false`
  rather than reporting confident-looking numbers built on a shaky
  foundation.
- **A design's real results now survive a scene-export failure.**
  Previously, if `build_scene_glb` threw for any reason after a
  successful agent run, the whole HTTP request failed and the caller got
  nothing — losing real, verified placement/routing work over a
  visualization bug. Now the manifest and tool_trace (the actual
  engineering results) always return; `scene_export_error` is populated
  and `scene_glb_url`/`viewer_url` are simply omitted if only the 3D
  export failed. Same principle applied per-component and per-wire inside
  `export_scene.py` — one bad item is skipped and logged, not fatal.
- **Mesh loading failures now return a clean 422**, not an opaque
  unhandled 500 — a corrupt/unsupported STL is a caller error, not a
  server error, and should read like one.
- Re-ran the full deterministic test suite after every change above
  (`test_astar_logic.py`, `test_spatial_logic.py` — now 6 cases including
  2 new regressions, `test_agent_tools_integration.py`) — all pass.

## Known gaps / next steps

- **Real parts data**: wiring in a real component database (Octopart,
  Digikey API, or just your own curated BOM file) instead of the
  representative library is the single highest-value accuracy upgrade.
- **True OBB collision**: swap the AABB check in `check_box_fits` for a
  separating-axis-theorem OBB-OBB test if you need components placed at
  arbitrary (non-axis-aligned) angles packed tightly.
- **Antenna/sky-view is advisory only** — `requires_sky_view` is surfaced
  to the model as a hint, not enforced geometrically. Could be upgraded to
  a real check (e.g. raycast from the connector outward to confirm it
  isn't fully enclosed by shell material).
- **Persistence**: `DESIGN_STORE` is in-memory (fine for a free-tier demo
  service, gone on restart). Swap for Supabase/S3 if designs need to
  outlive a restart.
- **Not yet load-tested against a real deployment.** Everything above was
  verified with synthetic tests against the real algorithmic code (no
  trimesh/fastapi available in the sandbox this was built in) — the
  constants (MAX_VOXELS, max_expansions, wall-clock budgets) are reasoned
  estimates for 0.1 vCPU, not measured against an actual Render instance.
  Worth a real smoke test with a real CAD file before trusting them.

