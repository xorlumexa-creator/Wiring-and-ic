"""
main.py — Lumexa Electronics Design Service.

Takes a CAD shell (an STL exported from the existing Lumexa CAD/FEA
system) plus a device's electronics requirements, runs an AI agent
(Nemotron 3 Ultra via OpenRouter, tool-calling) that selects components
and places/wires them inside the shell, verifies every placement and
every wire route deterministically (spatial.py / routing.py), and returns
an X-ray-viewable 3D scene.

This is a separate repo/service from the CAD+FEA backend by design (per
the brief) — it only *consumes* a CAD mesh file, it doesn't generate or
modify one. Point it at whatever STL the existing system's
/generate-validate-refine or /engineering-agent endpoints produced.
"""
from __future__ import annotations

import asyncio
import os
import time
import uuid

from fastapi import FastAPI, File, Form, HTTPException, UploadFile
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse, Response

from agent_tools import (
    ELECTRONICS_AGENT_SYSTEM_PROMPT, ELECTRONICS_AGENT_TOOLS, TOOL_DISPATCH, DesignState,
    tool_get_cavity_summary,
)
from ai_client import (
    AGENT_TURN_MAX_TOKENS, AI_PROVIDER, GROQ_API_KEY, OPENROUTER_API_KEY, OPENROUTER_MODEL,
    ProviderError, call_model_with_tools, format_tool_result_message, parse_retry_after_seconds,
)
from component_library import NETLIST_TEMPLATES
from export_scene import build_design_manifest, build_scene_glb
from spatial import CavityGrid, load_mesh

app = FastAPI(title="Lumexa Electronics Design Service")

# Wide open by default since this is meant to be called cross-origin from
# the existing Lumexa frontend/backend on a different Render service —
# tighten to your actual frontend origin before this is public-facing.
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])

STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")

# In-memory design store. This service is stateless-by-design for the
# Render free tier (no DB, restarts lose history) — the /design-electronics
# response includes the full manifest and a base64-able GLB reference so
# the caller isn't strictly dependent on this surviving a restart; this
# store just makes /designs/{id}/... convenient for the same session.
DESIGN_STORE: dict[str, dict] = {}
MAX_STORED_DESIGNS = 20


def _evict_old_designs():
    if len(DESIGN_STORE) <= MAX_STORED_DESIGNS:
        return
    oldest = sorted(DESIGN_STORE.items(), key=lambda kv: kv[1]["created"])[:-MAX_STORED_DESIGNS]
    for design_id, _ in oldest:
        DESIGN_STORE.pop(design_id, None)


@app.get("/health")
async def health():
    return {
        "status": "ok",
        "ai_provider": AI_PROVIDER,
        "ai_model": OPENROUTER_MODEL if AI_PROVIDER == "openrouter" else None,
        "ai_configured": bool(OPENROUTER_API_KEY or GROQ_API_KEY),
        "device_types_supported": list(NETLIST_TEMPLATES),
        "designs_in_memory": len(DESIGN_STORE),
    }


@app.get("/")
async def root():
    return {
        "service": "Lumexa Electronics Design Service",
        "endpoints": {
            "POST /design-electronics": "multipart form: cad_file, device_type, requirements, max_iterations",
            "GET /designs/{id}/scene.glb": "the X-ray-viewable 3D scene",
            "GET /designs/{id}/manifest.json": "BOM + netlist + wire lengths as JSON",
            "GET /viewer?design_id={id}": "interactive X-ray viewer",
        },
    }


async def run_electronics_agent(mesh_path: str, device_type: str, requirements: str,
                                 max_iterations: int = 6):
    mesh, was_watertight, is_watertight_now = load_mesh(mesh_path)
    grid = CavityGrid(mesh, mesh_watertight=is_watertight_now)
    state = DesignState(grid, device_type)

    cavity = tool_get_cavity_summary(state)
    watertight_note = ""
    if not was_watertight:
        watertight_note = (
            "\nNOTE: the uploaded CAD shell was not watertight and needed automatic hole-filling "
            f"before this analysis could run. {'The repair succeeded' if is_watertight_now else 'The '
            'repair did NOT fully succeed'} — "
            + ("cavity/placement/routing results below should still be reasonably trustworthy."
               if is_watertight_now else
               "treat every placement and wire-routing result below with real skepticism; "
               "the underlying free-space calculation may not be accurate for this mesh. Consider "
               "re-exporting the STL with better mesh repair before trusting this design.") + "\n"
        )
    user_msg = (
        f"DEVICE TYPE: {device_type}\n"
        f"REQUIREMENTS: {requirements or '(none given — use reasonable defaults for this device type)'}\n"
        f"{watertight_note}\n"
        f"CAD shell bounding box: {cavity['bbox_extents_mm']} mm. "
        f"Usable interior volume: ~{cavity['usable_volume_mm3']:.0f} mm^3 "
        f"({cavity['usable_fraction_of_bbox']*100:.0f}% of the bounding box, at "
        f"{cavity['grid_pitch_mm']}mm grid resolution).\n"
        f"Standard netlist template available for this device_type: "
        f"{'yes' if device_type in NETLIST_TEMPLATES else 'NO — you will need to wire manually with add_connection'}.\n\n"
        f"Begin with list_component_library to see what's available, then start placing components."
    )
    messages = [{"role": "system", "content": ELECTRONICS_AGENT_SYSTEM_PROMPT},
                {"role": "user", "content": user_msg}]

    max_steps = max(12, min(int(max_iterations) * 6, 40))
    tool_trace = []
    stopped_reason = None
    no_tool_call_strikes = 0
    rate_limit_wait_remaining = 60.0
    run_started = time.time()
    MAX_WALL_CLOCK_SECONDS = 200.0  # server gives up before Render's own request timeout does

    for step in range(1, max_steps + 1):
        if time.time() - run_started > MAX_WALL_CLOCK_SECONDS:
            stopped_reason = "wall_clock_budget_exceeded"
            break

        assistant = None
        while True:
            try:
                assistant = await asyncio.to_thread(
                    call_model_with_tools, messages, ELECTRONICS_AGENT_TOOLS, 0.2, AGENT_TURN_MAX_TOKENS)
                break
            except ProviderError as e:
                if e.status_code == 429 and rate_limit_wait_remaining > 0:
                    wait_s = min(parse_retry_after_seconds(e.detail), rate_limit_wait_remaining)
                    rate_limit_wait_remaining -= wait_s
                    print(f"[electronics-agent] step {step}: 429, waiting {wait_s:.1f}s "
                          f"({rate_limit_wait_remaining:.1f}s retry budget left)")
                    await asyncio.sleep(wait_s)
                    continue
                stopped_reason = "rate_limited" if e.status_code == 429 else f"model_call_failed: {e.detail}"
                break
        if assistant is None:
            break

        messages.append(assistant["_raw_message"])

        if not assistant["tool_calls"]:
            if assistant.get("content") and not state.final_summary:
                state.final_summary = assistant["content"][:2000]
            no_tool_call_strikes += 1
            if no_tool_call_strikes > 2:
                stopped_reason = "model_stopped_calling_tools"
                break
            messages.append({"role": "user",
                              "content": "Keep going using the tools — call get_design_status to see "
                                         "what's left, or finalize_design if everything is routed."})
            continue

        for tc in assistant["tool_calls"]:
            fn = TOOL_DISPATCH.get(tc["name"])
            if fn is None:
                result = {"status": "error", "reason": f"Unknown tool '{tc['name']}'"}
            else:
                try:
                    result = fn(state, **tc["arguments"])
                except TypeError as e:
                    result = {"status": "error", "reason": f"Bad arguments for {tc['name']}: {e}"}
                except Exception as e:  # noqa: BLE001 — a tool bug shouldn't kill the whole run
                    result = {"status": "error", "reason": f"{type(e).__name__}: {e}"}
            tool_trace.append({"step": step, "tool": tc["name"], "arguments": tc["arguments"],
                                "result_status": result.get("status")})
            messages.append(format_tool_result_message(tc["id"], tc["name"], result))

        if state.finalized:
            stopped_reason = "finalized"
            break

    if stopped_reason is None:
        stopped_reason = "max_steps_reached"

    return state, mesh, tool_trace, stopped_reason


@app.post("/design-electronics")
async def design_electronics(
    cad_file: UploadFile = File(..., description="STL exported from the existing Lumexa CAD system"),
    device_type: str = Form("quadcopter_4motor"),
    requirements: str = Form(""),
    max_iterations: int = Form(6),
):
    if not (OPENROUTER_API_KEY or GROQ_API_KEY):
        raise HTTPException(500, "No AI provider configured — set OPENROUTER_API_KEY (and AI_PROVIDER="
                                  "openrouter, the default) or GROQ_API_KEY (AI_PROVIDER=groq).")

    tmp_path = os.path.join("/tmp", f"{uuid.uuid4().hex}_{cad_file.filename or 'shell.stl'}")
    with open(tmp_path, "wb") as f:
        f.write(await cad_file.read())

    try:
        try:
            state, mesh, tool_trace, stopped_reason = await run_electronics_agent(
                tmp_path, device_type, requirements, max_iterations)
        except Exception as e:
            # Mesh loading/voxelization is the one part of this endpoint
            # that depends entirely on what the caller uploaded — a
            # corrupt STL, an unsupported format, or a degenerate mesh
            # should come back as a clear 4xx, not an opaque 500.
            raise HTTPException(422, f"Could not process the uploaded CAD file: "
                                      f"{type(e).__name__}: {e}") from e
    finally:
        if os.path.exists(tmp_path):
            os.remove(tmp_path)

    design_id = uuid.uuid4().hex
    scene_export_error = None
    try:
        glb_bytes = build_scene_glb(mesh, state)
    except Exception as e:  # noqa: BLE001 — the design itself is still valid even if the viewer export isn't
        glb_bytes = None
        scene_export_error = f"{type(e).__name__}: {e}"

    manifest = build_design_manifest(state)
    if glb_bytes is not None:
        DESIGN_STORE[design_id] = {"glb": glb_bytes, "manifest": manifest, "created": time.time()}
        _evict_old_designs()

    response = {
        "design_id": design_id,
        "stopped_reason": stopped_reason,
        "mesh_watertight": state.grid.mesh_watertight,
        "manifest": manifest,
        "tool_trace": tool_trace,
    }
    if glb_bytes is not None:
        response.update({
            "scene_glb_url": f"/designs/{design_id}/scene.glb",
            "manifest_url": f"/designs/{design_id}/manifest.json",
            "viewer_url": f"/viewer?design_id={design_id}",
        })
    else:
        response["scene_export_error"] = (
            f"The design itself completed, but building the 3D viewer scene failed: "
            f"{scene_export_error}. The manifest/tool_trace above still reflect real, "
            f"verified placement and routing results — only the visualization failed."
        )
    return JSONResponse(response)


@app.get("/designs/{design_id}/scene.glb")
async def get_scene_glb(design_id: str):
    entry = DESIGN_STORE.get(design_id)
    if entry is None:
        raise HTTPException(404, "Design not found (in-memory store — cleared on restart, or evicted "
                                  "if more than 20 designs have been generated since).")
    return Response(content=entry["glb"], media_type="model/gltf-binary")


@app.get("/designs/{design_id}/manifest.json")
async def get_manifest(design_id: str):
    entry = DESIGN_STORE.get(design_id)
    if entry is None:
        raise HTTPException(404, "Design not found.")
    return JSONResponse(entry["manifest"])


@app.get("/viewer")
async def viewer():
    path = os.path.join(STATIC_DIR, "viewer.html")
    if not os.path.exists(path):
        raise HTTPException(404, "viewer.html missing from static/.")
    return FileResponse(path, media_type="text/html")


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=int(os.environ.get("PORT", 8000)))
