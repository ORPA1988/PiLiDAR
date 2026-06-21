"""FastAPI-Server: REST-Steuerung + WebSocket-Rohdatenstream + statisches Frontend.

Start:  uvicorn backend.app:app --host 0.0.0.0 --port 8000
oder:   python -m backend.app
Mock-Modus (ohne Hardware, zum Testen am PC):  PILIDAR_MOCK=1 python -m backend.app
"""

from __future__ import annotations

import asyncio
import os
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, JSONResponse, Response
from fastapi.staticfiles import StaticFiles

from .config import config
from .controller import ScanController

ROOT = Path(__file__).resolve().parent.parent
FRONTEND = ROOT / "frontend"

FORCE_MOCK = os.environ.get("PILIDAR_MOCK", "0") == "1"

app = FastAPI(title="PiLiDAR", version="2.0")
controller = ScanController(config, force_mock=FORCE_MOCK)

if FORCE_MOCK:
    # Synthetischer LiDAR -> komplette UI ohne Hardware demonstrierbar
    from .hardware.mock_serial import MockLidarSerial
    controller.lidar._serial_factory = lambda: MockLidarSerial()


@app.on_event("startup")
async def _startup():
    controller.bind_loop(asyncio.get_running_loop())


@app.on_event("shutdown")
async def _shutdown():
    controller.close()


# --- REST -------------------------------------------------------------
@app.get("/api/status")
async def api_status():
    return controller.status()


@app.get("/api/config")
async def api_config():
    return config.as_dict()


@app.post("/api/lidar/start")
async def api_lidar_start():
    controller.start_lidar_only()
    return {"ok": True, "state": controller.state}


@app.post("/api/lidar/stop")
async def api_lidar_stop():
    controller.stop_lidar_only()
    return {"ok": True, "state": controller.state}


@app.post("/api/scan/start")
async def api_scan_start(payload: dict | None = None):
    payload = payload or {}
    mode = str(payload.get("mode", "B")).upper()
    name = str(payload.get("name", ""))
    scan_id = controller.start_scan(mode=mode, name=name)
    return {"ok": True, "scan_id": scan_id}


@app.post("/api/scan/stop")
async def api_scan_stop():
    controller.stop()
    return {"ok": True, "state": controller.state}


@app.get("/api/scans")
async def api_scans():
    return controller.store.list_scans()


@app.get("/api/scans/{scan_id}/download")
async def api_scan_download(scan_id: str):
    data = controller.store.zip_bytes(scan_id)
    return Response(content=data, media_type="application/zip",
                    headers={"Content-Disposition": f'attachment; filename="{scan_id}.zip"'})


@app.post("/api/calibrate/rotation")
async def api_cal_rotation():
    result = await asyncio.to_thread(controller.run_rotation_calibration)
    return result


@app.post("/api/calibrate/offset")
async def api_cal_offset():
    result = await asyncio.to_thread(controller.run_offset_calibration)
    return result


@app.post("/api/config/apply")
async def api_config_apply(payload: dict):
    """Kalibrierergebnisse übernehmen (z.B. gear_ratio, angle_offset, offsets)."""
    if "gear_ratio" in payload:
        config.update_section("STEPPER", {"GEAR_RATIO": float(payload["gear_ratio"])})
    geom = {}
    if "model_y_offset" in payload:
        geom["MODEL_Y_OFFSET"] = float(payload["model_y_offset"])
    if "model_z_offset" in payload:
        geom["MODEL_Z_OFFSET"] = float(payload["model_z_offset"])
    if geom:
        config.update_section("GEOMETRY", geom)
    if "angle_offset" in payload:
        config.update_section("LIDAR", {"ANGLE_OFFSET": float(payload["angle_offset"])})
    if "overlap_deg" in payload:
        config.update_section("MODE_B_CONTINUOUS", {"OVERLAP_DEG": float(payload["overlap_deg"])})
    return {"ok": True, "config": config.as_dict()}


# --- WebSocket: Rohdaten-Stream --------------------------------------
@app.websocket("/ws/scan")
async def ws_scan(ws: WebSocket):
    await ws.accept()
    q = controller.subscribe()
    # geometrische Parameter einmalig senden, damit der Client rechnen kann
    await ws.send_json({
        "type": "init",
        "angle_offset": config.LIDAR["ANGLE_OFFSET"],
        "model_y": config.GEOMETRY["MODEL_Y_OFFSET"],
        "model_z": config.GEOMETRY["MODEL_Z_OFFSET"],
        "dist_min": config.LIDAR["DISTANCE_MIN_MM"],
        "dist_max": config.LIDAR["DISTANCE_MAX_MM"],
    })
    try:
        while True:
            msg = await q.get()
            await ws.send_json({"type": "frame", **msg})
    except WebSocketDisconnect:
        pass
    finally:
        controller.unsubscribe(q)


# --- statisches Frontend (zuletzt gemountet) -------------------------
if FRONTEND.exists():
    app.mount("/", StaticFiles(directory=str(FRONTEND), html=True), name="frontend")
else:  # pragma: no cover
    @app.get("/")
    async def _root():
        return HTMLResponse("<h1>PiLiDAR</h1><p>Frontend fehlt.</p>")


def main():
    import uvicorn
    uvicorn.run(app, host=config.SERVER["HOST"], port=config.SERVER["PORT"])


if __name__ == "__main__":
    main()
