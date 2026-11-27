"""FastAPI-Server: REST-Steuerung + WebSocket-Rohdatenstream + statisches Frontend.

Start:  uvicorn backend.app:app --host 0.0.0.0 --port 8000
oder:   python -m backend.app
Mock-Modus (ohne Hardware, zum Testen am PC):  PILIDAR_MOCK=1 python -m backend.app
"""

from __future__ import annotations

import asyncio
import os
import subprocess
import time
from contextlib import asynccontextmanager
from pathlib import Path

import numpy as np
from fastapi import Body, FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, JSONResponse, Response
from fastapi.staticfiles import StaticFiles

try:
    import psutil as _psutil
except ImportError:
    _psutil = None

_net_prev: dict = {}  # {iface: (bytes_sent, bytes_recv, timestamp)}

from .config import config
from .controller import ScanController

ROOT = Path(__file__).resolve().parent.parent
FRONTEND = ROOT / "frontend"

FORCE_MOCK = os.environ.get("PILIDAR_MOCK", "0") == "1"

controller = ScanController(config, force_mock=FORCE_MOCK)

if FORCE_MOCK:
    # Synthetischer LiDAR -> komplette UI ohne Hardware demonstrierbar
    from .hardware.mock_serial import MockLidarSerial
    controller.lidar._serial_factory = lambda: MockLidarSerial()
    try:
        controller.confirm_home()  # simulierter ESP32: Ausgangslage gilt als bekannt
    except Exception:
        pass

_shutting_down = asyncio.Event()


@asynccontextmanager
async def _lifespan(_app):
    controller.bind_loop(asyncio.get_running_loop())
    yield
    _shutting_down.set()           # WebSocket-Schleifen beenden
    await asyncio.to_thread(controller.close)   # Treiber aus (EN 0), Ports schließen


app = FastAPI(title="PiLiDAR", version="2.0", lifespan=_lifespan)


@app.exception_handler(RuntimeError)
@app.exception_handler(TimeoutError)
async def _busy_handler(_request: Request, exc: Exception):
    """Bedienfehler (Beschäftigt, Position unbekannt, Port fehlt …) als lesbare
    JSON-Meldung statt "Internal Server Error" – die Oberfläche zeigt sie an."""
    return JSONResponse(status_code=409, content={"ok": False, "error": str(exc)})


@app.middleware("http")
async def _no_stale_frontend(request: Request, call_next):
    """Frontend-Dateien immer neu validieren (ETag/304), sonst läuft nach einem
    Update alter JavaScript-Code aus dem Browser-Cache."""
    resp = await call_next(request)
    if not request.url.path.startswith("/api/"):
        resp.headers["Cache-Control"] = "no-cache"
    return resp


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


@app.get("/api/scans/{scan_id}/pointcloud")
async def api_scan_pointcloud(scan_id: str):
    """Gespeicherte Punktwolke für die 3D-Anzeige als kompakter Binär-Buffer.

    Layout (little-endian): uint32 n | float32[n*3] xyz [m] | uint8[n] Intensität.
    Bei Millionen Punkten ist das um Größenordnungen kleiner/schneller als JSON.
    """
    buf = await asyncio.to_thread(controller.store.load_pointcloud_packed, scan_id)
    if len(buf) <= 4:  # nur der n=0-Header => keine Punktwolke
        return JSONResponse(status_code=404, content={"error": "keine Punktwolke"})
    return Response(content=buf, media_type="application/octet-stream")


@app.get("/api/scans/{scan_id}/raw")
async def api_scan_raw(scan_id: str):
    """Rohdaten als Binärpuffer; der Browser rechnet daraus die Punktwolke mit
    den aktuellen Offsets (siehe storage.load_raw_packed)."""
    buf = await asyncio.to_thread(controller.store.load_raw_packed, scan_id)
    if buf is None:
        return JSONResponse(status_code=404, content={"error": "keine Rohdaten"})
    return Response(content=buf, media_type="application/octet-stream")


@app.post("/api/calibrate/offset_from_scan")
async def api_cal_offset_from_scan(payload: dict = Body(...)):
    """Offset-Kalibrierung aus einem gespeicherten Scan (ohne Motor/LiDAR, ~3 s)."""
    return await asyncio.to_thread(controller.offset_from_scan, str(payload.get("scan_id", "")))


@app.post("/api/stepper/confirm_home")
async def api_confirm_home():
    """Benutzer bestätigt: Drehkopf steht in der Ausgangslage (Position = 0)."""
    await asyncio.to_thread(controller.confirm_home)
    return {"ok": True}


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


@app.get("/api/system/stats")
async def api_system_stats():
    if _psutil is None:
        return {"available": False}
    cpu = _psutil.cpu_percent(interval=None)
    ram = _psutil.virtual_memory().percent
    net: dict = {}
    try:
        counters = _psutil.net_io_counters(pernic=True)
        now = time.monotonic()
        for iface in ("eth0", "wlan0", "bnep0", "end0"):
            if iface not in counters:
                continue
            c = counters[iface]
            if iface in _net_prev:
                prev_sent, prev_recv, prev_t = _net_prev[iface]
                dt = max(1e-3, now - prev_t)
                net[iface] = {
                    "tx_bps": int((c.bytes_sent - prev_sent) / dt),
                    "rx_bps": int((c.bytes_recv - prev_recv) / dt),
                }
            else:
                net[iface] = {"tx_bps": 0, "rx_bps": 0}
            _net_prev[iface] = (c.bytes_sent, c.bytes_recv, now)
    except Exception:
        pass
    return {"available": True, "cpu_percent": cpu, "ram_percent": ram, "net": net}


@app.post("/api/system/reboot")
async def api_reboot():
    subprocess.run(["sudo", "systemctl", "reboot"], check=False)
    return {"ok": True}


@app.post("/api/system/poweroff")
async def api_poweroff():
    subprocess.run(["sudo", "systemctl", "poweroff"], check=False)
    return {"ok": True}


@app.delete("/api/scans/{scan_id}")
async def api_scan_delete(scan_id: str):
    controller.store.delete_scan(scan_id)
    return {"ok": True}


@app.post("/api/scans/{scan_id}/annotation")
async def api_scan_annotation(scan_id: str, payload: dict = Body(...)):
    controller.store.update_annotation(scan_id, str(payload.get("text", "")))
    return {"ok": True}


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
    lidar_cfg = config.LIDAR
    # geometrische Parameter einmalig senden, damit der Client rechnen kann
    await ws.send_json({
        "type": "init",
        "angle_offset": config.LIDAR["ANGLE_OFFSET"],
        "model_y": config.GEOMETRY["MODEL_Y_OFFSET"],
        "model_z": config.GEOMETRY["MODEL_Z_OFFSET"],
        "dist_min": config.LIDAR["DISTANCE_MIN_MM"],
        "dist_max": config.LIDAR["DISTANCE_MAX_MM"],
    })
    # Gegenstelle beobachten: schließt der Browser oder beim Beenden der Server die
    # Verbindung, endet die Schleife sofort (sonst bricht uvicorn sie nach 3 s mit
    # Fehlermeldung ab).
    closed = asyncio.create_task(ws.receive())
    try:
        while not _shutting_down.is_set() and not closed.done():
            try:
                first = await asyncio.wait_for(q.get(), timeout=0.5)
            except asyncio.TimeoutError:
                continue
            # alles Wartende bündeln und binär senden (statt 1 800 JSON-Nachrichten/s)
            batch = [first]
            while len(batch) < 400:
                try:
                    batch.append(q.get_nowait())
                except asyncio.QueueEmpty:
                    break
            await ws.send_bytes(_pack_frames(batch))
            await asyncio.sleep(0.03)   # ~30 Pakete/s genügen für die Anzeige
    except (WebSocketDisconnect, asyncio.CancelledError, RuntimeError):
        pass
    finally:
        closed.cancel()
        controller.unsubscribe(q)


def _pack_frames(batch: list[dict]) -> bytes:
    """Binärformat (little-endian) für den Browser:
    uint32 n | float32 z[n] | float32 speed[n] | float32 a[n*12] | uint16 d[n*12] | uint8 i[n*12]"""
    n = len(batch)
    z = np.fromiter((m["z"] for m in batch), dtype="<f4", count=n)
    s = np.fromiter((m["s"] for m in batch), dtype="<f4", count=n)
    a = np.asarray([m["a"] for m in batch], dtype="<f4")
    d = np.asarray([m["d"] for m in batch], dtype="<u2")
    i = np.asarray([m["i"] for m in batch], dtype=np.uint8)
    return np.asarray([n], dtype="<u4").tobytes() + z.tobytes() + s.tobytes() + a.tobytes() + d.tobytes() + i.tobytes()


# --- statisches Frontend (zuletzt gemountet) -------------------------
if FRONTEND.exists():
    app.mount("/", StaticFiles(directory=str(FRONTEND), html=True), name="frontend")
else:  # pragma: no cover
    @app.get("/")
    async def _root():
        return HTMLResponse("<h1>PiLiDAR</h1><p>Frontend fehlt.</p>")


def main():
    import uvicorn
    # kurze Frist beim Beenden: sonst wartet uvicorn auf offene WebSockets, bis
    # systemd nach 90 s hart beendet und controller.close() (EN 0) nie läuft
    uvicorn.run(app, host=config.SERVER["HOST"], port=config.SERVER["PORT"],
                timeout_graceful_shutdown=3)


if __name__ == "__main__":
    main()
