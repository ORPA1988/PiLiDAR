"""Startet den PiLiDAR-Server im Mock-Modus (ohne Hardware) für Demo/Preview."""
import os
import sys

os.environ["PILIDAR_MOCK"] = "1"
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import uvicorn

if __name__ == "__main__":
    port = int(os.environ.get("PORT", "8012"))
    uvicorn.run("backend.app:app", host="127.0.0.1", port=port, log_level="warning")
