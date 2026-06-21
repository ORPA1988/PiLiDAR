"""Zentrale Konfiguration. config.json ist die Single-Source-of-Truth.

Die Klasse spiegelt die JSON-Struktur als verschachtelte, attributierbare Objekte
und stellt einige abgeleitete Werte bereit (z.B. Schritte pro Grad)."""

from __future__ import annotations

import json
import os
from pathlib import Path

# Projekt-Wurzel = Elternverzeichnis von backend/
ROOT = Path(__file__).resolve().parent.parent
CONFIG_PATH = ROOT / "config.json"


class _Section(dict):
    """dict mit Attribut-Zugriff (cfg.LIDAR.PORT)."""

    def __getattr__(self, name):
        try:
            value = self[name]
        except KeyError as exc:  # pragma: no cover - defensiv
            raise AttributeError(name) from exc
        if isinstance(value, dict):
            return _Section(value)
        return value


class Config:
    def __init__(self, path: os.PathLike | str = CONFIG_PATH):
        self.path = Path(path)
        self._data: dict = {}
        self.reload()

    # ------------------------------------------------------------------
    def reload(self) -> None:
        with open(self.path, "r", encoding="utf-8") as fh:
            self._data = json.load(fh)

    def save(self) -> None:
        with open(self.path, "w", encoding="utf-8") as fh:
            json.dump(self._data, fh, indent=2, ensure_ascii=False)
            fh.write("\n")

    # ------------------------------------------------------------------
    def __getattr__(self, name):
        # Wird nur aufgerufen, wenn das Attribut nicht regulär existiert.
        data = self.__dict__.get("_data", {})
        if name in data:
            value = data[name]
            return _Section(value) if isinstance(value, dict) else value
        raise AttributeError(name)

    def as_dict(self) -> dict:
        return self._data

    # --- abgeleitete Werte -------------------------------------------
    @property
    def steps_per_degree(self) -> float:
        s = self.STEPPER
        return (s["MICROSTEPS"] * s["GEAR_RATIO"]) / s["STEP_ANGLE"]

    def update_section(self, section: str, values: dict) -> None:
        """Einzelne Schlüssel in einer Sektion überschreiben und speichern."""
        self._data.setdefault(section, {})
        self._data[section].update(values)
        self.save()


# Modul-Singleton zur einfachen Wiederverwendung
config = Config()
