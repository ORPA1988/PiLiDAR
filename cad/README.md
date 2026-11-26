# Scankopf v3 – zwei Wege

1. **Original-Gehäuse unverändert + Nachrüstsatz Lagerfuß** (empfohlen, wenn das Original-Gehäuse gedruckt wird oder schon vorhanden ist)
2. **Turm neu gezeichnet** (`turm_v3.py`), gleiche Bauweise, als CAD-Code – siehe unten

## Weg 1: Nachrüstsatz für das Original-Gehäuse

Das Gehäuse aus [PiLiDAR-Hardware/FDM](https://github.com/PiLiDAR/PiLiDAR-Hardware/tree/main/FDM) bleibt, wie es ist.
Getauscht wird nur der Getriebedeckel: `2_lagerdeckel_original.stl` ist der Original-Deckel (`gearbox_deckel`,
CC BY-NC-SA 4.0, Philip Gutjahr) mit angeformter Scheibe und Lagerhülse. Der Turm läuft damit auf zwei Kugellagern
6810-2RS um den Stativfuß; der Abtriebs-Vierkant steckt im Fuß und hält nur gegen Verdrehen.

![Lagerdeckel](out/vorschau_lagerdeckel.png)

| Datei | Teil | Drucklage |
|---|---|---|
| `2_lagerdeckel_original.stl` | ersetzt `gearbox_deckel` | Getriebeseite nach unten, Stützen nur vom Druckbett |
| `1_stativfuss` | Zapfen mit Vierkant-Aufnahme und Stativmutter | Flansch auf dem Bett |
| `6_ring_oben`, `7_distanz_aussen`, `8_distanz_innen` | Lagerringe | flach |
| `passprobe_*` | zuerst drucken | |

- Die vier Schrauben Deckel → Getriebe → Motor sitzen wie bisher, gleiche Länge; die Köpfe sind durch vier Löcher erreichbar.
- Montage: Abtriebsplatte (32-mm-Vierkant) abziehen, Deckel tauschen, Abtriebsplatte von unten wieder aufstecken,
  Lager und Ringe in die Hülse (Ring oben, Lager, zwei Distanzringe, Lager), Fuß mit Stativmutter einschieben.
- Zukauf: 2 × Rillenkugellager 6810-2RS (50 × 65 × 7), 1 × Mutter 1/4"-20 UNC.
- Der Turm steht 21 mm höher. Die Kalibrierwerte des Originals bleiben gültig (`POSITION [0, -37.5, -41.9]`).
- Die Hülse (Radius 37 mm) endet 0,5 mm vor der Scan-Ebene. Sie streift nur den Strahlrand in etwa ± 7° um die Lotrechte,
  also dort, wo ohnehin das Stativ steht.

Neu erzeugen: `python cad/nachruest_original.py <Pfad>/PiLiDAR-Hardware/FDM/gearbox_deckel.obj` (braucht zusätzlich `manifold3d`).

---

# Weg 2: Turm nach Originalbauweise, neu gezeichnet

Aufbau und Maße folgen dem Original-PiLiDAR (Gehäuse 92 × 70 × 200 mm, hochkant, alles dreht mit).
Neu gezeichnet als CAD-Code, weil es vom Original nur Netzdateien gibt.

![Vorschau](out/vorschau.png)

## Unterschiede zum Original

| | Original | hier |
|---|---|---|
| Lagerung | Turm steht auf dem Getriebeabtrieb | zwei Kugellager 6810-2RS um einen Stativfuß; der Abtriebs-Vierkant hält nur gegen Verdrehen |
| Scan-Ebene – Drehachse | 37,5 mm | 43,5 mm (die Lagerhülse hat 37 mm Radius und stünde sonst im Strahl) |
| STL27L | auf Adapter, Lochabstand 45,9 mm | direkt auf der Rückwand, 46,8 / 31,92 mm nach Datenblatt |
| LiDAR unter der Kameraachse | 41,9 mm | 31,0 mm |
| Powerbank 91 × 63 × 22 | in einer Mulde der Rückschale | auf einer Auflage an der Rückwand, mit Band |
| Elektronik | auf der Mittelplatte | am Lochraster des Frontdeckels (innen) |
| Höhe über der Stativplatte | etwa 216 mm | 237 mm (Lagerfuß 37 mm statt 16 mm) |

## Teile (`out/*.stl`, `out/*.step`)

| Datei | Teil | Drucklage |
|---|---|---|
| `1_stativfuss` | Zapfen Ø 50 mit Vierkant-Aufnahme und Schacht für die Stativmutter | Flansch auf dem Bett |
| `2_teller` | Bodenplatte 92 × 74 mit Lagerhülse | Oberseite auf dem Bett (kopfüber) |
| `3_mitte` | Wanne 92 × 200 × 42: Rückwand mit allen Aufnahmen, Seitenwände, Klemmleisten für den Getriebedeckel | liegend, Rückwand auf dem Bett |
| `4_front` | Deckel mit Objektivöffnung und Lochraster | Außenfläche auf dem Bett |
| `5_akkutraeger` | Auflage für die Powerbank | Anlagefläche auf dem Bett |
| `6_ring_oben`, `7_distanz_aussen`, `8_distanz_innen` | Lagerringe | flach |
| `passprobe_*` | Fußoberteil (Vierkant, Lager-Innenring), Lagersitz | zuerst drucken |

PETG, 0,2 mm, 4 Wände, 20 % Füllung. Größtes Teil 92 × 200 mm (passt auf den MK4S).

## Kleinteile

- 2 × Rillenkugellager 6810-2RS (50 × 65 × 7), 1 × Mutter 1/4"-20 UNC
- 4 × M3 × 16 (Teller → Mitte, von unten), 4 × M3 × 20 (Frontdeckel), 4 × M3 × 8 (Akkuträger), 2 × M3 × 6 (IMU)
- 3 × M2 × 8 (STL27L), 4 × M2.5 × 6 (Pi), 4 × M2.5 × 8 (Kamera) – alle Schrauben direkt in den Kunststoff
- Klettband bis 16 mm, Kabelbinder

## Montage

1. In die Lagerhülse von unten: Ring oben (1 mm), Lager, beide Distanzringe, Lager. Mutter in den Schacht des Fußes, Fuß einschieben.
2. Motor mit Getriebe auf den Teller stellen, Deckel nach unten; der Vierkant greift in den Fuß.
3. Mitte von oben über den Motor absenken. Die zwei Leisten greifen über den Deckelrand. 4 × M3 × 16 von unten.
4. Pi (USB nach links, von vorne gesehen), Kamera und IMU auf die Rückwand schrauben.
5. STL27L von hinten anschrauben, Stecker oben, Kabel durch den Schlitz. Akkuträger anschrauben, Powerbank aufsetzen,
   Band oben durch die zwei Schlitze und unten durch den Motorausschnitt.
6. ESP32, Treiber und Wandler mit Kabelbindern innen am Lochraster des Frontdeckels, Deckel mit 4 × M3 × 20 schließen.

## Maße für die Software (`out/geometrie.json`)

- `MODEL_Y_OFFSET = -43.5` (Abstand Scan-Ebene – Drehachse), LiDAR 31,0 mm unter der Kameraachse.
- Mindest-Scanwinkel 197° für Objekte ab 0,3 m, 190° ab 0,5 m.
- Hinter der Powerbank bleiben 5,0 mm bis zur Scan-Ebene (Original 4,9 mm); dickere Powerbanks stehen im Strahl.

## Annahmen (Parameter in `turm_v3.py`)

| Annahme | Parameter |
|---|---|
| Getriebedeckel ist die PiLiDAR-Variante 52,3 × 52,3 × 3 mm | `LID_SQ`, `LID_T` |
| Objektiv: Sensor 30 mm hinter der Frontfläche | `CAM_SENS` |
| Raspberry Pi 4 oder 5 (Bohrbild 58 × 49) | `PI_*` |
| Motorlänge 23 mm (17HS4023) | `MOTOR_L` |

## Neu erzeugen und prüfen

```
pip install build123d trimesh shapely rtree matplotlib
python cad/turm_v3.py     # STEP, STL, geometrie.json
python cad/pruefen.py     # Volumenkörper, Durchdringungen, freie Scan-Ebene, Schwerpunkt, Vorschau
```

Die STEP-Dateien lassen sich in Fusion öffnen und dort weiterbearbeiten.
