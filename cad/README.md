# Scankopf v3 – drehende Plattform

Unten steht nur der Stativfuß. Motor, Getriebe, STL27L mit USB-Board, Raspberry Pi, Kamera, IMU, Elektronik und Akku
sitzen auf der Plattform und drehen mit. Über die Drehstelle läuft kein Kabel, der Drehwinkel ist nicht begrenzt.
Der STL27L sitzt außen an der Frontwand; seine Scan-Ebene liegt 69 mm vor der Drehachse und sieht an Plattform,
Fuß und Stativ vorbei.

![Vorschau](out/vorschau.png)

## Aufbau

- Der Abtriebs-Vierkant des Getriebes (32 mm) steckt im Stativfuß und hält ihn gegen Verdrehen. Er trägt keine Last.
- Die Plattform läuft auf zwei Kugellagern 6810-2RS im Abstand von 19 mm (Mitte zu Mitte) um den Fuß.
- Der Getriebedeckel liegt auf dem Teller und wird von zwei Klemmleisten gehalten.

## Teile (`out/*.stl`, `out/*.step`)

| Datei | Teil | Drucklage |
|---|---|---|
| `1_stativfuss` | Zapfen Ø 50 mit Vierkant-Aufnahme, Schacht für die Stativmutter | Flansch auf dem Bett |
| `2_teller` | Platte 92 × 102 mit Lagerhülse und Akkuauflage | Oberseite auf dem Bett (kopfüber) |
| `3_aufbau` | vier Wände: LiDAR vorne, Pi links, Akku rechts, Lochraster hinten | wie exportiert |
| `4_bruecke` | Deckplatte mit Kameraplatte (30 × 30, M2.5), IMU-Löcher, Raster fürs USB-Board | Platte auf dem Bett |
| `5_klemmleiste` | hält den Getriebedeckel, **2 × drucken** | Senkungen nach unten |
| `6_ring_oben`, `7_distanz_aussen`, `8_distanz_innen` | Lagerringe | flach |
| `passprobe_*` | Fußoberteil (Vierkant und Lager-Innenring), Lagersitz | zuerst drucken |

PETG, 0,2 mm, 4 Wände, 20 % Füllung. Größtes Teil 92 × 102 mm.

## Kleinteile

- 2 × Rillenkugellager 6810-2RS (50 × 65 × 7), 1 × Mutter 1/4"-20 UNC
- 4 × M3 × 16 Teller → Aufbau (von unten), 4 × M3 × 12 Brücke, 4 × M3 × 8 Klemmleisten – alle direkt in den Kunststoff
- 3 × M2 × 8 STL27L (direkt in den Kunststoff)
- 4 × M2.5 × 12 mit 6-mm-Abstandshaltern und Muttern (Pi), 4 × M2.5 × 10 mit 3-mm-Abstandshaltern und Muttern (Kamera)
- 2 × M3 × 6 (IMU GY-521), Klettband 12–15 mm (Akku), Kabelbinder

## Montage

1. In die Lagerhülse von unten: Ring oben (1 mm), Lager, beide Distanzringe, Lager.
2. Stativmutter in den Schacht des Fußes fallen lassen, Fuß von unten in die Lager schieben.
3. Aufbau auf den Teller setzen, 4 × M3 × 16 von unten.
4. Motor mit Getriebe von oben einsetzen, Deckel nach unten. Der Vierkant greift in den Fuß.
   Klemmleisten vorne und hinten auflegen, 4 × M3 × 8.
5. STL27L außen an die Frontwand, Stecker oben; Kabel durch den Schlitz nach innen.
6. Pi an die linke Wand (USB nach hinten), Akku an die rechte Wand auf die Arme, Band durch die zwei Schlitze.
7. Brücke mit Kamera und IMU aufsetzen, 4 × M3 × 12. Elektronik mit Kabelbindern ans Lochraster der Rückwand.

## Maße für die Software (`out/geometrie.json`)

- Abstand Scan-Ebene – Drehachse: **69,0 mm** (`MODEL_Y_OFFSET = -69.0`). Innerhalb dieses Radius um die Achse
  (senkrecht über und unter dem Gerät) wird nichts erfasst.
- Wegen des Abstands reichen 180° nicht: mindestens 207° für Objekte ab 0,3 m, 196° ab 0,5 m. Empfehlung 210° oder 360°.
- LiDAR-Achse 64,9 mm, Kameraachse 114 mm, Gesamthöhe 136 mm über der Stativplatte.

## Annahmen (Parameter in `plattform_v3.py`)

| Annahme | Parameter |
|---|---|
| Getriebedeckel ist die PiLiDAR-Variante 52,3 × 52,3 × 3 mm | `LID_SQ`, `LID_T` |
| Akku: Powerbank bis 150 × 75 × 22 mm | `AKKU_*`, `SH_X1` |
| Raspberry Pi 4 oder 5 (Bohrbild 58 × 49) | `PI_Y`, `PI_Z` |
| Eintrittspupille des Fischauges 15 mm vor dem Sensor | `CAM_PUPIL` |
| Motorlänge 23 mm (17HS4023) | `MOTOR_L` |

Getriebemaße stammen aus den Modelldateien (Printables 782336 und PiLiDAR-Hardware): Gehäuse 42,3 × 42,3 × 20 mm,
Abtriebsfläche 16,0 mm unter der Deckel-Außenfläche.

## Neu erzeugen und prüfen

```
pip install build123d trimesh shapely rtree matplotlib
python cad/plattform_v3.py   # STEP, STL, geometrie.json
python cad/pruefen.py        # Volumenkörper, Durchdringungen, freie Scan-Ebene, Schwerpunkt, Vorschau
```
