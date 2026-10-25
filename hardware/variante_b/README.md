# PiLiDAR Variante B – Testaufbau Stufe 1c

LiDAR-Halter V2 nach den echten STL27L-Maßen (Datenblatt V0.3), direkt auf das bestehende
Abtriebsteil (Vierkant 32 × 32 mm mit 1/4"-Stativgewinde) gesteckt und mit einer Schraube geklemmt.
Der STL27L steht gerade zur Vierkant-Aufnahme; die Rückwand sitzt bündig an der Vorderkante der
Bodenplatte und steht über die ganze Breite bis aufs Druckbett (keine freien Überhänge). Der Turmdeckel
liegt 55,1 mm von der Drehachse, 33,9 mm über der Motorseite.
Kamera und Hall-Sensor folgen später.

## Teile (in `out/`)

| Datei | Teil | Druck |
|---|---|---|
| `0_nabentest` | nur die Vierkant-Aufnahme – zuerst drucken und aufstecken | Tasche unten, ca. 15 min |
| `8_lidarhalter_v2` | 44,2 × 54,8 × 57,0 mm: Vierkant-Tasche 32,5 × 32,5 × 4 mm, Rückwand bündig an der Vorderkante, bis aufs Druckbett, mit den 3 Befestigungslöchern des STL27L (M2, Mutterntaschen hinten), Gewichtsfenster mit 45°-Dach, 2 Rippen | Tasche unten, ohne Stützen |
| `1_motorgehaeuse` | Grundplatte mit Stativmutter 1/4", Motorschacht NEMA 17 | liegend, ohne Stützen |

PETG, 4 Wände, 30 % Füllung. `.step` zum Weiterbearbeiten.

## Schnittstelle zum Abtrieb

- Vierkant-Tasche: 32,0 mm + 0,25 mm Spiel je Seite, 4 mm tief – überträgt das Drehmoment
- Mitte Ø 6,8 mm für eine Schraube 1/4"-20 (z. B. 1/2" lang) in das Stativgewinde des Abtriebsteils
- Kopf liegt oben auf der Deckplatte, mit Inbus zwischen Rückwand und Rippen erreichbar

Sitzt die Tasche zu stramm oder zu locker: `SQ_FIT` im Skript ändern, Nabentest neu drucken.

## Kleinteile

- 1 × Zylinderschraube 1/4"-20 × 1/2" (Halter → Abtrieb)
- 3 × M2 × 8 + Muttern (STL27L-Laschen → Rückwand; Köpfe in den Senkungen der Laschen)
- Motorgehäuse: 2 × M3 × 16 + Muttern, Stativmutter 1/4"-20

## Montage

1. Nabentest aufstecken – spielfrei, aber ohne Gewalt? Dann den Halter drucken.
2. Halter auf das Abtriebsteil stecken, Schraube 1/4" durch die Mitte festziehen.
3. STL27L mit der Grundplatte an die Rückwand, Lochpaar unten, Einzelloch oben; 3 × M2 von der LiDAR-Seite, Muttern hinten.

## Hinweise

- Lochbild aus dem Datenblatt: Lochpaar 46,80 mm, Einzelloch 31,92 mm darüber, Ø 2,50.
  Die Lage der LiDAR-Drehachse zu den Löchern ist aus der Datenblattzeichnung abgegriffen (± ca. 1 mm);
  den genauen Versatz liefert die Offset-Kalibrierung.
- Ohne Gegengewicht, damit der Halter klein bleibt. Bei Vibrationen später auf der Rippenseite ergänzen.
- `python pilidar_testaufbau.py` (CadQuery ≥ 2.4) erzeugt alle Dateien neu; Alternativen
  (große Platte, Sechskant-Nabe, Hall-Halter) stehen im Skript und lassen sich über `PARTS` aktivieren.
