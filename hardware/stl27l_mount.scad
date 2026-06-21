// =====================================================================
// PiLiDAR 2.0 — Parametrischer Halter für LDROBOT STL27L (USB-Variante)
// ---------------------------------------------------------------------
// KORREKTE AUSRICHTUNG (entscheidend für die Punktwolken-Geometrie):
//   Der STL27L wird AUF DER SEITE montiert -> Spinachse HORIZONTAL (Y),
//   Scan-Ebene VERTIKAL (X-Z). Der Stepper revolviert diese Ebene um die
//   senkrechte Z-Achse (siehe backend/pointcloud.py). Würde der LiDAR flach
//   liegen, wäre die Scan-Ebene horizontal -> es entstünde KEINE 3D-Wolke.
//
//   Koordinaten dieses Modells:  Z = oben (Drehachse), Y = Spinachse des
//   LiDAR (horizontal, weg von der Rückplatte), X = quer.
//
// Konstruktionsziele:
//   * Vertikale Rückplatte, an der die LiDAR-BODENPLATTE flächig anliegt.
//   * Klemmung nur am Sockel (untere ~8 mm der Bautiefe) -> das 360°-Optik-
//     fenster (Band um den rotierenden Kopf) bleibt frei (Datenblatt:
//     Verdeckung verschlechtert die Messung).
//   * Geringe Masse / wenig Material in der Scan-Ebene (Eigenverdeckung),
//     ruhige kontinuierliche Drehung (Modus B).
//   * Kabelführung ZH1.5T-4P + Zugentlastung; Kabel dreht mit.
//
// Maße aus STL-27L Datasheet V0.3 (morpheusTEK / LDROBOT):
//   Gehäuse 54.00(L) x 46.29(W) x 34.80(H) mm; Bohrbild-Toleranz ±0.2
//   5 V / 290 mA, UART 921600 (3.3 V), Stecker ZH1.5T-4P
//   Pinfolge: 1=Tx, 2=PWM, 3=GND, 4=VCC
//
// HINWEIS: Exaktes Boden-Bohrbild steht nur in der Datenblatt-Zeichnung
// (Bild, S.6). Die Schraubloch-Parameter sind ANNAHMEN -> vor dem Druck
// prüfen. Die Sockelklemme hält auch ohne Schrauben.
// =====================================================================

/* [LiDAR-Körper] */
lidar_base_x = 54.00;   // Bodenplatte Länge  (-> X)
lidar_base_z = 46.29;   // Bodenplatte Breite (-> Z, vertikal)
lidar_depth  = 34.80;   // Bautiefe entlang Spinachse (-> Y)
grip_depth   = 8.0;     // wie tief (in Y) der Sockel umfasst wird
fit_clear    = 0.4;     // Druckpassung

/* [Optische Freistellung] */
window_margin = 2.0;    // Sicherheitsabstand der Klemme zum Fensterband

/* [Schraubmontage Boden -> Rückplatte — ANNAHME, bitte prüfen] */
use_screws    = false;
screw_pitch_x = 35.0;   // Lochabstand in X  (ANNAHME)
screw_pitch_z = 35.0;   // Lochabstand in Z  (ANNAHME)
screw_d       = 2.7;    // Durchgang M2.5

/* [Rückplatte] */
back_t   = 4.0;         // Dicke der Rückplatte (in Y)
back_pad = 5.0;         // Rand um die LiDAR-Bodenplatte

/* [Rotor-Grundplatte / Plattform] */
plate_d  = 70.0;        // Durchmesser
plate_t  = 4.0;         // Dicke
rotor_bolt_circle = 30.0;
rotor_bolt_d = 3.4;     // M3
rotor_bolt_n = 3;

/* [Versatz LiDAR-Zentrum zur Drehachse] (vgl. config GEOMETRY) */
center_height = 60.0;   // Höhe des LiDAR-Zentrums über der Grundplatte (Z)
axis_offset_y = 8.0;    // Versatz der Rückplatte von der Drehachse in Y

/* [Kabel/Stecker] */
cable_slot_w = 8.0;
cable_slot_h = 4.5;
strain_relief = true;

wall = 3.0;
$fn = 72;

bx = lidar_base_x + 2*fit_clear;   // Innenmaß X
bz = lidar_base_z + 2*fit_clear;   // Innenmaß Z
plate_w = bx + 2*back_pad;         // Rückplatte Breite (X)
plate_h = bz + 2*back_pad;         // Rückplatte Höhe (Z)

// ---------------------------------------------------------------------
module rotor_plate() {
    difference() {
        cylinder(d = plate_d, h = plate_t);
        for (i = [0:rotor_bolt_n-1])
            rotate([0,0,i*360/rotor_bolt_n])
              translate([rotor_bolt_circle/2,0,-1])
                cylinder(d = rotor_bolt_d, h = plate_t+2);
        translate([0,0,-1]) cylinder(d = 14, h = plate_t+2); // Kabel/Gewicht
    }
}

// Vertikale Rückplatte (Ebene X-Z), Vorderseite bei y = axis_offset_y
module back_plate() {
    z0 = center_height - plate_h/2;
    translate([-plate_w/2, axis_offset_y, z0])
        difference() {
            cube([plate_w, back_t, plate_h]);
            // Schrauben in die LiDAR-Bodenplatte (ANNAHME)
            if (use_screws)
                for (sx=[-1,1], sz=[-1,1])
                    translate([plate_w/2 + sx*screw_pitch_x/2, -1,
                               plate_h/2 + sz*screw_pitch_z/2])
                        rotate([-90,0,0]) cylinder(d=screw_d, h=back_t+2);
            // Kabeldurchlass (Stecker an der LiDAR-Bodenplatte)
            translate([plate_w/2 - cable_slot_w/2, -1, 2])
                cube([cable_slot_w, back_t+2, cable_slot_h]);
        }
}

// Sockelklemme: U-Rahmen, der die unteren grip_depth (Y) des LiDAR umfasst
module base_clamp() {
    z0 = center_height - bz/2;
    y0 = axis_offset_y + back_t;            // direkt vor der Rückplatte
    // Außenrahmen
    translate([-bx/2 - wall, y0, z0 - wall])
        difference() {
            cube([bx + 2*wall, grip_depth + wall, bz + 2*wall]);
            // Innentasche für den LiDAR-Körper
            translate([wall, -1, wall])
                cube([bx, grip_depth + 1, bz]);
            // große seitliche Fenster, damit nichts in die Scan-Ebene ragt
            translate([-1, wall, bz*0.18])
                cube([bx + 2*wall + 2, grip_depth, bz*0.64]);
        }
    // Zugentlastung am Kabelaustritt
    if (strain_relief)
        translate([bx/2 - cable_slot_w/2 - 1, y0, z0 + 1.5])
            cube([cable_slot_w + 2, 3, 4]);
}

// Schräge Streben Rückplatte -> Grundplatte (Stabilität, geringe Masse)
module gussets() {
    for (sx = [-1, 1])
        translate([sx*(plate_w/2 - wall), axis_offset_y, plate_t])
            rotate([0,0,0])
            linear_extrude(height = 0.001) {}
    // einfache dreieckige Streben
    for (sx = [-1, 1])
        hull() {
            translate([sx*(plate_w/2 - wall/2), axis_offset_y + back_t/2, plate_t])
                cube([wall, back_t, 0.1], center=true);
            translate([sx*(plate_w/2 - wall/2), axis_offset_y + back_t/2,
                       center_height - plate_h/2])
                cube([wall, back_t, 0.1], center=true);
            translate([sx*(plate_w/3), axis_offset_y + 22, plate_t])
                cube([wall, 0.1, 0.1], center=true);
        }
}

module mount() {
    rotor_plate();
    back_plate();
    base_clamp();
    gussets();
}

mount();

// =====================================================================
// EINBAU / AUSRICHTUNG (auch in der Doku, mit Skizze):
//   * Z = senkrecht = Drehachse des Stepper-Rotors.
//   * LiDAR-Bodenplatte liegt an der Rückplatte (steht senkrecht).
//   * Spinachse des LiDAR zeigt horizontal nach +Y (weg von der Platte).
//   * 360°-Optikfenster muss rundum frei sein -> nach Einbau im
//     Lidar-Only-/2D-Live-Modus prüfen.
//   * Stecker ZH1.5T-4P an der Bodenplatte -> Kabel durch Kanal nach unten,
//     mitdrehende Zugentlastung.
// Druck: PETG, 3–4 Wände, 30–40% Infill, 0.2 mm; Rückplatte flach auf das Bett
//   legen (Aussparungen ggf. mit minimalen Stützen).
// =====================================================================
