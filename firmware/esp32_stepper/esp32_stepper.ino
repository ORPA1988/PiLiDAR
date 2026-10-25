// PiLiDAR Variante B – Inbetriebnahme-Firmware Stepper
// ESP32 DevKit (ESP-WROOM-32) + A4988 + NEMA17 17HS4023 + Planetengetriebe 1 + 38/14
//
// Verdrahtung:
//   STEP D25 · DIR D26 · EN D27 (LOW = Treiber ein)
//   MS1 D32 · MS2 D14 · MS3 D13 · VDD 3V3 · GND GND · RESET–SLEEP gebrückt
//
// Serielle Schnittstelle 115200 Bd, Zeilenende "\n". Winkel = Drehkopf (nach Getriebe).
//   STATUS            Zustand ausgeben
//   EN 1 | EN 0       Treiber ein / aus (aus = Motor stromlos, Bewegung abgebrochen)
//   MS 1|2|4|8|16     Mikroschritt-Teilung (nur im Stillstand)
//   SPEED <°/s>       Maximalgeschwindigkeit für MOVE/GOTO   (Standard 20)
//   ACC <°/s²>        Beschleunigung                        (Standard 40)
//   MOVE <°>          relativ fahren, z. B. MOVE 90 / MOVE -90
//   GOTO <°>          absolut fahren
//   RUN <°/s>         Dauerlauf mit Vorzeichen, z. B. RUN 1.67
//   STOP              mit Rampe anhalten
//   ZERO              aktuelle Position = 0°
//   REP <ms>          alle <ms> eine Zeile "P <micros> <schritte> <grad>" (0 = aus)
// Antworten: "OK ...", "ERR ...", nach Zielerreichung "DONE <grad>".

#include <Arduino.h>
#include <math.h>

const int PIN_STEP = 25, PIN_DIR = 26, PIN_EN = 27;
const int PIN_MS1 = 32, PIN_MS2 = 14, PIN_MS3 = 13;

const float GEAR = 1.0f + 38.0f / 14.0f;   // 3,7142857
const int FULL_STEPS = 200;

int   ms = 16;
float speedDeg = 20.0f, accDeg = 40.0f;
float spd = 0, acc = 0, vmin = 0;          // in Schritten/s bzw. Schritten/s²

enum Mode { IDLE, MOVE, RUN };
Mode  mode = IDLE;
bool  enabled = false;
long  pos = 0, target = 0;                  // Schritte bei aktueller Teilung
float v = 0;                                // Betrag Schritte/s
int   dir = 1;
float runV = 0;                             // Schritte/s mit Vorzeichen
uint32_t nextUs = 0, repMs = 0, lastRep = 0;
String line;

float stepsPerDeg() { return FULL_STEPS * ms * GEAR / 360.0f; }
float posDeg() { return pos / stepsPerDeg(); }

void updateKinematics() {
  spd = speedDeg * stepsPerDeg();
  acc = accDeg * stepsPerDeg();
  vmin = sqrtf(2.0f * acc);
}

void setMsPins(int m) {
  int a = 0, b = 0, c = 0;
  switch (m) { case 2: a = 1; break; case 4: b = 1; break;
               case 8: a = b = 1; break; case 16: a = b = c = 1; break; }
  digitalWrite(PIN_MS1, a); digitalWrite(PIN_MS2, b); digitalWrite(PIN_MS3, c);
}

void setDir(int d) {
  dir = d;
  digitalWrite(PIN_DIR, d > 0 ? HIGH : LOW);
  delayMicroseconds(2);                     // A4988: 200 ns Setup-Zeit
}

void startMotion() {
  if (mode == IDLE) { v = 0; nextUs = micros(); }
}

void finish() {
  mode = IDLE; v = 0;
  Serial.printf("DONE %.3f\n", posDeg());
}

void status() {
  const char* mn = mode == IDLE ? "IDLE" : mode == MOVE ? "MOVE" : "RUN";
  Serial.printf("OK en=%d ms=%d mode=%s pos=%ld deg=%.3f v=%.2f°/s speed=%.2f acc=%.2f steps/rev=%.1f\n",
                enabled, ms, mn, pos, posDeg(), v * dir / stepsPerDeg(), speedDeg, accDeg,
                stepsPerDeg() * 360.0f);
}

void handle(String s) {
  s.trim(); s.toUpperCase();
  if (!s.length()) return;
  int sp = s.indexOf(' ');
  String cmd = sp < 0 ? s : s.substring(0, sp);
  String arg = sp < 0 ? "" : s.substring(sp + 1);
  bool hasArg = arg.length() > 0;
  float x = arg.toFloat();

  if (cmd == "STATUS") { status(); return; }
  if (cmd == "EN" && hasArg) {
    enabled = x != 0;
    digitalWrite(PIN_EN, enabled ? LOW : HIGH);
    if (!enabled) { mode = IDLE; v = 0; }
    Serial.printf("OK EN %d\n", enabled); return;
  }
  if (cmd == "MS" && hasArg) {
    int m = (int)x;
    if (m != 1 && m != 2 && m != 4 && m != 8 && m != 16) { Serial.println("ERR MS 1|2|4|8|16"); return; }
    if (mode != IDLE) { Serial.println("ERR erst STOP"); return; }
    float d = posDeg();
    ms = m; setMsPins(ms); updateKinematics();
    pos = lroundf(d * stepsPerDeg());
    Serial.printf("OK MS %d (%.1f Schritte/U Kopf)\n", ms, stepsPerDeg() * 360.0f); return;
  }
  if (cmd == "SPEED" && hasArg && x > 0) { speedDeg = x; updateKinematics(); Serial.printf("OK SPEED %.3f\n", x); return; }
  if (cmd == "ACC" && hasArg && x > 0) { accDeg = x; updateKinematics(); Serial.printf("OK ACC %.3f\n", x); return; }
  if ((cmd == "MOVE" || cmd == "GOTO") && hasArg) {
    if (!enabled) { Serial.println("ERR EN 1 fehlt"); return; }
    long t = lroundf(x * stepsPerDeg());
    if (cmd == "MOVE") t += (mode == MOVE ? target : pos);
    if (mode == RUN) { Serial.println("ERR erst STOP"); return; }
    startMotion(); target = t; mode = MOVE;
    Serial.printf("OK %s -> %.3f\n", cmd.c_str(), target / stepsPerDeg()); return;
  }
  if (cmd == "RUN" && hasArg) {
    if (!enabled) { Serial.println("ERR EN 1 fehlt"); return; }
    if (mode == MOVE) { Serial.println("ERR erst STOP"); return; }
    startMotion(); runV = x * stepsPerDeg(); mode = RUN;
    Serial.printf("OK RUN %.3f\n", x); return;
  }
  if (cmd == "STOP") {
    if (mode == MOVE) {                     // Zielpunkt auf kürzesten Bremsweg legen
      long brake = (long)ceilf(v * v / (2 * acc));
      target = pos + dir * brake;
    } else if (mode == RUN) runV = 0;
    Serial.println("OK STOP"); return;
  }
  if (cmd == "ZERO") {
    if (mode != IDLE) { Serial.println("ERR erst STOP"); return; }
    pos = 0; Serial.println("OK ZERO"); return;
  }
  if (cmd == "REP" && hasArg) { repMs = (uint32_t)x; Serial.printf("OK REP %lu\n", (unsigned long)repMs); return; }
  Serial.println("ERR unbekannt: STATUS EN MS SPEED ACC MOVE GOTO RUN STOP ZERO REP");
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { if (line.length()) handle(line); line = ""; }
    else if (line.length() < 64) line += c;
  }
}

void stepper() {
  if (mode == IDLE || !enabled) return;
  uint32_t now = micros();
  if ((int32_t)(now - nextUs) < 0) return;

  int wantDir; float vTarget;
  if (mode == MOVE) {
    long rem = target - pos;
    if (rem == 0) { finish(); return; }
    wantDir = rem > 0 ? 1 : -1;
    float brake = v * v / (2 * acc);
    vTarget = (labs(rem) <= brake + 1) ? 0 : spd;
  } else {
    wantDir = runV >= 0 ? 1 : -1;
    vTarget = fabsf(runV);
    if (vTarget == 0 && v <= vmin) { finish(); return; }
  }
  if (wantDir != dir) {
    if (v <= vmin) setDir(wantDir);
    else vTarget = 0;                       // erst abbremsen, dann umkehren
  }

  digitalWrite(PIN_STEP, HIGH);
  delayMicroseconds(3);
  digitalWrite(PIN_STEP, LOW);
  pos += dir;

  if (v < vTarget)      { v = sqrtf(v * v + 2 * acc); if (v > vTarget) v = vTarget; }
  else if (v > vTarget) { v = sqrtf(fmaxf(v * v - 2 * acc, 0)); if (v < vTarget) v = vTarget; }
  float floorV = vTarget > 0 ? fminf(vmin, vTarget) : vmin;
  if (v < floorV) v = floorV;

  nextUs += (uint32_t)(1e6f / v);
  if ((int32_t)(now - nextUs) > 20000) nextUs = now;   // nach Blockade nicht nachholen
}

void setup() {
  pinMode(PIN_EN, OUTPUT);  digitalWrite(PIN_EN, HIGH);  // zuerst: Treiber aus
  pinMode(PIN_STEP, OUTPUT); digitalWrite(PIN_STEP, LOW);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_MS1, OUTPUT); pinMode(PIN_MS2, OUTPUT); pinMode(PIN_MS3, OUTPUT);
  setMsPins(ms); setDir(1); updateKinematics();
  Serial.begin(115200);
  delay(200);
  Serial.println("PiLiDAR-Stepper bereit. STATUS / EN 1 / MOVE 90 ...");
}

void loop() {
  readSerial();
  stepper();
  if (repMs && millis() - lastRep >= repMs) {
    lastRep = millis();
    Serial.printf("P %lu %ld %.3f\n", (unsigned long)micros(), pos, posDeg());
  }
}
