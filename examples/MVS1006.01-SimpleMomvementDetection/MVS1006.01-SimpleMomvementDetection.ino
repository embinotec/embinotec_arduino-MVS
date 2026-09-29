/*
 * MVS1006.01 Mikro-Vibrationssensor – Vibrations- und Lageauswertung
 * ------------------------------------------------------------------
 * Schaltung:
 *   ENABLE (Pin 4) -> EMD22 -> 3,3V -> R1 (1..10 MOhm) -> SENSE -> MVS1006.01 -> GND
 *   SENSE  (Pin 2) = Digitaleingang OHNE internen Pull-up, interruptfähig
 *
 * Pegel an SENSE:
 *   Kontakt geschlossen (Kugel liegt auf den Kontakten) -> LOW
 *   Kontakt offen                                       -> HIGH (über R1)
 *
 * Funktionsprinzip:
 *   1) Jede Flanke an SENSE wird per Interrupt mit micros() in einen Ringpuffer geschrieben.
 *   2) Flanken mit Abstand < BURST_GAP_US gehören zu einem "Burst" (eine Erschütterung).
 *      Dauer, Flankenanzahl und Flankendichte des Bursts werden klassifiziert:
 *      Klopfen, Objekt abgelegt, Tür zugeschlagen, Hinsetzen, Dauervibration.
 *   3) Klopfer werden zu einem Rhythmus zusammengefasst und mit einem Muster verglichen.
 *   4) Nach einer Ruhephase wird der statische Kontaktzustand gelesen -> Lage
 *      (Kugel liegt auf Kontakten = z.B. horizontal, sonst vertikal/anders).
 *
 * WICHTIG (laut Application Note): Der Sensor ist in Ruhe nur zu ca. 70..99 % der Zeit
 * geschlossen. Deshalb wird die Lage erst nach Ruhephase UND mehrfach bestätigt gewertet.
 * Alle Schwellwerte sind Startwerte und müssen mit dem realen Board (Debug 'd') angepasst werden.
 *
 * Serielle Befehle (115200 Baud):
 *   h = aktuelle Lage als "horizontal" einlernen
 *   v = aktuelle Lage als "vertikal" einlernen
 *   d = Debug-Ausgabe der Flankenzeiten pro Burst ein/aus
 */

#include <Arduino.h>

#if defined(ESP32) || defined(ESP8266)
  #define ISR_ATTR IRAM_ATTR
#else
  #define ISR_ATTR
#endif

// ---------------- Hardware ----------------
const uint8_t PIN_ENABLE = 4;
const uint8_t PIN_SENSE  = 2;          // Uno/Nano: 2 oder 3
const bool ENABLE_ACTIVE_HIGH = true;  // false, falls EMD22-Ansteuerung invertiert ist

// ---------------- Parameter ----------------
const uint32_t BURST_GAP_US      = 40000UL;  // Pause > 40 ms beendet einen Burst
const uint8_t  MIN_EDGES         = 2;        // weniger Flanken = Zucken, wird ignoriert
const uint8_t  MAX_EDGES         = 96;       // gespeicherte Flanken pro Burst (für Debug/Decay)

// Klassifikation (Dauer in ms, Anzahl Flanken)
const uint16_t KNOCK_MAX_MS      = 60;
const uint16_t OBJECT_MAX_MS     = 250;
const uint16_t OBJECT_MAX_EDGES  = 30;
const uint16_t DOOR_MAX_MS       = 900;
const uint16_t DOOR_MIN_EDGES    = 30;
const uint16_t CHAIR_MIN_MS      = 250;

// Klopfrhythmus
const uint8_t  MAX_KNOCKS        = 8;
const uint16_t RHYTHM_TIMEOUT_MS = 1200;     // Ende des Rhythmus, wenn so lange kein Klopfer
const uint16_t SECRET_INTERVALS[] = {300, 300, 600};  // Beispiel: 4 Klopfer (3 Abstände in ms)
const uint8_t  SECRET_LEN        = sizeof(SECRET_INTERVALS) / sizeof(SECRET_INTERVALS[0]);
const uint8_t  RHYTHM_TOL_PCT    = 35;       // Toleranz pro Abstand
const uint16_t RHYTHM_TOL_MIN_MS = 60;

// Lage
const uint16_t ORIENT_QUIET_MS   = 800;      // so lange keine Flanke, bevor Lage gelesen wird
const uint16_t ORIENT_CHECK_MS   = 400;      // Abstand der Lage-Messungen
const uint8_t  ORIENT_CONFIRM    = 2;        // so viele gleiche Messungen in Folge nötig

// ---------------- Zustände ----------------
bool closedMeansHorizontal = true;  // Zuordnung: Kontakt geschlossen = horizontal (per 'h'/'v' änderbar)
bool debugDump = false;

enum Lage : uint8_t { LAGE_UNBEKANNT, LAGE_HORIZONTAL, LAGE_VERTIKAL };
enum Ereignis : uint8_t { EV_ZUCKEN, EV_KLOPFEN, EV_OBJEKT, EV_TUER, EV_HINSETZEN, EV_DAUER };

// ---------------- Flanken-Ringpuffer (ISR -> loop) ----------------
const uint8_t EV_BUF = 128;
volatile uint32_t evT[EV_BUF];
volatile uint8_t  evHead = 0;
volatile uint8_t  evTail = 0;
volatile bool     evOverflow = false;

void ISR_ATTR isrSense() {
  uint8_t next = (evHead + 1) % EV_BUF;
  if (next != evTail) {
    evT[evHead] = micros();
    evHead = next;
  } else {
    evOverflow = true;
  }
}

// ---------------- Burst-Daten ----------------
struct Burst {
  bool     active = false;
  uint32_t start = 0, last = 0;
  uint16_t n = 0;
  uint32_t t[MAX_EDGES];
} burst;

uint32_t lastEdgeMs = 0;

// Klopfrhythmus
uint32_t knockMs[MAX_KNOCKS];
uint8_t  knockCount = 0;
uint32_t lastKnockMs = 0;

// Lage
Lage     lage = LAGE_UNBEKANNT;
Lage     candLage = LAGE_UNBEKANNT;
uint8_t  candCount = 0;
uint32_t lastOrientCheckMs = 0;

// ---------------- Hilfsfunktionen ----------------
inline bool contactClosed() { return digitalRead(PIN_SENSE) == LOW; }

void sensorEnable(bool on) {
  digitalWrite(PIN_ENABLE, (on == ENABLE_ACTIVE_HIGH) ? HIGH : LOW);
}

const char* ereignisName(Ereignis e) {
  switch (e) {
    case EV_ZUCKEN:    return "Zucken (ignoriert)";
    case EV_KLOPFEN:   return "Klopfer";
    case EV_OBJEKT:    return "Gegenstand abgelegt";
    case EV_TUER:      return "Tuer zugeschlagen";
    case EV_HINSETZEN: return "Hinsetzen / weiche Last";
    default:           return "Dauervibration";
  }
}

Ereignis klassifiziere(uint16_t durMs, uint16_t edges) {
  if (edges < MIN_EDGES)                                          return EV_ZUCKEN;
  if (durMs <= KNOCK_MAX_MS)                                      return EV_KLOPFEN;
  if (durMs <= OBJECT_MAX_MS && edges <= OBJECT_MAX_EDGES)        return EV_OBJEKT;
  if (durMs <= DOOR_MAX_MS && edges > DOOR_MIN_EDGES)             return EV_TUER;
  if (durMs >= CHAIR_MIN_MS && durMs <= DOOR_MAX_MS)              return EV_HINSETZEN;
  return EV_DAUER;
}

// ---------------- Burst-Verarbeitung ----------------
void onEdge(uint32_t t) {
  lastEdgeMs = millis();
  if (!burst.active) {
    burst.active = true;
    burst.n = 0;
    burst.start = t;
  }
  if (burst.n < MAX_EDGES) burst.t[burst.n] = t;
  burst.n++;
  burst.last = t;
}

void drainEvents() {
  while (evTail != evHead) {
    uint32_t t = evT[evTail];
    evTail = (evTail + 1) % EV_BUF;
    onEdge(t);
  }
  if (evOverflow) {
    evOverflow = false;
    Serial.println(F("! Flanken-Puffer uebergelaufen"));
  }
}

void finishBurst() {
  burst.active = false;
  uint32_t durUs = burst.last - burst.start;
  uint16_t durMs = durUs / 1000UL;
  uint16_t edges = burst.n;

  Ereignis e = klassifiziere(durMs, edges);
  if (e == EV_ZUCKEN) return;   // Kugel-Wackeln im Ruhezustand nicht melden

  // Abklingverhalten: Anteil der Flanken in der ersten Burst-Hälfte
  uint16_t stored = min<uint16_t>(edges, MAX_EDGES);
  uint16_t firstHalf = 0;
  for (uint16_t i = 0; i < stored; i++) {
    if ((burst.t[i] - burst.start) < durUs / 2) firstHalf++;
  }
  uint8_t decayPct = stored ? (uint8_t)((uint32_t)firstHalf * 100UL / stored) : 0;

  Serial.print(F("[Burst] "));
  Serial.print(durMs);  Serial.print(F(" ms, "));
  Serial.print(edges);  Serial.print(F(" Flanken, 1. Haelfte "));
  Serial.print(decayPct); Serial.print(F(" % -> "));
  Serial.println(ereignisName(e));

  if (debugDump) {
    Serial.print(F("  t[us]: "));
    for (uint16_t i = 0; i < stored; i++) {
      Serial.print(burst.t[i] - burst.start);
      Serial.print(' ');
    }
    Serial.println();
  }

  if (e == EV_KLOPFEN) {
    uint32_t nowMs = millis();
    if (knockCount < MAX_KNOCKS) knockMs[knockCount++] = nowMs;
    lastKnockMs = nowMs;
  }
}

// ---------------- Klopfrhythmus ----------------
void evaluateRhythm() {
  if (knockCount == 0) return;
  if (millis() - lastKnockMs < RHYTHM_TIMEOUT_MS) return;

  Serial.print(F("[Rhythmus] "));
  Serial.print(knockCount);
  Serial.print(F(" Klopfer"));

  if (knockCount > 1) {
    Serial.print(F(", Abstaende [ms]:"));
    for (uint8_t i = 1; i < knockCount; i++) {
      Serial.print(' ');
      Serial.print(knockMs[i] - knockMs[i - 1]);
    }
  }

  // Vergleich mit Geheimmuster
  bool match = (knockCount == SECRET_LEN + 1);
  for (uint8_t i = 0; match && i < SECRET_LEN; i++) {
    int32_t d   = (int32_t)(knockMs[i + 1] - knockMs[i]);
    int32_t ref = SECRET_INTERVALS[i];
    int32_t tol = max<int32_t>(RHYTHM_TOL_MIN_MS, ref * RHYTHM_TOL_PCT / 100);
    if (abs(d - ref) > tol) match = false;
  }
  Serial.println(match ? F("  -> MUSTER ERKANNT") : F(""));
  knockCount = 0;
}

// ---------------- Lage ----------------
void updateOrientation() {
  if (burst.active) return;
  uint32_t nowMs = millis();
  if (nowMs - lastEdgeMs < ORIENT_QUIET_MS) return;
  if (nowMs - lastOrientCheckMs < ORIENT_CHECK_MS) return;
  lastOrientCheckMs = nowMs;

  bool closed = contactClosed();
  Lage cand = (closed == closedMeansHorizontal) ? LAGE_HORIZONTAL : LAGE_VERTIKAL;

  if (cand == lage) { candCount = 0; return; }
  if (cand == candLage) candCount++; else { candLage = cand; candCount = 1; }

  if (candCount >= ORIENT_CONFIRM) {
    lage = cand;
    candCount = 0;
    Serial.print(F("[Lage] "));
    Serial.print(lage == LAGE_HORIZONTAL ? F("HORIZONTAL") : F("VERTIKAL"));
    Serial.print(F("  (Kontakt "));
    Serial.print(closed ? F("geschlossen)") : F("offen)"));
    Serial.println();
  }
}

// ---------------- Serielle Bedienung ----------------
void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'h' || c == 'v') {
      bool closed = contactClosed();
      closedMeansHorizontal = (c == 'h') ? closed : !closed;
      lage = LAGE_UNBEKANNT; candCount = 0;
      Serial.print(F("Eingelernt: Kontakt "));
      Serial.print(closed ? F("geschlossen") : F("offen"));
      Serial.println(c == 'h' ? F(" = horizontal") : F(" = vertikal"));
    } else if (c == 'd') {
      debugDump = !debugDump;
      Serial.println(debugDump ? F("Debug an") : F("Debug aus"));
    }
  }
}

// ---------------- Arduino ----------------
void setup() {
  Serial.begin(115200);
  pinMode(PIN_ENABLE, OUTPUT);
  pinMode(PIN_SENSE, INPUT);          // kein Pull-up: R1 (MOhm) bildet den Spannungsteiler
  sensorEnable(true);
  delay(5);                           // Einschwingen (R1 * Pin-/Leitungskapazität)

  noInterrupts();
  evHead = evTail = 0;
  interrupts();
  attachInterrupt(digitalPinToInterrupt(PIN_SENSE), isrSense, CHANGE);

  lastEdgeMs = millis();
  Serial.println(F("MVS1006.01 bereit. Befehle: h / v = Lage einlernen, d = Debug"));
}

void loop() {
  drainEvents();

  if (burst.active &&
      (uint32_t)(micros() - burst.last) > BURST_GAP_US &&
      evTail == evHead) {
    finishBurst();
  }

  evaluateRhythm();
  updateOrientation();
  handleSerial();
}
