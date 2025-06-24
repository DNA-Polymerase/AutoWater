/**
 * @file capacitive_soil_moisture_sensor_controller.ino
 * @brief Arduino-Code zur Steuerung einer automatischen Pflanzenbewässerung
 * basierend auf einem kapazitiven Feuchtigkeitssensor (invers: niedriger Wert = feucht), mit RTC,
 * LCD-Display, Tankfüllstandsmessung und manueller Steuerung.
 *
 * Funktionen:
 * - Automatische Bewässerung basierend auf Bodentrockenheit.
 * - 30-stündige Trockensperre nach Trockenheitserkennung (verhindert Überwässerung).
 * - Speicherung des Trockenzeit-Starts im RTC NVRAM (überlebt Neustarts).
 * - Manuelles Pumpen per Knopfdruck (langer Druck) mit Anzeige der gepumpten Menge (ml)
 * und sekündlicher Aktualisierung der Feuchtigkeitsmessung währenddessen.
 * - Dynamisches Messintervall (kurz nach Start/Änderung, dann länger).
 * - Anzeige von Uhrzeit, Feuchtigkeit, Tankstatus und verbleibender Trockenzeit auf LCD.
 * - LCD-Hintergrundbeleuchtung mit Timeout.
 * - Tankfüllstandsüberwachung (verhindert Pumpen bei leerem Tank).
 * - Bugfix: Verhindert ungewolltes Pumpen direkt nach Systemstart.
 */

// ======================
// 📚 BIBLIOTHEKEN
// ======================
#include <Wire.h>             // Für I2C Kommunikation (LCD, RTC)
#include <RTClib.h>           // Für Real-Time Clock (DS1307)
#include <LiquidCrystal_I2C.h> // Für I2C LCD Display

// ======================
// ⏰ RTC INITIALISIERUNG
// ======================
RTC_DS1307 rtc; // Real-Time Clock Objekt erstellen

// ======================
// 🔧 EINSTELLBARE WERTE (Konstanten)
// ======================
// --- Sensor & Schwellwerte ---
// *** WICHTIG: Dieser Code ist für Sensoren, bei denen NIEDRIGERE Werte FEUCHTERE Erde bedeuten! ***
const int TROCKEN_SCHWELLE = 300;     // Sensorwert, ÜBER dem der Boden als trocken gilt.
const int MIN_TANK_WERT = 0;          // Minimaler Sensorwert für den Wassertank (Anpassen, falls Sensor anders reagiert oder leer/voll vertauscht ist)
const unsigned long MESSDAUER_MS = 100; // Kurze Verzögerung (ms) nach dem Einschalten der Sensoren vor dem Lesen (Stabilisierung)

// --- Bewässerung & Pumpe ---
const int GIESS_MENGE_ML = 100;          // Wassermenge (in ml), die pro Gießvorgang abgegeben wird
const float ML_PRO_MINUTE = 115.0;       // Durchflussrate der Pumpe (ml pro Minute) - WICHTIG: KALIBRIEREN!
const int TROCKEN_SPERRE_H = 30;         // Erforderliche Trockenperiode (in Stunden), bevor erneut automatisch gegossen wird

// --- Zeit & Intervalle ---
const float INITIAL_MESSINTERVALL_SEC = 2.0;   // Anfängliches Intervall zwischen Sensorlesungen (Sekunden)
const float LONG_MESSINTERVALL_SEC = 300.0; // Intervall nach mehreren anfänglichen Messungen (Sekunden) -> 5 Minuten
const unsigned int INITIAL_MEASUREMENT_THRESHOLD = 15; // Anzahl der anfänglichen Messungen, bevor zum langen Intervall gewechselt wird
const unsigned long LONG_PRESS_DURATION_MS = 1000; // Dauer (ms) für einen langen Tastendruck (manuelles Pumpen)
const unsigned long BACKLIGHT_TIMEOUT_MS = 60000UL; // Timeout für LCD-Hintergrundbeleuchtung (ms) -> 1 Minute

// ======================
// 🔌 PIN-BELEGUNG
// ======================
const int SENSOR_BODEN_PIN = A0; // Analoger Pin für Bodenfeuchtigkeitssensor
const int SENSOR_BODEN_VCC = 7;  // Digitaler Pin zur Stromversorgung des Bodensensors (abschaltbar)
const int SENSOR_TANK_PIN = A1;  // Analoger Pin für Tankfüllstandssensor
const int SENSOR_TANK_VCC = 6;   // Digitaler Pin zur Stromversorgung des Tanksensors (abschaltbar)
const int PUMPE_PIN = 8;         // Digitaler Pin zur Steuerung des Pumpenrelais/Transistors
const int TASTER_PIN = 2;        // Digitaler Pin für den manuellen Steuerungstaster (mit internem Pull-up)

// ======================
// 📟 LCD & BERECHNETE KONSTANTEN
// ======================
LiquidCrystal_I2C lcd(0x27, 16, 2); // LCD initialisieren (Adresse 0x27, 16 Spalten, 2 Zeilen)

// --- Abgeleitete Konstanten (werden zur Laufzeit nicht verändert) ---
const unsigned long TROCKEN_SPERRE_MS = TROCKEN_SPERRE_H * 3600000UL; // Trockenperiode in Millisekunden (UL = Unsigned Long)
const float ML_PRO_MS = ML_PRO_MINUTE / 60000.0; // Durchflussrate in ml pro Millisekunde
const unsigned long PUMP_DAUER_MS = (unsigned long)(GIESS_MENGE_ML / ML_PRO_MS); // Berechnete Pumpdauer in ms für die Zielmenge

// ======================
// 💾 RTC NVRAM KONSTANTE
// ======================
const uint8_t RTC_TROCKEN_START_ADDR = 0; // Startadresse im RTC NVRAM zum Speichern der Trockenstartzeit (benötigt 6 Bytes)

// ======================
// 🌍 GLOBALE VARIABLEN (Zustandsvariablen)
// ======================
// --- Zeitmessung & Intervalle ---
unsigned long letzterMessZeitpunkt = 0;         // Zeitstempel (millis()) der letzten automatischen Sensormessung
unsigned long lastBacklightTime = 0;          // Zeitstempel (millis()) für LCD-Hintergrundbeleuchtungs-Timeout
unsigned long letzteMinutenZeit = 99;         // Speichert die letzte Minute, in der die Uhrzeit auf dem LCD aktualisiert wurde (Initialisierung mit ungültigem Wert)
float currentMessintervallSec = INITIAL_MESSINTERVALL_SEC; // Aktuelles Messintervall (kann wechseln)
unsigned int measurementCount = 0;            // Zähler für Messungen zur Anpassung des Intervalls

// --- Pumpensteuerung (Automatisch & Manuell) ---
unsigned long pumpStartzeit = 0;              // Zeitstempel (millis()), wann die *automatische* Pumpe gestartet wurde
bool pumpeAktiv = false;                      // Flag: Läuft gerade der *automatische* Pumpzyklus?
unsigned long manualPumpStartTimeMillis = 0;  // Zeitstempel (millis()), wann das *manuelle* Pumpen tatsächlich gestartet wurde (nach langem Druck)
bool longPressActive = false;                 // Flag: Ist das manuelle Pumpen (durch langen Druck) gerade aktiv?

// --- Tastersteuerung ---
unsigned long pressStartTime = 0;             // Zeitstempel (millis()), wann der Taster gedrückt wurde
bool buttonWasPressed = false;                // Flag: Wird der Taster gerade gehalten? (für Entprellung und Langerkennung)

// --- Zustandsflags & Sensorwerte ---
bool backlightStatus = true;                  // Aktueller Status der LCD-Hintergrundbeleuchtung (AN/AUS)
bool trockenzeitLaeuft = false;               // Flag: Läuft gerade der 30h Trockenzeit-Countdown?
int aktuelleFeuchtigkeit = 0;                 // Speichert den zuletzt gemessenen Bodenfeuchtigkeitswert
bool tankLeer = true;                         // Flag: Ist der Tank leer? (Initialisierung als leer = sicher)
unsigned long lastManualSoilReadTime = 0;     // Zeitstempel (millis()) der letzten Bodenmessung *während* des manuellen Pumpens
bool initialMeasurementDone = false; // *** NEU *** Flag, um sicherzustellen, dass die erste Messung erfolgt ist

// =========================================
// 💾 RTC NVRAM FUNKTIONEN (Speichern/Laden der Trockenstartzeit)
// =========================================

/**
 * @brief Speichert die übergebene DateTime-Struktur im NVRAM des RTC.
 * @param zeit Das zu speichernde DateTime-Objekt.
 */
void speichereTrockenzeitRTC(const DateTime &zeit) {
  if (!zeit.isValid()) {
    Serial.println(F("FEHLER: Versuch, ungültige Zeit in RTC zu speichern!"));
    return;
  }
  // Speichere Jahr (als Offset von 2000), Monat, Tag, Stunde, Minute, Sekunde
  rtc.writenvram(RTC_TROCKEN_START_ADDR + 0, zeit.year() - 2000);
  rtc.writenvram(RTC_TROCKEN_START_ADDR + 1, zeit.month());
  rtc.writenvram(RTC_TROCKEN_START_ADDR + 2, zeit.day());
  rtc.writenvram(RTC_TROCKEN_START_ADDR + 3, zeit.hour());
  rtc.writenvram(RTC_TROCKEN_START_ADDR + 4, zeit.minute());
  rtc.writenvram(RTC_TROCKEN_START_ADDR + 5, zeit.second());
  Serial.println(F("Trockenstartzeit im RTC gespeichert."));
}

/**
 * @brief Lädt die Trockenstartzeit aus dem NVRAM des RTC.
 * @return Ein DateTime-Objekt. Ist die gespeicherte Zeit ungültig, wird ein ungültiges DateTime zurückgegeben.
 */
DateTime ladeTrockenzeitRTC() {
  uint16_t jahr = rtc.readnvram(RTC_TROCKEN_START_ADDR + 0) + 2000;
  uint8_t monat = rtc.readnvram(RTC_TROCKEN_START_ADDR + 1);
  uint8_t tag = rtc.readnvram(RTC_TROCKEN_START_ADDR + 2);
  uint8_t stunde = rtc.readnvram(RTC_TROCKEN_START_ADDR + 3);
  uint8_t minute = rtc.readnvram(RTC_TROCKEN_START_ADDR + 4);
  uint8_t sekunde = rtc.readnvram(RTC_TROCKEN_START_ADDR + 5);

  // Plausibilitätsprüfung der geladenen Werte
  if (monat < 1 || monat > 12 || tag < 1 || tag > 31 || jahr < 2024 || jahr > 2050 || stunde > 23 || minute > 59 || sekunde > 59) {
    // Ungültige Daten im NVRAM gefunden
    return DateTime(); // Gibt ein ungültiges DateTime-Objekt zurück (isValid() == false)
  }
  return DateTime(jahr, monat, tag, stunde, minute, sekunde);
}

/**
 * @brief Setzt die gespeicherte Trockenstartzeit im NVRAM des RTC zurück (überschreibt mit Nullen).
 */
void resetTrockenzeitRTC() {
  for (int i = 0; i < 6; i++) {
    rtc.writenvram(RTC_TROCKEN_START_ADDR + i, 0); // Überschreibe alle 6 Bytes mit 0
  }
  Serial.println(F("Trockenstartzeit im RTC zurückgesetzt."));
}


// ======================
// 🚀 SETUP - Einmalige Initialisierung beim Start
// ======================
void setup() {
  Serial.begin(9600);
  delay(500); // Kurze Pause, damit der Serial Monitor verbunden werden kann
  Serial.println(F("\n--- Systemstart ---"));

  // --- Pin-Modi konfigurieren ---
  pinMode(PUMPE_PIN, OUTPUT);
  pinMode(SENSOR_BODEN_VCC, OUTPUT);
  pinMode(SENSOR_TANK_VCC, OUTPUT);
  pinMode(TASTER_PIN, INPUT_PULLUP); // Taster nutzt internen Pull-up-Widerstand

  // --- Ausgänge initial auf LOW setzen (Sicherheitszustand) ---
  digitalWrite(PUMPE_PIN, LOW);       // Pumpe aus
  digitalWrite(SENSOR_BODEN_VCC, LOW); // Bodensensor aus
  digitalWrite(SENSOR_TANK_VCC, LOW);  // Tanksensor aus
  Serial.println(F("Pins konfiguriert."));

  // --- LCD initialisieren ---
  lcd.init();
  lcd.backlight(); // Hintergrundbeleuchtung initial einschalten
  backlightStatus = true;
  lastBacklightTime = millis(); // Timer für Backlight starten
  lcd.clear();
  lcd.print(F("Starte System..."));
  Serial.println(F("LCD initialisiert."));
  delay(1000);

  // --- RTC initialisieren ---
  if (!rtc.begin()) {
    Serial.println(F("--- FEHLER: RTC nicht gefunden! System HÄNGT HIER. ---"));
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(F("FEHLER:"));
    lcd.setCursor(0,1);
    lcd.print(F("RTC nicht gef.!"));
    while (1) delay(100); // Endlosschleife, da RTC essentiell ist
  }
  Serial.println(F("RTC initialisiert."));

  // --- RTC Zeit prüfen und ggf. setzen / NVRAM prüfen ---
  if (!rtc.isrunning()) {
    Serial.println(F("WARNUNG: RTC lief nicht! Setze Zeit auf Kompilierungszeit..."));
    lcd.clear();
    lcd.print(F("RTC Zeit gesetzt"));
    // Setzt die RTC auf das Datum und die Uhrzeit, zu der der Sketch kompiliert wurde.
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    delay(1000);
    resetTrockenzeitRTC(); // Wenn die Zeit neu gesetzt wurde, ist eine alte Trockenzeit ungültig
    trockenzeitLaeuft = false;
  } else {
    Serial.println(F("RTC läuft. Prüfe gespeicherte Trockenstartzeit..."));
    DateTime gespeicherterStart = ladeTrockenzeitRTC();
    if (gespeicherterStart.isValid()) {
        // Gültige Zeit gefunden, Trockenzeit-Timer könnte noch laufen
        trockenzeitLaeuft = true;
        Serial.println(F("Gültige Trockenstartzeit aus RTC geladen."));
        // Die tatsächliche Prüfung, ob die Zeit abgelaufen ist, erfolgt im Loop
    } else {
        // Keine gültige Zeit im NVRAM
        trockenzeitLaeuft = false;
        Serial.println(F("Keine gültige Trockenstartzeit im RTC gefunden."));
    }
  }

  // --- Start abgeschlossen ---
  lcd.clear();
  lcd.print(F("System bereit."));
  Serial.println(F("System bereit."));
  delay(1000);

  // Initialisiere letzte Aktualisierungszeit der LCD-Uhr, um sofortige Anzeige zu erzwingen
  letzteMinutenZeit = 99; // Ungültiger Wert, damit die Uhr beim ersten Loop-Durchlauf aktualisiert wird
  initialMeasurementDone = false; // Sicherstellen, dass Flag bei Start false ist
}

// =========================================
// 🛠️ HILFSFUNKTIONEN
// =========================================

/**
 * @brief Liest den analogen Wert des Bodenfeuchtigkeitssensors.
 * Schaltet den Sensor kurz ein, liest den Wert und schaltet ihn wieder aus.
 * @return Der rohe Analogwert des Sensors (0-1023).
 */
int leseBodenFeuchtigkeit() {
  digitalWrite(SENSOR_BODEN_VCC, HIGH); // Sensor einschalten
  delay(MESSDAUER_MS);                  // Kurze Wartezeit zur Stabilisierung
  int feuchtigkeit = analogRead(SENSOR_BODEN_PIN); // Wert lesen
  digitalWrite(SENSOR_BODEN_VCC, LOW);  // Sensor ausschalten (spart Strom, erhöht Lebensdauer)
  return feuchtigkeit;
}

/**
 * @brief Liest den analogen Wert des Tankfüllstandssensors.
 * Schaltet den Sensor kurz ein, liest den Wert und schaltet ihn wieder aus.
 * @return Der rohe Analogwert des Sensors (0-1023).
 */
int leseTankFuellstand() {
  digitalWrite(SENSOR_TANK_VCC, HIGH); // Sensor einschalten
  delay(MESSDAUER_MS);                 // Kurze Wartezeit zur Stabilisierung
  int tankWert = analogRead(SENSOR_TANK_PIN); // Wert lesen
  digitalWrite(SENSOR_TANK_VCC, LOW);  // Sensor ausschalten
  return tankWert;
}


/**
 * @brief Berechnet und zeigt die verbleibende Trockenzeit auf dem LCD an.
 * Aktualisiert nur den Bereich für die Trockenzeit in der zweiten Zeile.
 * @param jetztZeit Die aktuelle Uhrzeit als DateTime-Objekt.
 */
void displayTrockenzeit(DateTime jetztZeit) {
  lcd.setCursor(9, 1); // Cursor auf Position für Trockenzeit setzen (Zeile 1, Spalte 9)
  if (trockenzeitLaeuft) {
    DateTime start = ladeTrockenzeitRTC();
    if (!start.isValid()) {
        // Sollte nicht passieren, wenn Logik stimmt, aber als Sicherheitsnetz
        lcd.print(F("RTC Err "));
        Serial.println(F("FEHLER: Ungültige Trockenstartzeit beim Anzeigen gelesen."));
        trockenzeitLaeuft = false; // Zustand korrigieren
        resetTrockenzeitRTC();     // NVRAM zurücksetzen
        return;
    }
    // Berechne das Ende der Trockenperiode
    DateTime ende = start + TimeSpan(TROCKEN_SPERRE_MS / 1000UL); // TimeSpan erwartet Sekunden
    TimeSpan verbleibend = ende - jetztZeit; // Berechne die verbleibende Zeit

    if (verbleibend.totalseconds() <= 0) {
        // Zeit ist abgelaufen oder Startzeit war ungültig/in der Zukunft
        lcd.print(F("Bereit ")); // Zeigt an, dass gegossen werden könnte
    } else {
        // Zeit läuft noch, zeige verbleibende h:mm an
        long verbleibendeStunden = verbleibend.totalseconds() / 3600;
        long verbleibendeMinuten = (verbleibend.totalseconds() % 3600) / 60;
        // Formatierte Ausgabe (z.B. " 9h05m" oder "12h30m")
        if (verbleibendeStunden < 10) lcd.print(F(" ")); // Führendes Leerzeichen für einstellige Stunden
        lcd.print(verbleibendeStunden);
        lcd.print(F("h"));
        if (verbleibendeMinuten < 10) lcd.print(F("0")); // Führende Null für einstellige Minuten
        lcd.print(verbleibendeMinuten);
        lcd.print(F("m"));
    }
  } else {
    // Trockenzeit-Timer ist nicht aktiv
    lcd.print(F("--h--m ")); // Platzhalter anzeigen
  }
}

/**
 * @brief Aktualisiert die Hauptanzeige auf dem LCD (Zeile 0 und Zeile 1).
 * Wird aufgerufen bei Zustandsänderungen oder nach Messungen.
 * Beachtet den aktuellen Systemzustand (Normal, Pumpe aktiv, Tank leer).
 * Ruft displayTrockenzeit() auf, um die zweite Zeile zu vervollständigen.
 */
void updateLCD() {
  // Wenn manuelles Pumpen aktiv ist, wird das Display separat in der loop() aktualisiert.
  if (longPressActive) {
      return;
  }

  // Aktuelle Uhrzeit holen
  DateTime jetzt = rtc.now();
  char uhrzeitStr[6]; // Buffer für "HH:MM\0"
  sprintf(uhrzeitStr, "%02d:%02d", jetzt.hour(), jetzt.minute());

  // Merken, dass die Uhrzeit für diese Minute aktualisiert wurde
  letzteMinutenZeit = jetzt.minute();

  // --- LCD löschen und Zeile 0 füllen ---
  lcd.clear();
  lcd.setCursor(0, 0); // Cursor an Anfang Zeile 0

  if (tankLeer) {
      lcd.print(F("Tank leer!      ")); // Füllt die Zeile mit Leerzeichen auf
  } else if (pumpeAktiv) {
      // Zeige Fortschritt des automatischen Pumpvorgangs an
      unsigned long elapsedPumpTimeSec = (millis() - pumpStartzeit) / 1000;
      unsigned long totalPumpTimeSec = PUMP_DAUER_MS / 1000;
      // Sicherstellen, dass die angezeigte Zeit nicht über die Gesamtzeit hinausgeht
      if (elapsedPumpTimeSec > totalPumpTimeSec) elapsedPumpTimeSec = totalPumpTimeSec;
      lcd.print(F("Pumpe:"));
      lcd.print(elapsedPumpTimeSec);
      lcd.print(F("/"));
      lcd.print(totalPumpTimeSec);
      lcd.print(F("s   ")); // Füllt auf
  } else {
      // Normalbetrieb: Zeige Feuchtigkeitsstatus und Wert an
      bool istTrocken = aktuelleFeuchtigkeit > TROCKEN_SCHWELLE;
      lcd.print(istTrocken ? F("Trocken ") : F("Feucht  ")); // Feste Breite für Konsistenz
      lcd.print(aktuelleFeuchtigkeit);
      // Füge Leerzeichen hinzu, um alte Zahlen zu überschreiben
      if(aktuelleFeuchtigkeit < 10) lcd.print(F("    "));
      else if (aktuelleFeuchtigkeit < 100) lcd.print(F("   "));
      else if (aktuelleFeuchtigkeit < 1000) lcd.print(F("  "));
      else lcd.print(F(" "));
  }

  // --- Zeile 1 füllen ---
  lcd.setCursor(0, 1); // Cursor an Anfang Zeile 1
  lcd.print(uhrzeitStr); // "HH:MM"
  lcd.print(F(" | T:")); // Trenner und Label für Trockenzeit
  displayTrockenzeit(jetzt); // Ruft die Funktion auf, um den Rest der Zeile zu füllen
}


// ======================
// 🔄 HAUPTSCHLEIFE (loop)
// ======================
void loop() {
  // Aktuelle Zeit und Tasterstatus am Anfang des Loops abfragen
  unsigned long currentMillis = millis();
  bool buttonIsPressed = (digitalRead(TASTER_PIN) == LOW); // LOW bedeutet gedrückt (wegen INPUT_PULLUP)
  DateTime jetztZeit = rtc.now(); // Aktuelle Zeit von der RTC holen

  // ----------------------------------------------------
  // --- 1. Tasterlogik (Drücken, Halten, Loslassen) ---
  // ----------------------------------------------------

  // --- Taster gerade gedrückt (Flankenerkennung) ---
  if (buttonIsPressed && !buttonWasPressed) {
    pressStartTime = currentMillis; // Startzeit des Drückens merken
    buttonWasPressed = true;        // Merker setzen, dass der Taster gehalten wird

    // LCD-Backlight bei Tastendruck einschalten (falls aus) und Timer zurücksetzen
    lastBacklightTime = currentMillis;
    if (!backlightStatus) {
      lcd.backlight();
      backlightStatus = true;
    }

    // Kurzer Druck: Messintervall zurücksetzen (falls es lang war)
    // Dies geschieht *bevor* der lange Druck erkannt wird.
    if (!longPressActive && currentMessintervallSec != INITIAL_MESSINTERVALL_SEC) {
        currentMessintervallSec = INITIAL_MESSINTERVALL_SEC;
        measurementCount = 0;      // Zähler zurücksetzen
        letzterMessZeitpunkt = 0; // Erzwingt sofortige Messung im nächsten Durchlauf
        Serial.println(F("Messintervall durch kurzen Tasterdruck zurückgesetzt."));
        // Kurze Info auf dem LCD anzeigen
        if (backlightStatus) {
            lcd.setCursor(0,0);
            lcd.print(F("Messint: Sofort "));
            delay(1000); // Kurz anzeigen
            updateLCD(); // Normales Display wiederherstellen
        }
    }
  }

  // --- Langer Druck wird gehalten (Manuelles Pumpen starten) ---
  if (buttonIsPressed && buttonWasPressed) {
    // Prüfen, ob die Dauer für langen Druck erreicht wurde UND manuelles Pumpen noch nicht aktiv ist
    if (!longPressActive && (currentMillis - pressStartTime >= LONG_PRESS_DURATION_MS)) {
        longPressActive = true; // Manuelles Pumpen aktivieren
        manualPumpStartTimeMillis = currentMillis; // Startzeit des *Pumpens* merken
        pumpeAktiv = false; // Sicherstellen, dass der automatische Modus deaktiviert ist
        digitalWrite(PUMPE_PIN, HIGH); // Pumpe einschalten
        Serial.println(F("Langer Druck erkannt - Manuelle Pumpe AN"));

        // Sofort aktuelle Sensorwerte lesen für die Anzeige
        aktuelleFeuchtigkeit = leseBodenFeuchtigkeit();
        tankLeer = leseTankFuellstand() < MIN_TANK_WERT;
        lastManualSoilReadTime = currentMillis; // Initialisiere den Timer für die sekündliche Messung

        // Spezielle Anzeige für manuelles Pumpen auf LCD
        lcd.clear();
        lcd.setCursor(0, 0);
        if (tankLeer) {
            lcd.print(F("Tank leer! MPUMP"));
        } else {
            lcd.print(F("Man. Pumpe EIN "));
        }
        // Zeile 1 initialisieren (wird gleich im nächsten Block aktualisiert)
        lcd.setCursor(0, 1);
        char manualPumpDisplay[17]; // Buffer für "S:xxx   P:yyyml\0"
        sprintf(manualPumpDisplay, "S:%-3d   P:%3dml", aktuelleFeuchtigkeit, 0); // Starte mit 0ml
        manualPumpDisplay[16] = '\0'; // Nullterminierung sicherstellen
        lcd.print(manualPumpDisplay);

        // Backlight sicherstellen (sollte schon an sein, aber zur Sicherheit)
        lastBacklightTime = currentMillis;
        if (!backlightStatus) {
          lcd.backlight();
          backlightStatus = true;
        }
    }
  }

  // --- Taster losgelassen ---
  if (!buttonIsPressed && buttonWasPressed) {
    buttonWasPressed = false; // Merker zurücksetzen

    // Wenn Taster losgelassen wird, während manuelles Pumpen aktiv war -> Stoppen
    if (longPressActive) {
      longPressActive = false; // Manuelles Pumpen beenden
      digitalWrite(PUMPE_PIN, LOW); // Pumpe ausschalten
      unsigned long pumpingDurationMs = currentMillis - manualPumpStartTimeMillis;
      int pumpedMl = (int)(pumpingDurationMs * ML_PRO_MS); // Berechne gepumpte Menge

      Serial.print(F("Taster losgelassen - Manuelle Pumpe AUS. Gepumpt: ca. "));
      Serial.print(pumpedMl);
      Serial.println(F(" ml"));

      // Kurze Bestätigung auf dem LCD anzeigen
      lcd.clear();
      lcd.print(F("Man. Pumpe AUS"));
      lcd.setCursor(0,1);
      lcd.print(F("Gepumpt: ~")); // Tilde für "ungefähr"
      lcd.print(pumpedMl);
      lcd.print(F("ml"));
      delay(1500); // Anzeige für 1.5 Sekunden

      // Nach manuellem Pumpen: Messintervall zurücksetzen und aktuelle Werte lesen
      letzterMessZeitpunkt = 0; // Erzwingt sofortige Messung
      measurementCount = 0;
      currentMessintervallSec = INITIAL_MESSINTERVALL_SEC;
      aktuelleFeuchtigkeit = leseBodenFeuchtigkeit(); // Endgültigen Wert nach Pumpen lesen
      tankLeer = leseTankFuellstand() < MIN_TANK_WERT;
      updateLCD(); // Normales Display wiederherstellen

    }
    // else { // Hier könnte Logik für einen kurzen Druck stehen, falls gewünscht }
  }

  // ----------------------------------------------------
  // --- 2. Update während manuellem Pumpen ---
  // ----------------------------------------------------
  if (longPressActive) {
      // --- Sekündliche Bodenfeuchtigkeitsmessung während manuellem Pumpen ---
      if (currentMillis - lastManualSoilReadTime >= 1000) { // Prüfen, ob 1 Sekunde vergangen ist
          aktuelleFeuchtigkeit = leseBodenFeuchtigkeit();     // Bodenfeuchtigkeit erneut lesen
          lastManualSoilReadTime = currentMillis;             // Zeitstempel für nächste Sekunde aktualisieren
          // Serial.print("."); // Optional: Debug-Ausgabe für jede Messung
      }

      // --- Anzeige aktualisieren (gepumpte Menge und aktuelle Feuchtigkeit) ---
      unsigned long pumpingDurationMs = currentMillis - manualPumpStartTimeMillis;
      int pumpedMl = (int)(pumpingDurationMs * ML_PRO_MS);

      // Nur die zweite Zeile des LCDs aktualisieren, um Flackern zu vermeiden
      lcd.setCursor(0, 1);
      char manualPumpDisplay[17];
      // Nutzt die potenziell gerade aktualisierte 'aktuelleFeuchtigkeit'
      sprintf(manualPumpDisplay, "S:%-3d   P:%3dml", aktuelleFeuchtigkeit, pumpedMl);
      manualPumpDisplay[16] = '\0';
      lcd.print(manualPumpDisplay);

      // Backlight während manuellem Pumpen anlassen
      lastBacklightTime = currentMillis;
      if (!backlightStatus) {
          lcd.backlight();
          backlightStatus = true;
      }
  } // Ende if (longPressActive)

  // ----------------------------------------------------
  // --- 3. Automatischer Modus (wenn nicht manuell gepumpt wird) ---
  // ----------------------------------------------------
  if (!longPressActive) {

      // --- Automatisches LCD Backlight Timeout ---
      if (backlightStatus && (currentMillis - lastBacklightTime > BACKLIGHT_TIMEOUT_MS)) {
        backlightStatus = false;
        lcd.noBacklight(); // Schalte Backlight aus
        Serial.println(F("LCD Backlight Timeout."));
      }

      // --- Automatische Messung basierend auf Intervall ---
      bool performMeasurement = (letzterMessZeitpunkt == 0) || // Erste Messung nach Start/Reset
                                (currentMillis - letzterMessZeitpunkt >= (unsigned long)(currentMessintervallSec * 1000.0));
      bool lcdNeedsUpdate = false; // Flag, um zu entscheiden, ob das LCD komplett neu gezeichnet werden muss

      if (performMeasurement) {
        letzterMessZeitpunkt = currentMillis; // Zeitstempel der aktuellen Messung speichern

        // Sensorwerte lesen
        int tankWert = leseTankFuellstand();
        bool previousTankLeer = tankLeer; // Alten Zustand merken für Vergleich
        tankLeer = tankWert < MIN_TANK_WERT;

        int previousFeuchtigkeit = aktuelleFeuchtigkeit; // Alten Zustand merken
        aktuelleFeuchtigkeit = leseBodenFeuchtigkeit();
        bool istTrocken = aktuelleFeuchtigkeit > TROCKEN_SCHWELLE;

        measurementCount++; // Messzähler erhöhen

        // Debug-Ausgabe im Serial Monitor
        Serial.print(F("Messung #")); Serial.print(measurementCount);
        Serial.print(F(" | Feucht: ")); Serial.print(aktuelleFeuchtigkeit);
        Serial.print(F(" | Tank: ")); Serial.print(tankWert);
        Serial.print(F(" (Leer: ")); Serial.print(tankLeer ? F("Ja") : F("Nein")); Serial.print(F(")"));
        Serial.print(F(" | Trocken: ")); Serial.print(istTrocken ? F("Ja") : F("Nein"));
        Serial.print(F(" | Timer: ")); Serial.print(trockenzeitLaeuft ? F("An") : F("Aus"));
        Serial.print(F(" | PumpeAuto: ")); Serial.println(pumpeAktiv ? F("An") : F("Aus"));

        // --- Messintervall anpassen ---
        if (measurementCount >= INITIAL_MEASUREMENT_THRESHOLD && currentMessintervallSec != LONG_MESSINTERVALL_SEC) {
            currentMessintervallSec = LONG_MESSINTERVALL_SEC;
            Serial.print(F("Messintervall auf ")); Serial.print(currentMessintervallSec); Serial.println(F("s erhöht."));
            // Kurze Info auf LCD, falls Backlight an ist
            if (backlightStatus) {
                lcd.setCursor(0,0);
                lcd.print(F("Messint: 5 min "));
                delay(1500); // Kurz anzeigen
                lcdNeedsUpdate = true; // Danach muss das normale Display wiederhergestellt werden
            }
        }

        // --- Trockenzeit-Timer Logik ---
        bool timerStateChanged = false; // Hat sich der Timer-Status geändert?
        if (istTrocken) {
          // Boden ist trocken (Wert > Schwelle)
          if (!trockenzeitLaeuft && !pumpeAktiv) { // Starte Timer nur, wenn er nicht schon läuft und Pumpe nicht aktiv ist
            DateTime startZeit = rtc.now();
            if (startZeit.isValid()){
                speichereTrockenzeitRTC(startZeit); // Startzeit im NVRAM sichern
                trockenzeitLaeuft = true;
                timerStateChanged = true;
                Serial.println(F("Boden ist trocken. Trockenzeit-Timer gestartet."));
            } else {
                Serial.println(F("FEHLER: Ungültige RTC Zeit beim Starten des Timers!"));
            }
          }
        } else {
          // Boden ist feucht (Wert <= Schwelle)
          if (trockenzeitLaeuft) { // Stoppe/resette Timer, wenn er lief
            trockenzeitLaeuft = false;
            resetTrockenzeitRTC(); // Zeit im NVRAM löschen
            timerStateChanged = true;
            Serial.println(F("Boden ist feucht. Trockenzeit-Timer gestoppt/zurückgesetzt."));
          }
        }

        // Entscheiden, ob das LCD aktualisiert werden muss
        if ((previousTankLeer != tankLeer) || timerStateChanged || (previousFeuchtigkeit != aktuelleFeuchtigkeit)) {
            lcdNeedsUpdate = true;
        }

        // *** NEU: Setze das Flag, nachdem die erste Messung abgeschlossen ist ***
        if (!initialMeasurementDone) {
            initialMeasurementDone = true;
            Serial.println(F("Erste Messung nach Start abgeschlossen."));
        }

      } // Ende if (performMeasurement)

      // --- Automatische Pumpe starten? ---
      bool pumpJustStarted = false; // Wurde die Pumpe *in diesem Durchlauf* gestartet?
      // *** GEÄNDERT: Zusätzliche Bedingung initialMeasurementDone hinzugefügt ***
      if (initialMeasurementDone && trockenzeitLaeuft && !tankLeer && !pumpeAktiv) {
          DateTime start = ladeTrockenzeitRTC();
          if (start.isValid()) {
              DateTime ende = start + TimeSpan(TROCKEN_SPERRE_MS / 1000UL);
              if (jetztZeit >= ende) {
                  Serial.println(F(">>> Trockenzeit abgelaufen & erste Messung erfolgt. Starte automatische Pumpe. <<<"));
                  digitalWrite(PUMPE_PIN, HIGH); // Pumpe EIN
                  pumpeAktiv = true;             // Flag setzen
                  pumpStartzeit = currentMillis; // Startzeit merken
                  trockenzeitLaeuft = false;     // Timer beenden
                  resetTrockenzeitRTC();         // Gespeicherte Zeit löschen
                  pumpJustStarted = true;        // Flag für diesen Durchlauf setzen
                  lcdNeedsUpdate = true;         // LCD muss aktualisiert werden

                  lastBacklightTime = currentMillis;
                  if (!backlightStatus) {
                      lcd.backlight();
                      backlightStatus = true;
                  }
              }
          } else {
              Serial.println(F("FEHLER: Ungültige Trockenstartzeit aus RTC beim Pumpencheck gelesen. Reset."));
              trockenzeitLaeuft = false;
              resetTrockenzeitRTC();
              lcdNeedsUpdate = true;
          }
      }

      // --- Automatische Pumpe stoppen? ---
      bool pumpJustStopped = false; // Wurde die Pumpe *in diesem Durchlauf* gestoppt?
      if (pumpeAktiv && (currentMillis - pumpStartzeit >= PUMP_DAUER_MS)) {
          digitalWrite(PUMPE_PIN, LOW); // Pumpe AUS
          pumpeAktiv = false;           // Flag zurücksetzen
          pumpJustStopped = true;       // Flag für diesen Durchlauf setzen
          Serial.println(F(">>> Automatische Pumpe beendet. <<<"));

          letzterMessZeitpunkt = 0;
          measurementCount = 0;
          currentMessintervallSec = INITIAL_MESSINTERVALL_SEC;
          aktuelleFeuchtigkeit = leseBodenFeuchtigkeit();
          lcdNeedsUpdate = true;
      }

      // --- LCD Aktualisierung im Automatikmodus ---
      if (lcdNeedsUpdate) {
          updateLCD(); // Komplettes LCD neu zeichnen bei relevanten Änderungen
      } else if (jetztZeit.minute() != letzteMinutenZeit) {
          // Wenn sich nur die Minute geändert hat, nur die Uhrzeit und Trockenzeit aktualisieren
          letzteMinutenZeit = jetztZeit.minute();
          char uhrzeitStr[6];
          sprintf(uhrzeitStr, "%02d:%02d", jetztZeit.hour(), jetztZeit.minute());
          lcd.setCursor(0, 1);
          lcd.print(uhrzeitStr);
          lcd.print(F(" | T:"));
          displayTrockenzeit(jetztZeit);
      }

  } // Ende if (!longPressActive) - Automatischer Modus

  // ----------------------------------------------------
  // --- 4. Sicherheitscheck ---
  // ----------------------------------------------------
  if (!pumpeAktiv && !longPressActive && digitalRead(PUMPE_PIN) == HIGH) {
     Serial.println(F("WARNUNG: Pumpe war AN, obwohl Flags AUS waren. Schalte aus."));
     digitalWrite(PUMPE_PIN, LOW);
  }

  // --- Kurze Pause am Ende des Loops ---
  delay(50); // Kleine Verzögerung, um den Prozessor nicht voll auszulasten
}
