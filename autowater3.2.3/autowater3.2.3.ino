// FUNKTIONEN:
// 30h Trockenintervall mit Speicherung der Tockenzeit
// manuelles pumpen mit ml Anzeige und messen im Sekundentakt
// 2 Messintervalle
// Uhr
// Display backlight timer


#include <Wire.h>
#include <RTClib.h>
#include <LiquidCrystal_I2C.h>

RTC_DS1307 rtc; // Real-Time Clock object

// ======================
// 🔧 ADJUSTABLE VALUES
// ======================
const int TROCKEN_SCHWELLE = 175;         // Sensor value below which the soil is considered dry
const int GIESS_MENGE_ML = 100;            // Amount of water (in ml) to dispense per watering cycle
const int TROCKEN_SPERRE_H = 30;          // Required dry period (in hours) before watering
const int MIN_TANK_WERT = 0;              // Minimum sensor value for the water tank (adjust if needed)
const float ML_PRO_MINUTE = 115.0;        // Pump flow rate (ml per minute) - CALIBRATE THIS!
const unsigned long LONG_PRESS_DURATION_MS = 1000; // Duration (ms) for a long button press (manual pump)

// Dynamic measurement interval settings
const float INITIAL_MESSINTERVALL_SEC = 5.0;    // Initial interval between sensor readings (seconds)
const float LONG_MESSINTERVALL_SEC = 300.0;  // Interval after several initial readings (seconds)
const unsigned int INITIAL_MEASUREMENT_THRESHOLD = 10; // Number of initial readings before switching to long interval

// ======================
// 🔌 PIN ASSIGNMENTS
// ======================
const int SENSOR_BODEN_PIN = A0; // Analog pin for soil moisture sensor
const int SENSOR_BODEN_VCC = 7;  // Digital pin to power the soil sensor
const int SENSOR_TANK_PIN = A1;  // Analog pin for water tank level sensor
const int SENSOR_TANK_VCC = 6;   // Digital pin to power the tank sensor
const int PUMPE_PIN = 8;         // Digital pin to control the water pump relay/transistor
const int TASTER_PIN = 2;        // Digital pin for the manual control button

// ======================
// LCD & CONSTANTS
// ======================
LiquidCrystal_I2C lcd(0x27, 16, 2); // Initialize LCD (address 0x27, 16 columns, 2 rows)
const unsigned long TROCKEN_SPERRE_MS = TROCKEN_SPERRE_H * 3600000UL; // Dry period in milliseconds
const float ML_PRO_MS = ML_PRO_MINUTE / 60000.0; // Flow rate in ml per millisecond
const unsigned long PUMP_DAUER_MS = (unsigned long)(GIESS_MENGE_ML / ML_PRO_MS); // Calculated pump duration in ms
const unsigned long MESSDAUER_MS = 50; // Short delay (ms) after powering sensors before reading
const uint8_t RTC_TROCKEN_START_ADDR = 0; // Start address in RTC NVRAM to store dry period start time

// ======================
// GLOBAL VARIABLES
// ======================
unsigned long letzterMessZeitpunkt = 0; // Timestamp of the last sensor measurement
unsigned long pumpStartzeit = 0;        // Timestamp when the (automatic) pump started
bool pumpeAktiv = false;                // Flag indicating if the automatic pump cycle is running
unsigned long lastBacklightTime = 0;    // Timestamp for LCD backlight timeout
bool backlightStatus = true;            // Current status of the LCD backlight
unsigned long pressStartTime = 0;       // Timestamp when the button was pressed
bool buttonWasPressed = false;          // Flag indicating if the button is currently held down
bool longPressActive = false;           // Flag indicating if a long press (manual pump) is active
float currentMessintervallSec = INITIAL_MESSINTERVALL_SEC; // Current measurement interval
unsigned int measurementCount = 0;      // Counter for measurements to adjust interval
bool trockenzeitLaeuft = false;         // Flag indicating if the 30h dry period countdown is active
int aktuelleFeuchtigkeit = 0;           // Stores the last read soil moisture value
unsigned long letzteMinutenZeit = 99;   // Stores the last minute the clock was updated on LCD (init to invalid value)
bool tankLeer = true;                   // Initialize to true (empty) as a safe default
unsigned long manualPumpStartTimeMillis = 0; // Timestamp when manual pump *actually* started
unsigned long lastManualSoilReadTime = 0;    // *** ADDED *** Timestamp of last soil read during manual pump

// ======================
// RTC NVRAM FUNCTIONS (Store/Load Dry Time)
// ======================
// (Functions speichereTrockenzeitRTC, ladeTrockenzeitRTC, resetTrockenzeitRTC remain unchanged)
void speichereTrockenzeitRTC(const DateTime &zeit) {
  if (!zeit.isValid()) {
     Serial.println(F("FEHLER: Versuch, ungültige Zeit in RTC zu speichern!"));
     return;
  }
  rtc.writenvram(RTC_TROCKEN_START_ADDR + 0, zeit.year() - 2000);
  rtc.writenvram(RTC_TROCKEN_START_ADDR + 1, zeit.month());
  rtc.writenvram(RTC_TROCKEN_START_ADDR + 2, zeit.day());
  rtc.writenvram(RTC_TROCKEN_START_ADDR + 3, zeit.hour());
  rtc.writenvram(RTC_TROCKEN_START_ADDR + 4, zeit.minute());
  rtc.writenvram(RTC_TROCKEN_START_ADDR + 5, zeit.second());
  Serial.println(F("Trockenstartzeit im RTC gespeichert."));
}
DateTime ladeTrockenzeitRTC() {
  uint16_t jahr = rtc.readnvram(RTC_TROCKEN_START_ADDR + 0) + 2000;
  uint8_t monat = rtc.readnvram(RTC_TROCKEN_START_ADDR + 1);
  uint8_t tag = rtc.readnvram(RTC_TROCKEN_START_ADDR + 2);
  uint8_t stunde = rtc.readnvram(RTC_TROCKEN_START_ADDR + 3);
  uint8_t minute = rtc.readnvram(RTC_TROCKEN_START_ADDR + 4);
  uint8_t sekunde = rtc.readnvram(RTC_TROCKEN_START_ADDR + 5);
  if (monat < 1 || monat > 12 || tag < 1 || tag > 31 || jahr < 2024 || jahr > 2050 || stunde > 23 || minute > 59 || sekunde > 59) {
     return DateTime(); // Return invalid DateTime
  }
  return DateTime(jahr, monat, tag, stunde, minute, sekunde);
}
void resetTrockenzeitRTC() {
  for (int i = 0; i < 6; i++) {
    rtc.writenvram(RTC_TROCKEN_START_ADDR + i, 0);
  }
  Serial.println(F("Trockenstartzeit im RTC zurückgesetzt."));
}


// ======================
// 🔄 SETUP
// ======================
void setup() {
  Serial.begin(9600);
  delay(500);
  Serial.println(F("\n--- Systemstart ---"));

  pinMode(PUMPE_PIN, OUTPUT);
  pinMode(SENSOR_BODEN_VCC, OUTPUT);
  pinMode(SENSOR_TANK_VCC, OUTPUT);
  pinMode(TASTER_PIN, INPUT_PULLUP);
  digitalWrite(PUMPE_PIN, LOW);
  digitalWrite(SENSOR_BODEN_VCC, LOW);
  digitalWrite(SENSOR_TANK_VCC, LOW);
  Serial.println(F("Pins konfiguriert."));

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.print(F("Starte System..."));
  Serial.println(F("LCD initialisiert."));
  delay(1000);

  if (!rtc.begin()) {
    Serial.println(F("--- FEHLER: RTC nicht gefunden! System HÄNGT HIER. ---"));
    lcd.clear();
    lcd.print(F("RTC nicht gef.!"));
    while (1) delay(100);
  }
  Serial.println(F("RTC initialisiert."));

   if (!rtc.isrunning()) {
    Serial.println(F("WARNUNG: RTC lief nicht! Setze Zeit..."));
    lcd.clear();
    lcd.print(F("RTC Zeit gesetzt"));
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    delay(1000);
    resetTrockenzeitRTC();
    trockenzeitLaeuft = false;
  } else {
     Serial.println(F("RTC läuft. Prüfe gespeicherte Zeit..."));
     DateTime gespeicherterStart = ladeTrockenzeitRTC();
     if (gespeicherterStart.isValid()) {
         trockenzeitLaeuft = true;
         Serial.println(F("Gültige Trockenstartzeit aus RTC geladen."));
     } else {
         trockenzeitLaeuft = false;
         Serial.println(F("Keine gültige Trockenstartzeit im RTC gefunden."));
     }
  }

  lcd.clear();
  lcd.print(F("System bereit."));
  Serial.println(F("System bereit."));
  delay(1000);

  lastBacklightTime = millis();
  letzteMinutenZeit = 99;
}

// ======================
// HELPER FUNCTIONS
// ======================

// Updates the main LCD display (called on state changes or measurement)
void updateLCD() {
  if (longPressActive) {
      return; // Manual pump display handled separately
  }

  char uhrzeitStr[6];
  DateTime jetzt = rtc.now();
  sprintf(uhrzeitStr, "%02d:%02d", jetzt.hour(), jetzt.minute());

  letzteMinutenZeit = jetzt.minute();

  lcd.clear();
  lcd.setCursor(0, 0); // Row 0

  if (tankLeer) {
    lcd.print(F("Tank leer!      "));
  } else if (pumpeAktiv) {
    unsigned long elapsedPumpTime = (millis() - pumpStartzeit) / 1000;
    unsigned long totalPumpTime = PUMP_DAUER_MS / 1000;
    lcd.print(F("Pumpe:"));
    if (elapsedPumpTime > totalPumpTime) elapsedPumpTime = totalPumpTime;
    lcd.print(elapsedPumpTime);
    lcd.print(F("/"));
    lcd.print(totalPumpTime);
    lcd.print(F("s   "));
  } else {
    bool istTrocken = aktuelleFeuchtigkeit < TROCKEN_SCHWELLE;
    lcd.print(istTrocken ? F("Trocken ") : F("Feucht  "));
    lcd.print(aktuelleFeuchtigkeit);
    if(aktuelleFeuchtigkeit < 10) lcd.print(F("    "));
    else if (aktuelleFeuchtigkeit < 100) lcd.print(F("   "));
    else if (aktuelleFeuchtigkeit < 1000) lcd.print(F("  "));
    else lcd.print(F(" "));
  }

  lcd.setCursor(0, 1); // Row 1
  lcd.print(uhrzeitStr);
  lcd.print(F(" | T:"));
  displayTrockenzeit(jetzt);
}

// Calculates and displays the remaining dry time on the LCD (without clearing)
void displayTrockenzeit(DateTime jetztZeit) {
  lcd.setCursor(9, 1);
  if (trockenzeitLaeuft) {
    DateTime start = ladeTrockenzeitRTC();
    if (!start.isValid()) {
        lcd.print(F("RTC Err "));
        Serial.println(F("FEHLER: Ungültige Trockenstartzeit beim Anzeigen gelesen."));
        trockenzeitLaeuft = false;
        resetTrockenzeitRTC();
        return;
    }
    DateTime ende = start + TimeSpan(TROCKEN_SPERRE_MS / 1000UL);
    TimeSpan verbleibend = ende - jetztZeit;
    if (verbleibend.totalseconds() <= 0) {
        lcd.print(F("Bereit "));
    } else {
        long verbleibendeStunden = verbleibend.totalseconds() / 3600;
        long verbleibendeMinuten = (verbleibend.totalseconds() % 3600) / 60;
        if (verbleibendeStunden < 10) lcd.print(F(" "));
        lcd.print(verbleibendeStunden);
        lcd.print(F("h"));
        if (verbleibendeMinuten < 10) lcd.print(F("0"));
        lcd.print(verbleibendeMinuten);
        lcd.print(F("m"));
    }
  } else {
    lcd.print(F("--h--m "));
  }
}

// Reads the soil moisture sensor value
int leseBodenFeuchtigkeit() {
  digitalWrite(SENSOR_BODEN_VCC, HIGH);
  delay(MESSDAUER_MS); // Wait for sensor stabilization
  int feuchtigkeit = analogRead(SENSOR_BODEN_PIN);
  digitalWrite(SENSOR_BODEN_VCC, LOW);
  return feuchtigkeit;
}

// Reads the tank level sensor value
int leseTankFuellstand() {
  digitalWrite(SENSOR_TANK_VCC, HIGH);
  delay(MESSDAUER_MS); // Wait for sensor stabilization
  int tankWert = analogRead(SENSOR_TANK_PIN);
  digitalWrite(SENSOR_TANK_VCC, LOW);
  return tankWert;
}

// ======================
// 🔄 MAIN LOOP
// ======================
void loop() {
  unsigned long currentMillis = millis();
  bool buttonIsPressed = (digitalRead(TASTER_PIN) == LOW);
  DateTime jetztZeit = rtc.now();

  // --- Button Logic ---
  if (buttonIsPressed && !buttonWasPressed) {
    pressStartTime = currentMillis;
    buttonWasPressed = true;
    lastBacklightTime = currentMillis;
    if (!backlightStatus) {
      lcd.backlight();
      backlightStatus = true;
    }
    if (!longPressActive && currentMessintervallSec != INITIAL_MESSINTERVALL_SEC) {
        currentMessintervallSec = INITIAL_MESSINTERVALL_SEC;
        measurementCount = 0;
        letzterMessZeitpunkt = 0;
        Serial.println(F("Messintervall durch Tasterdruck zurückgesetzt."));
    }
  }

  // --- Long Press Handling (Manual Pump Start) ---
  if (buttonIsPressed && buttonWasPressed) {
    if (!longPressActive && (currentMillis - pressStartTime >= LONG_PRESS_DURATION_MS)) {
        longPressActive = true;
        manualPumpStartTimeMillis = currentMillis;
        pumpeAktiv = false;
        digitalWrite(PUMPE_PIN, HIGH);
        Serial.println(F("Langer Druck erkannt - Manuelle Pumpe AN"));

        aktuelleFeuchtigkeit = leseBodenFeuchtigkeit(); // Initial read
        tankLeer = leseTankFuellstand() < MIN_TANK_WERT;
        lastManualSoilReadTime = currentMillis; // *** ADDED: Initialize manual read timer ***

        lcd.clear();
        lcd.setCursor(0, 0);
        if (tankLeer) {
            lcd.print(F("Tank leer! MPUMP"));
        } else {
            lcd.print(F("Man. Pumpe EIN"));
        }
        lcd.setCursor(0, 1);
        char manualPumpDisplay[17];
        sprintf(manualPumpDisplay, "S:%-3d   P:%3dml", aktuelleFeuchtigkeit, 0);
        manualPumpDisplay[16] = '\0';
        lcd.print(manualPumpDisplay);

        lastBacklightTime = currentMillis;
        if (!backlightStatus) {
          lcd.backlight();
          backlightStatus = true;
        }
    }
  }

  // --- Button Release Handling ---
  if (!buttonIsPressed && buttonWasPressed) {
    buttonWasPressed = false;
    if (longPressActive) {
      longPressActive = false;
      digitalWrite(PUMPE_PIN, LOW);
      unsigned long pumpingDurationMs = currentMillis - manualPumpStartTimeMillis;
      int pumpedMl = (int)(pumpingDurationMs * ML_PRO_MS);
      Serial.print(F("Taster losgelassen - Manuelle Pumpe AUS. Gepumpt: ca. "));
      Serial.print(pumpedMl);
      Serial.println(F(" ml"));

      lcd.clear();
      lcd.print(F("Man. Pumpe AUS"));
      lcd.setCursor(0,1);
      lcd.print(F("Gepumpt: ~"));
      lcd.print(pumpedMl);
      lcd.print(F("ml"));
      delay(1500);

      letzterMessZeitpunkt = 0;
      measurementCount = 0;
      currentMessintervallSec = INITIAL_MESSINTERVALL_SEC;
      aktuelleFeuchtigkeit = leseBodenFeuchtigkeit(); // Read final value
      tankLeer = leseTankFuellstand() < MIN_TANK_WERT;
      updateLCD(); // Restore normal display

    }
    // else { short press occurred }
  }

  // --- Manual Pump Display Update (Only if active) ---
  if (longPressActive) {
      // *** ADDED: Periodic Soil Reading Logic ***
      if (currentMillis - lastManualSoilReadTime >= 1000) { // Check if 1 second (1000 ms) has passed
          aktuelleFeuchtigkeit = leseBodenFeuchtigkeit();     // Read the soil sensor again
          lastManualSoilReadTime = currentMillis;             // Reset the timer for the next second
          // Serial.print("."); // Optional: Print dot to serial monitor for debugging reads
      }

      // Calculate elapsed time and pumped volume
      unsigned long pumpingDurationMs = currentMillis - manualPumpStartTimeMillis;
      int pumpedMl = (int)(pumpingDurationMs * ML_PRO_MS);

      // Format and update the second row ONLY
      // This part now uses the potentially updated 'aktuelleFeuchtigkeit'
      lcd.setCursor(0, 1);
      char manualPumpDisplay[17];
      sprintf(manualPumpDisplay, "S:%-3d   P:%3dml", aktuelleFeuchtigkeit, pumpedMl);
      manualPumpDisplay[16] = '\0';
      lcd.print(manualPumpDisplay);

      // Keep backlight on
      lastBacklightTime = currentMillis;
      if (!backlightStatus) {
          lcd.backlight();
          backlightStatus = true;
      }
  }


  // --- Automatic LCD Backlight Timeout ---
  if (!longPressActive && backlightStatus && (currentMillis - lastBacklightTime > 60000UL)) {
    backlightStatus = false;
    lcd.noBacklight();
  }

  // --- Automatic Measurement and Watering Logic (Only run if manual pump is OFF) ---
  if (!longPressActive) {

    // --- Sensor Reading Interval ---
    bool performMeasurement = (letzterMessZeitpunkt == 0) || (currentMillis - letzterMessZeitpunkt >= (unsigned long)(currentMessintervallSec * 1000.0));
    bool lcdNeedsUpdate = false;

    if (performMeasurement) {
      letzterMessZeitpunkt = currentMillis;
      int tankWert = leseTankFuellstand();
      bool previousTankLeer = tankLeer;
      tankLeer = tankWert < MIN_TANK_WERT;
      int previousFeuchtigkeit = aktuelleFeuchtigkeit;
      aktuelleFeuchtigkeit = leseBodenFeuchtigkeit();
      bool istTrocken = aktuelleFeuchtigkeit < TROCKEN_SCHWELLE;
      measurementCount++;

      Serial.print(F("Messung #")); Serial.print(measurementCount);
      Serial.print(F(" Feucht: ")); Serial.print(aktuelleFeuchtigkeit);
      Serial.print(F(" Tank: ")); Serial.print(tankWert);
      Serial.print(F(" (Leer: ")); Serial.print(tankLeer ? F("Ja") : F("Nein")); Serial.print(F(")"));
      Serial.print(F(" Trocken: ")); Serial.print(istTrocken ? F("Ja") : F("Nein"));
      Serial.print(F(" Timer: ")); Serial.print(trockenzeitLaeuft ? F("An") : F("Aus"));
      Serial.print(F(" PumpeAuto: ")); Serial.println(pumpeAktiv ? F("An") : F("Aus"));

      if (measurementCount >= INITIAL_MEASUREMENT_THRESHOLD && currentMessintervallSec != LONG_MESSINTERVALL_SEC) {
          currentMessintervallSec = LONG_MESSINTERVALL_SEC;
          Serial.print(F("Messintervall auf ")); Serial.print(currentMessintervallSec); Serial.println(F("s erhöht."));
          if (backlightStatus) {
              lcd.setCursor(0,0);
              lcd.print(F("Messint: 5 min "));
              delay(1500);
              lcdNeedsUpdate = true;
          }
      }

      bool timerStateChanged = false;
      if (istTrocken) {
        if (!trockenzeitLaeuft && !pumpeAktiv) {
          DateTime startZeit = rtc.now();
          if (startZeit.isValid()){
              speichereTrockenzeitRTC(startZeit);
              trockenzeitLaeuft = true;
              timerStateChanged = true;
              Serial.println(F("Boden ist trocken. Trockenzeit-Timer gestartet."));
          } else {
              Serial.println(F("FEHLER: Ungültige RTC Zeit beim Starten des Timers!"));
          }
        }
      } else {
        if (trockenzeitLaeuft) {
          trockenzeitLaeuft = false;
          resetTrockenzeitRTC();
          timerStateChanged = true;
          Serial.println(F("Boden ist feucht. Trockenzeit-Timer gestoppt/zurückgesetzt."));
        }
      }

      if ((previousTankLeer != tankLeer) || timerStateChanged || (previousFeuchtigkeit != aktuelleFeuchtigkeit)) {
          lcdNeedsUpdate = true;
      }
    } // End of measurement interval check

    // --- Automatic Pump Trigger ---
    bool pumpJustStarted = false;
    if (trockenzeitLaeuft && !tankLeer && !pumpeAktiv) {
        DateTime start = ladeTrockenzeitRTC();
        if (start.isValid()) {
            DateTime ende = start + TimeSpan(TROCKEN_SPERRE_MS / 1000UL);
            if (jetztZeit >= ende) {
                Serial.println(F(">>> Trockenzeit abgelaufen. Starte automatische Pumpe. <<<"));
                digitalWrite(PUMPE_PIN, HIGH);
                pumpeAktiv = true;
                pumpStartzeit = currentMillis;
                trockenzeitLaeuft = false;
                resetTrockenzeitRTC();
                pumpJustStarted = true;
                lcdNeedsUpdate = true;
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

    // --- Automatic Pump Stop ---
    bool pumpJustStopped = false;
    if (pumpeAktiv && (currentMillis - pumpStartzeit >= PUMP_DAUER_MS)) {
        digitalWrite(PUMPE_PIN, LOW);
        pumpeAktiv = false;
        pumpJustStopped = true;
        Serial.println(F(">>> Automatische Pumpe beendet. <<<"));
        letzterMessZeitpunkt = 0;
        measurementCount = 0;
        currentMessintervallSec = INITIAL_MESSINTERVALL_SEC;
        aktuelleFeuchtigkeit = leseBodenFeuchtigkeit();
        lcdNeedsUpdate = true;
    }

    // --- Update LCD Display ---
    if (lcdNeedsUpdate) {
        updateLCD();
    } else if (jetztZeit.minute() != letzteMinutenZeit) {
        letzteMinutenZeit = jetztZeit.minute();
        char uhrzeitStr[6];
        sprintf(uhrzeitStr, "%02d:%02d", jetztZeit.hour(), jetztZeit.minute());
        lcd.setCursor(0, 1);
        lcd.print(uhrzeitStr);
        lcd.print(F(" | T:"));
        displayTrockenzeit(jetztZeit);
    }

  } // End of automatic mode block (!longPressActive)

  // --- Final safety check ---
  if (!pumpeAktiv && !longPressActive && digitalRead(PUMPE_PIN) == HIGH) {
     Serial.println(F("WARNUNG: Pumpe war AN, obwohl Flags AUS waren. Schalte aus."));
     digitalWrite(PUMPE_PIN, LOW);
  }

  delay(50); // Small delay
}