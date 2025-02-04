/*
Arduino Micro = Leonardo


2025.01.27 erster Versuch

Set RTC: send over serial "YYYY-MM-DD HH:MM:SS" eg. "2025-02-01 01:22:40"

Arduino Micro
*/

#include "globals.h"




// Forward declarations of functions in other .ino files
void initSniffer();
void initRTC();
void initLogging();
bool newMeasurementAvailable();          // returns true if a fresh measurement arrived
float getLatestMeasurementValue();       // returns the most recent measurement
void processSerialForTimeNonBlocking();  // checks serial for time input, non-blocking
bool isTimeSetPending();                 // true if we parsed a new time but haven't set RTC yet
void applyPendingTime();                 // sets the RTC from the parsed time
void handleSniffer();                    // handle completed messages
DecodeResult getLastDecodeResult();      // Retrieves the last decoded measurement and clears the 'ready' flag.
void logData(const DecodeResult &res);
uint16_t readNrTxtOrSearch();
void onI2CReceive(int numBytes);
void onI2CRequest();
DecodeResult decodePCFData(const uint8_t data[], size_t len);
void openLogFile(uint16_t num);


void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN_TX, OUTPUT);
  pinMode(LED_BUILTIN_RX, OUTPUT);
  digitalWrite(LED_BUILTIN_TX, LOW);
  digitalWrite(LED_BUILTIN_RX, LOW);
  //while (!Serial) { ; }  // Blocking
  delay(2000);
  //Serial.println("Setup...");
  // If you want to detect if USB host is present, you can do:
  if (Serial) {
    serialEnabled = true;
    Serial.println("USB connected");
  }
  //delay(2000);
  //Serial.flush();
  //delay(1000);
  // 2) Initialize sub-systems
  initRTC();


  initSniffer();
  //Serial.println("Sniffer done");
  initLogging();
  if (serialEnabled) {
    Serial.println("YYYY-MM-DD HH:MM:SS");
  }
  //Serial.println("Setup finished.");
  digitalWrite(LED_BUILTIN_TX, HIGH);
  digitalWrite(LED_BUILTIN_RX, HIGH);
}

void loop() {
  // 1) For the first 10 seconds after power-up, we watch for date/time on Serial.
  //    We'll do this by recording the time on the first call.
  static bool started = false;
  if (!started) {
    started = true;
    startupMillis = millis();
  }
  if (serialEnabled) {
    if (millis() - startupMillis < SERIAL_WINDOW_MS) {
      // Non-blocking check for date/time on Serial, do not block
      processSerialForTimeNonBlocking();
    }
  }
  handleSniffer();
  // then the rest:
  if (newMeasurementAvailable()) {
    if (isTimeSetPending()) {
      // We do it here so we don't interfere with I2C bus during the initial flurry,
      // also ensures time is set after the sniffer is fully running
      applyPendingTime();
    }

    DecodeResult res = getLastDecodeResult();
    if (serialEnabled) {
      /*
      if (res.overload) {
        Serial.print("OVERLOAD");
      } else if (res.underload) {
        Serial.print("UNDERLOAD");
      } 
      */

      Serial.print(res.text);
      Serial.print(" text <=> value ");
      Serial.print(res.value, 2);  // show numeric

      if (res.stable) {
        Serial.print("(Stable)");
      }
      Serial.println();
    }
    // The logger writes to "currentLogFile" automatically
    // including date/time from RTC, or fallback if RTC not set
    logData(res);
  }
}
