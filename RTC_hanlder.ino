// RTC_handler.ino
// DS3231 code plus the non-blocking serial time logic

#include "globals.h"


void initRTC() {
  // Switch to master mode so we can access the RTC:
  //Serial.println("initRTC");
  // Switch to master mode:
  Wire.end();
  Wire.begin();
  myRTC.begin();  // Calls Wire.begin()
  //Serial.println("RTC connected");


  if (myRTC.oscStopped(true)) {  // check the oscillator and clear flag if present
    if (serialEnabled) {
      Serial.println("RTC was off");
    }
  }  //else {
  //Serial.println("RTC okay");
  //}


  //setSyncProvider(myRTC.get);  // causes the Time library to synchronize with the external RTC by calling RTC.get() every five minutes by default.
  //Serial.println("time synced");
  /*
  if (timeStatus() != timeSet) {
    Serial.println("Unable to sync with the RTC");
  } else {
    Serial.println("RTC has set the system time");
  }
*/

  rtcInitialized = true;
  time_t t = myRTC.get();  //Returns zero if an I2C error occurs (RTC not present, etc.).
  // Switch back to slave mode:
  Wire.end();
  Wire.begin(SNIFF_ADDR);
  UTC_offset = 1;  // Determine UTC_offset from our timestamp array
  for (size_t i = 0; i < sizeof(timestampDataArray) / sizeof(timestampDataArray[0]); i++) {
    if (t > timestampDataArray[i].timestamp) {
      UTC_offset = timestampDataArray[i].offset;
    }
  }
  // Adjust the pending time by the offset (in seconds):
  t += UTC_offset * 3600;

  if (serialEnabled) {
    char TimeStr[40];
    sprintf(TimeStr, "%04d-%02d-%02d %02d:%02d:%02d",
            year(t), month(t), day(t),
            hour(t), minute(t), second(t));

    //Serial.println("RTC time:");
    Serial.println(TimeStr);
    Serial.flush();
  }
}

// Non-blocking approach: we read from serial each loop
// If we find a line "YYYY-MM-DD HH:MM:SS", store in pendingDT
void processSerialForTimeNonBlocking() {
  static String inputLine;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputLine.length() >= 19) {
        // Expecting a string like "2023-10-03 15:23:45"
        int y = inputLine.substring(0, 4).toInt();
        int mo = inputLine.substring(5, 7).toInt();
        int d = inputLine.substring(8, 10).toInt();
        int hh = inputLine.substring(11, 13).toInt();
        int mm = inputLine.substring(14, 16).toInt();
        int ss = inputLine.substring(17, 19).toInt();
        if (y > 2000 && mo >= 1 && mo <= 12 && d >= 1 && d <= 31) {
          // Use TimeLib to convert to time_t.
          tmElements_t tm;
          tm.Year = y - 1970;  // TimeLib expects year offset from 1970
          tm.Month = mo;
          tm.Day = d;
          tm.Hour = hh;
          tm.Minute = mm;
          tm.Second = ss;
          pendingTime = makeTime(tm);
          // Determine UTC_offset from our timestamp array:
          UTC_offset = 1;
          for (size_t i = 0; i < sizeof(timestampDataArray) / sizeof(timestampDataArray[0]); i++) {
            if (pendingTime > timestampDataArray[i].timestamp) {
              UTC_offset = timestampDataArray[i].offset;
            }
          }
          // Adjust the pending time by the offset (in seconds):
          pendingTime -= UTC_offset * 3600;


          havePendingTime = true;
          //applyPendingTime();  //2025-02-01 15:44:00
          Serial.print("Parsed new time: ");
          Serial.println(inputLine);

        } else {
          Serial.print("Invalid parse in: ");
          Serial.println(inputLine);
        }
      }
      inputLine = "";
    } else {
      inputLine += c;
    }
  }
}

bool isTimeSetPending() {
  return havePendingTime;
}

void applyPendingTime() {
  if (!rtcInitialized || !havePendingTime) return;
  havePendingTime = false;

  // Switch to master mode:
  Wire.end();
  Wire.begin();

  // Read the current RTC time (as time_t):

  time_t oldTime = myRTC.get();
  // Get the pending time (from Serial) as t:
  time_t t = pendingTime;

  // Update the RTC:
  myRTC.set(t);

  // Switch back to slave mode:
  Wire.end();
  Wire.begin(SNIFF_ADDR);

  // Determine UTC_offset from our timestamp array:
  UTC_offset = 1;
  for (size_t i = 0; i < sizeof(timestampDataArray) / sizeof(timestampDataArray[0]); i++) {
    if (t > timestampDataArray[i].timestamp) {
      UTC_offset = timestampDataArray[i].offset;
    }
  }
  // Adjust the pending time by the offset (in seconds):
  oldTime += UTC_offset * 3600;
  t += UTC_offset * 3600;

  // Format the old and new times for printing using TimeLib functions:
  char oldTimeStr[32];
  char newTimeStr[32];
  sprintf(oldTimeStr, "%04d-%02d-%02d %02d:%02d:%02d",
          year(oldTime), month(oldTime), day(oldTime),
          hour(oldTime), minute(oldTime), second(oldTime));
  sprintf(newTimeStr, "%04d-%02d-%02d %02d:%02d:%02d",
          year(t), month(t), day(t),
          hour(t), minute(t), second(t));

  if (serialEnabled) {
    Serial.println("RTC time changed from -> to:");
    Serial.println(oldTimeStr);
    Serial.println(newTimeStr);
  }
}