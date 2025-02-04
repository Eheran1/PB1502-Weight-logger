// Log_to_SD.ino
// Manages the SD card, auto-incrementing filenames, and log writing.

#include "globals.h"


void initLogging() {
  if (!SD.begin(SD_CHIP_SELECT)) {
    if (serialEnabled) {
      Serial.println("SD failed");
    }
    return;
  }
  sdOK = true;
  // read "Nr.txt"
  nextFileNumber = readNrTxtOrSearch();
  // open new file
  openLogFile(nextFileNumber);
  // update Nr.txt to next
  //writeNrTxt(nextFileNumber + 1);
}

static void openLogFile(uint16_t num) {
  char fname[12];
  sprintf(fname, "%05u.txt", num);
  currentLogFile = SD.open(fname, FILE_WRITE);
  if (!currentLogFile) {
    if (serialEnabled) {
      Serial.print("Failed: ");
      Serial.println(fname);
    }
    sdOK = false;
    return;
  }
  if (serialEnabled) {
    //Serial.print("Logging to: ");
    Serial.println(fname);
  }

  // optional header
  currentLogFile.println("t,t,Text,m,Stable");
  currentLogFile.flush();
}


// Logs a transition (both to Serial and SD, for example)
void logTransition(unsigned long t, const DecodeResult &res) {
  if (serialEnabled) {
    Serial.print(",");
    Serial.print(t);
    Serial.print(",");
    Serial.print(res.text);
    Serial.print(",");
    Serial.print(res.value, 2);
    Serial.println();
  }

  // Also log to SD:
  logData(res);
}


// Buffering function: logs only the first and last values of a block.
// processDecodedResult() is called each time a new decoded measurement is available.
void processDecodedResult(const DecodeResult &newRes, unsigned long newTime) {
  // If no block is active, start one:
  if (!blockActive) {
    blockActive = true;
    blockRes = newRes;  // start the block with the new measurement
    blockStartTime = newTime;
    blockEndTime = newTime;
    lastLoggedValue = newRes.value;
    return;
  }
  //Serial.print(newRes.value);
  //Serial.print(",");
  //Serial.println(blockRes.value);
  // If the new reading is essentially the same as the current block’s value:
  if (fabs(newRes.value - blockRes.value) < TOLERANCE) {
    //Serial.println("same value");
    // Update the block’s end time.
    blockEndTime = newTime;
    // Check if the block has been active for longer than BLOCK_TIMEOUT.
    if ((newTime - blockStartTime) >= BLOCK_TIMEOUT) {
      //Serial.println("timeout");
      //logTransition(blockStartTime, blockRes);
      logTransition(blockEndTime, blockRes);
      lastLoggedValue = blockRes.value;
      // Reset the block.
      blockActive = false;
    }
  } else {
    // A new (changed) reading is detected.
    // Flush the current block first if it hasn't been logged yet.
    if (fabs(blockRes.value - lastLoggedValue) >= TOLERANCE) {
      logTransition(blockStartTime, blockRes);
      if (blockEndTime != blockStartTime) {
        logTransition(blockEndTime, blockRes);
      }
      lastLoggedValue = blockRes.value;
    }
    // Log the new reading immediately if it differs from the last logged value.
    if (fabs(newRes.value - lastLoggedValue) >= TOLERANCE) {
      logTransition(newTime, newRes);
      lastLoggedValue = newRes.value;
    }
    // Start a new block with the new reading.
    blockRes = newRes;
    blockStartTime = newTime;
    blockEndTime = newTime;
  }
}





void logData(const DecodeResult &res) {
    digitalWrite(LED_BUILTIN_TX, LOW);
  if (!sdOK || !currentLogFile) return;


  // Retrieve current time from RTC (using our own rtcInitialized flag)
  // Switch to master mode to get RTC time:
  time_t nowTime;
  bool haveRTC = false;
  if (rtcInitialized) {
    Wire.end();
    Wire.begin();
    nowTime = myRTC.get();
    UTC_offset = 1;  // Determine UTC_offset from our timestamp array
    for (size_t i = 0; i < sizeof(timestampDataArray) / sizeof(timestampDataArray[0]); i++) {
      if (nowTime > timestampDataArray[i].timestamp) {
        UTC_offset = timestampDataArray[i].offset;
      }
    }
    // Adjust the pending time by the offset (in seconds):
    nowTime += UTC_offset * 3600;

    Wire.end();
    Wire.begin(SNIFF_ADDR);
    haveRTC = true;
  }

  // Format the RTC time into a buffer:
  char buf[32];
  if (haveRTC) {
    sprintf(buf, "%04d-%02d-%02d %02d:%02d:%02d",
            year(nowTime), month(nowTime), day(nowTime),
            hour(nowTime), minute(nowTime), second(nowTime));
  } else {
    strcpy(buf, "NoRTC");
  }

  // Write CSV columns:
  // millis,DateTime,Value,Stable,Overload,Underload,Negative,Text
  currentLogFile.print(millis());
  currentLogFile.print(",");
  currentLogFile.print(buf);
  currentLogFile.print(",");
  currentLogFile.print(res.text);
  currentLogFile.print(",");
  currentLogFile.print(res.value, 2);
  currentLogFile.print(",");
  currentLogFile.print(res.stable ? 1 : 0);
  //currentLogFile.print(",");
  //currentLogFile.print(res.overload ? 1 : 0);
  //currentLogFile.print(",");
  //currentLogFile.print(res.underload ? 1 : 0);
  //currentLogFile.print(",");
  //currentLogFile.print(res.negative ? 1 : 0);

  currentLogFile.println();
  currentLogFile.flush();
    digitalWrite(LED_BUILTIN_TX, HIGH);
}

// A function to read Nr.txt or do fallback search
uint16_t readNrTxtOrSearch() {
  // do a binary search approach
  uint16_t low = 0, high = 99999;
  while (low < high) {
    uint16_t mid = (low + high + 1) / 2;
    char testName[12];
    sprintf(testName, "%05u.txt", mid);
    if (SD.exists(testName)) {
      low = mid;
    } else {
      high = mid - 1;
    }
  }
  return low + 1;
}
