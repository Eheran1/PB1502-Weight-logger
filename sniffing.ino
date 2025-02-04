// sniffing.ino
// Contains all your I2C sniffing code, decodePCFData, etc.


#include "globals.h"



void initSniffer() {
  // Start as I2C slave for address 0x38
  Wire.begin(SNIFF_ADDR);
  // Register callbacks
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);
  if (serialEnabled) {
    Serial.println("Sniffer initialized");
  }
}



// The callback for incoming data
void onI2CReceive(int numBytes) {
  digitalWrite(LED_BUILTIN_RX, LOW);
  if (numBytes > MAX_BUFFER_SIZE) {
    numBytes = MAX_BUFFER_SIZE;  // prevent overflow, we expect ~18 bytes
  }

  //Serial.print("Received bytes: ");
  for (int i = 0; i < numBytes; i++) {
    if (Wire.available()) {
      dataBuffer[i] = Wire.read();
      //Serial.print("0x");
      //if (dataBuffer[i] < 0x10)
      //  Serial.print("0");
      //Serial.print(dataBuffer[i], HEX);
      //Serial.print(" ");
    }
  }
  //Serial.println();

  dataCount = numBytes;
  messageComplete = true;
  digitalWrite(LED_BUILTIN_RX, HIGH);
}

// The callback when master requests data
void onI2CRequest() {
  // We do nothing, we are sniffing only
}

// Called from loop to see if we have new data
bool newMeasurementAvailable() {
  return measurementAvailable;
}

// Retrieves the last decoded measurement and clears the 'ready' flag.
DecodeResult getLastDecodeResult() {
  measurementAvailable = false;
  return lastDecode;
}


// data[] is our 16-byte display data.
// bitIndex is 0..127 (since 16 bytes × 8 bits = 128 bits).
// Returns true if that bit is set to 1.
bool isSegmentOn(const uint8_t data[], int bitIndex) {
  if (bitIndex < 0 || bitIndex >= 128) return false;  // out of range
  int byteIndex = bitIndex / 8;
  int bitOffset = bitIndex % 8;
  return (data[byteIndex] & (1 << bitOffset)) != 0;
}




// Return 0..9 if recognized, or -1 if the segments don’t match any known digit
// Also returns dpOn = true if decimal point is lit.
int decodeOneDigit(const uint8_t data[], const DigitMap &dm, bool &dpOn) {
  // Gather the 7 segments A..G into a 7-bit pattern in the order (G F E D C B A).
  // Or you can do (A,B,C,D,E,F,G). Just be consistent with SEG_PATTERN.
  // Here let's do (A,B,C,D,E,F,G) from LSB to MSB:
  uint8_t segs = 0;
  if (isSegmentOn(data, dm.segA)) segs |= (1 << 0);  // bit0 => A
  if (isSegmentOn(data, dm.segB)) segs |= (1 << 1);  // bit1 => B
  if (isSegmentOn(data, dm.segC)) segs |= (1 << 2);  // bit2 => C
  if (isSegmentOn(data, dm.segD)) segs |= (1 << 3);  // bit3 => D
  if (isSegmentOn(data, dm.segE)) segs |= (1 << 4);  // bit4 => E
  if (isSegmentOn(data, dm.segF)) segs |= (1 << 5);  // bit5 => F
  if (isSegmentOn(data, dm.segG)) segs |= (1 << 6);  // bit6 => G

  // Check the decimal point
  dpOn = isSegmentOn(data, dm.segDP);

  // Check for "only G=1" pattern => negative sign
  if (segs == 0x40) {
    return NEG_SIGN_ONLY;
  }

  // If all segments are OFF, return a special value (-2 for "unused")
  if (segs == 0x00) {
    return -2;  // indicates an unused digit
  }
  // Compare with each digit pattern
  for (int digit = 0; digit < 10; digit++) {
    if (segs == SEG_PATTERN[digit]) {
      return digit;  // found a match
    }
  }

  return -1;  // no match
}

bool compareArrays(const uint8_t a[], const uint8_t b[], size_t len) {
  for (size_t i = 0; i < len; i++) {
    if (a[i] != b[i]) return false;
  }
  return true;
}





// We call this in loop to handle completed messages
void handleSniffer() {
  if (!messageComplete) return;  // nothing new

  noInterrupts();
  bool localMsgComplete = messageComplete;
  size_t localDataCount = dataCount;
  messageComplete = false;
  dataCount = 0;
  interrupts();

  if (localMsgComplete && localDataCount > 2) {
    // Optionally, print the display data (skipping the first 2 bytes):
    /*
    Serial.print("Display data bytes: ");
    for (int i = 2; i < localDataCount; i++) {
      Serial.print("0x");
      if (dataBuffer[i] < 0x10)
        Serial.print("0");
      Serial.print(dataBuffer[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
    */

    // Check if the display data matches the known overload pattern.
    // (Here we assume the overload pattern is 16 bytes long.)
    if (compareArrays(&dataBuffer[2], OVERLOAD_PATTERN, 16)) {
      if (serialEnabled) {
        Serial.println("Overload");
      }
      return;  // Do not update lastMeasurement
    }
    if (compareArrays(&dataBuffer[2], UNDERLOAD_PATTERN, 16)) {
      if (serialEnabled) {
        Serial.println("Underload");
      }
      return;  // Do not update lastMeasurement
    }
    if (compareArrays(&dataBuffer[2], OFF_PATTERN, 16)) {
      if (serialEnabled) {
        Serial.println("OFF");
      }
      return;  // Do not update lastMeasurement
    }
    if (compareArrays(&dataBuffer[2], INIT_PATTERN, 16)) {
      if (serialEnabled) {
        Serial.println("Init");
      }
      return;  // Do not update lastMeasurement
    }
    if (compareArrays(&dataBuffer[2], VERSION_PATTERN, 16)) {
      if (serialEnabled) {
        Serial.println("Version");
      }
      return;  // Do not update lastMeasurement
    }


    if (localMsgComplete && localDataCount > 2) {
      lastDecode = decodePCFData(&dataBuffer[2], localDataCount - 2);
      if (lastDecode.valid) {
        //lastMeasurement = lastDecode.value;
        //measurementAvailable = true;
        processDecodedResult(lastDecode, millis());
      } else {
        if (serialEnabled) {
          Serial.println("Invalid reading discarded.");
        }
      }
    }
  }
}

DecodeResult decodePCFData(const uint8_t data[], size_t len) {
  DecodeResult result = {};
  // Initialize defaults:
  result.overload = false;
  result.underload = false;
  result.stable = false;
  result.negative = false;
  result.value = 0.0f;
  strcpy(result.text, "");  // empty string initially
  result.valid = true;      // assume valid unless proven otherwise

  // 1) Check Overload / Underload patterns (assuming data[] is 16 bytes of display data).
  //    If you only have 14 bytes or variable length, adapt accordingly.

  /*
  // We’ll assume 16 is always valid:
  if (len >= 16) {
    if (compareArrays(data, OVERLOAD_PATTERN, 16)) {
      result.overload = true;
      strcpy(result.text, "OVERLOAD");
      result.valid = false;  // Discard
      return result;         // done
    }
    if (compareArrays(data, UNDERLOAD_PATTERN, 16)) {
      result.underload = true;
      strcpy(result.text, "UNDERLOAD");
      result.valid = false;  // Discard
      return result;         // done
    }
  }
*/


  // 2) Check stable bit #5
  //    (We interpret "isStable = NOT isSegmentOn(...)", as in your code.)
  bool isStable = !isSegmentOn(data, READING_STABLE_BIT);
  result.stable = isStable;


  // 3) Decode each digit into a character array
  //    (8 digits + null terminator.)
  char decodedDigits[9];
  bool decimalPts[8];
  for (int d = 0; d < 8; d++) {
    decimalPts[d] = false;
    decodedDigits[d] = ' ';  // default blank
  }
  decodedDigits[8] = '\0';

  // We iterate over all 8 digits
  for (int d = 0; d < 8; d++) {
    bool dpOn = false;
    int val = decodeOneDigit(data, DIGITS[d], dpOn);
    decimalPts[d] = dpOn;
    if (val == NEG_SIGN_ONLY) {
      // A special marker for "negative sign"
      // We'll store '-' in that digit
      decodedDigits[d] = '-';
      // mark negative in the struct
      result.negative = true;
      // decodedDigits[d] = ' ';
    } else if (val == -2) {
      // All segments off => ' ' (blank)
      decodedDigits[d] = ' ';
    } else if (val == -1) {
      // Invalid segment pattern => '?'
      decodedDigits[d] = '?';
    } else {
      // 0..9
      decodedDigits[d] = (char)('0' + val);
    }
  }


  // 4) Count the number of minus signs.
  int minusCount = 0;
  for (int d = 0; d < 8; d++) {
    if (decodedDigits[d] == '-') {
      minusCount++;
    }
  }
  if (minusCount > 1) {
    result.valid = false;  // More than one minus sign: discard this reading.
    strcpy(result.text, "INVALID");
    result.value = 0.0f;
    return result;
  }






  char temp[16];
  int idx = 0;
  for (int d = 0; d < 8 && idx < 15; d++) {
    char c = decodedDigits[d];
    // skip leading blanks unless there's a minus sign
    if (c == ' ' && idx == 0) {
      // leading blank => skip
      continue;
    }
    //if (c != ' ') {
    temp[idx++] = c;
    //}
    if (decimalPts[d] && idx < 15) {
      temp[idx++] = '.';
    }
  }
  temp[idx] = '\0';







  // If it's empty, let's say " " => we could store "0" or leave blank
  if (idx == 0) {
    strcpy(result.text, "         ");
  } else {
    strcpy(result.text, temp);
  }

  // Count the number of decimal points:
  int dpCount = 0;
  for (int d = 0; d < 8; d++) {
    if (decimalPts[d]) dpCount++;
  }
  if (dpCount > 1) {
    result.valid = false;  // More than one decimal point: discard this reading.
    strcpy(result.text, "INVALID");
    result.value = 0.0f;
    return result;
  }


  // First, count the number of numeric digits in result.text, discard if less than 3
  int digitCount = 0;
  for (int i = 0; i < (int)strlen(result.text); i++) {
    char c = result.text[i];
    if (c >= '0' && c <= '9') {
      digitCount++;
    }
  }
  if (digitCount < 3) {
    result.valid = false;  // Not enough numeric digits—discard the reading.
    strcpy(result.text, "INVALID");
    result.value = 0.0f;
    return result;
  }


  // Attempt to parse the numeric value from that string
  float parsedValue = 0.0f;
  char filtered[16];
  int j = 0;
  for (int i = 0; i < (int)strlen(result.text); i++) {
    char c = result.text[i];
    // keep digits, '.', '-',
    if ((c >= '0' && c <= '9') || c == '.') {
      filtered[j++] = c;
    }
  }
  filtered[j] = '\0';
  parsedValue = atof(filtered);

  if (result.negative) {
    parsedValue = -fabs(parsedValue);
  }
  // Save to struct
  result.value = parsedValue;

  // Done
  return result;
}
