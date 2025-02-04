// globals.h
#ifndef GLOBALS_H
#define GLOBALS_H

// Include required libraries:
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <DS3232RTC.h>   // DS3232RTC library
#include <TimeLib.h>     // Time functions

bool serialEnabled = false;

// A container for the decoded result.
struct DecodeResult {
  bool valid;       // true if the reading is valid, false if it should be discarded
  bool overload;    // true if overload detected
  bool underload;   // true if underload detected
  bool stable;      // whether stable bit is set
  bool negative;    // if a negative sign was detected in the leftmost digit
  float value;      // parsed numeric value from the digits
  char text[16];    // textual representation (e.g. "12.34", "-0.12", "OVERLOAD", etc.)
};


// A flag & storage for new measurement
static bool measurementAvailable = false;
static DecodeResult lastDecode;
float lastMeasurement;

static unsigned long startupMillis;
static const unsigned long SERIAL_WINDOW_MS = 10000;


#define SDA_PIN 2  // 21 on ESP32, I2C SDA pin, gelb, links on PCB
#define SCL_PIN 3  // 22 on ESP32, I2C SCL pin, orange, rechts on PCB
#define LED_ONBOARD 13

// I2C address to sniff (7-bit address, 0x38 for PCF8576)
#define SNIFF_ADDR 0x38

// Buffer to hold the received I2C data
#define MAX_BUFFER_SIZE 32

// Use a global RTC object (make sure every file that uses it includes globals.h)
DS3232RTC myRTC;
time_t pendingTime = 0;      // Default initialization
// Flag for whether the RTC was successfully initialized.
bool rtcInitialized = false;
bool havePendingTime = false;
// For our DST handling:
unsigned long t, t_1;
int UTC_offset = 1;  // 1 = CET, 2 = CEST 
struct TimestampData {
  unsigned long timestamp;
  int offset;
};

// Define the array with pairs of [timestamp, UTC_offset]
// https://www.linker.ch/eigenlink/sommerzeit_winterzeit.htm
const TimestampData timestampDataArray[] = {
  { 1679792400, 2 },  // 26.03.2023 02:00 UTC+1
  { 1698541200, 1 },  // 29.10.2023 03:00 UTC+2
  { 1711846800, 2 },  // 31.03.2024 02:00 UTC+1
  { 1729990800, 1 },  // 27.10.2024 03:00 UTC+2
  { 1743296400, 2 },  // 30.03.2025 02:00 UTC+1
  { 1761440400, 1 },  // 26.10.2025 03:00 UTC+2
  { 1774746000, 2 },  // 29.03.2026 02:00 UTC+1
  { 1792890000, 1 },  // 25.10.2026 03:00 UTC+2
  { 1806199200, 2 },  // 28-03-2027 02:00
  { 1824951600, 1 },  // 31-10-2027 03:00
  { 1837648800, 2 },  // 26-03-2028 02:00
  { 1856401200, 1 },  // 29-10-2028 03:00
  { 1869098400, 2 },  // 25-03-2029 02:00
  { 1887850800, 1 },  // 28-10-2029 03:00
  { 1901152800, 2 },  // 31-03-2030 02:00
  { 1919300400, 1 }   // 27-10-2030 03:00
};


static bool sdOK = false;
static File currentLogFile;
static uint16_t nextFileNumber = 0;
static const int SD_CHIP_SELECT = 10; // or your SD CS pin
const float TOLERANCE = 0.001; // Adjust as needed
// Timeout in milliseconds – if no new measurement for 10 seconds, flush the block.
const unsigned long BLOCK_TIMEOUT = 10000;
static bool blockActive = false;
static DecodeResult blockRes;
static unsigned long blockStartTime = 0;
static unsigned long blockEndTime = 0;
static float lastLoggedValue = 0;

static volatile bool messageComplete = false;
static volatile size_t dataCount = 0;
static volatile uint8_t dataBuffer[32];



// -------------------------------
// USER’S BIT MAPPINGS
// -------------------------------

// Additional segments or symbols:
static const int READING_STABLE_BIT = 5;
//static const int SIGN_BIT = 53;
//bitmask is 0x40 (A=bit0 = 0, B=bit1 = 0, C=bit2 = 0, D=bit3 = 0, E=bit4 = 0, F=bit5 = 0, G=bit6 = 1), that means “only G is on” → negative sign
static const int NEG_SIGN_ONLY = -9;  // a special code to indicate “only G=1”

static const int UNIT_K_BIT = 64;
static const int UNIT_G_BIT = 68;
static const int UNIT_PCS_BIT = 79;
static const int UNIT_PERCENT_BIT = 89;
// Overload pattern, Underload pattern (16 bytes each)
static const uint8_t OVERLOAD_PATTERN[16] = {
  0x00, 0x00, 0x00, 0x42, 0x00, 0x20, 0x02, 0x00,
  0x00, 0x22, 0x00, 0x20, 0x02, 0x00, 0x00, 0x00
};
static const uint8_t UNDERLOAD_PATTERN[16] = {
  0x00, 0x00, 0x00, 0x11, 0x00, 0x10, 0x01, 0x00,
  0x00, 0x41, 0x00, 0x10, 0x01, 0x00, 0x00, 0x00
};
static const uint8_t OFF_PATTERN[16] = {
  0x00, 0x00, 0x00, 0x53, 0x65, 0x60, 0x56, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
static const uint8_t INIT_PATTERN[16] = {
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};
static const uint8_t VERSION_PATTERN[16] = {
  0x00, 0x00, 0x07, 0x17, 0x25, 0x74, 0x00, 0x00,
  0x00, 0x60, 0x06, 0x44, 0x70, 0x00, 0x00, 0x00
};

// Digit 1 segments (bit indices)
static const int DIGIT1_A = 1;
static const int DIGIT1_B = 13;
static const int DIGIT1_C = 14;
static const int DIGIT1_D = 0;
static const int DIGIT1_E = 4;
static const int DIGIT1_F = 6;
static const int DIGIT1_G = 2;
static const int DIGIT1_DP = 12;

// Digit 2 segments
static const int DIGIT2_A = 21;
static const int DIGIT2_B = 17;
static const int DIGIT2_C = 18;
static const int DIGIT2_D = 20;
static const int DIGIT2_E = 8;
static const int DIGIT2_F = 10;
static const int DIGIT2_G = 22;
static const int DIGIT2_DP = 16;

// Digit 3 segments
static const int DIGIT3_A = 25;
static const int DIGIT3_B = 37;
static const int DIGIT3_C = 38;
static const int DIGIT3_D = 24;
static const int DIGIT3_E = 28;
static const int DIGIT3_F = 30;
static const int DIGIT3_G = 26;
static const int DIGIT3_DP = 36;

// Digit 4 segments
static const int DIGIT4_A = 45;
static const int DIGIT4_B = 41;
static const int DIGIT4_C = 42;
static const int DIGIT4_D = 44;
static const int DIGIT4_E = 32;
static const int DIGIT4_F = 34;
static const int DIGIT4_G = 46;
static const int DIGIT4_DP = 40;

// Digit 5 segments
static const int DIGIT5_A = 49;
static const int DIGIT5_B = 105;
static const int DIGIT5_C = 106;
static const int DIGIT5_D = 48;
static const int DIGIT5_E = 52;
static const int DIGIT5_F = 54;
static const int DIGIT5_G = 50;
static const int DIGIT5_DP = 104;

// Digit 6 segments
static const int DIGIT6_A = 97;
static const int DIGIT6_B = 101;
static const int DIGIT6_C = 102;
static const int DIGIT6_D = 96;
static const int DIGIT6_E = 108;
static const int DIGIT6_F = 110;
static const int DIGIT6_G = 98;
static const int DIGIT6_DP = 100;

// Digit 7 segments
static const int DIGIT7_A = 93;
static const int DIGIT7_B = 81;
static const int DIGIT7_C = 82;
static const int DIGIT7_D = 92;
static const int DIGIT7_E = 88;
static const int DIGIT7_F = 90;
static const int DIGIT7_G = 94;
static const int DIGIT7_DP = 80;

// Digit 8 segments
static const int DIGIT8_A = 73;
static const int DIGIT8_B = 77;
static const int DIGIT8_C = 78;
static const int DIGIT8_D = 72;
static const int DIGIT8_E = 84;
static const int DIGIT8_F = 86;
static const int DIGIT8_G = 74;
static const int DIGIT8_DP = 76;

// We can group them in a struct or arrays, but here’s one approach:
struct DigitMap {
  int segA;
  int segB;
  int segC;
  int segD;
  int segE;
  int segF;
  int segG;
  int segDP;  // decimal point
};

// Put them in an array so we can index by digit = 0..7
static const DigitMap DIGITS[8] = {
  { DIGIT1_A, DIGIT1_B, DIGIT1_C, DIGIT1_D, DIGIT1_E, DIGIT1_F, DIGIT1_G, DIGIT1_DP },
  { DIGIT2_A, DIGIT2_B, DIGIT2_C, DIGIT2_D, DIGIT2_E, DIGIT2_F, DIGIT2_G, DIGIT2_DP },
  { DIGIT3_A, DIGIT3_B, DIGIT3_C, DIGIT3_D, DIGIT3_E, DIGIT3_F, DIGIT3_G, DIGIT3_DP },
  { DIGIT4_A, DIGIT4_B, DIGIT4_C, DIGIT4_D, DIGIT4_E, DIGIT4_F, DIGIT4_G, DIGIT4_DP },
  { DIGIT5_A, DIGIT5_B, DIGIT5_C, DIGIT5_D, DIGIT5_E, DIGIT5_F, DIGIT5_G, DIGIT5_DP },
  { DIGIT6_A, DIGIT6_B, DIGIT6_C, DIGIT6_D, DIGIT6_E, DIGIT6_F, DIGIT6_G, DIGIT6_DP },
  { DIGIT7_A, DIGIT7_B, DIGIT7_C, DIGIT7_D, DIGIT7_E, DIGIT7_F, DIGIT7_G, DIGIT7_DP },
  { DIGIT8_A, DIGIT8_B, DIGIT8_C, DIGIT8_D, DIGIT8_E, DIGIT8_F, DIGIT8_G, DIGIT8_DP }
};

// Standard 7-seg patterns for digits 0..9 (dp not included here):
// We’ll define each pattern as 7 bits: G F E D C B A in that order
// or (A,B,C,D,E,F,G) => 1 means "segment on"
// e.g. 0 => (A,B,C, D,E,F)=on, G=off => binary 0111111 => 0x3F
// but you can store them in any order as long as you're consistent.
static const uint8_t SEG_PATTERN[10] = {
  0x3F,  // 0 => A,B,C,D,E,F on, G=off => 0b0111111
  0x06,  // 1 => B,C
  0x5B,  // 2 => A,B,G,E,D
  0x4F,  // 3 => A,B,C,D,G
  0x66,  // 4 => F,G,B,C
  0x6D,  // 5 => A,F,G,C,D
  0x7D,  // 6 => A,F,E,D,C,G
  0x07,  // 7 => A,B,C
  0x7F,  // 8 => all
  0x6F   // 9 => A,B,C,D,F,G
};


#endif