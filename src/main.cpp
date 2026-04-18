#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <SpeeduinoProtocol.h>
#include <string.h>

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
HardwareSerial &speeduinoSerial = Serial1;  // RX1 = 19, TX1 = 18

#ifndef ENABLE_GPS
#define ENABLE_GPS 1
#endif

#if ENABLE_GPS
HardwareSerial &gpsSerial = Serial2;  // RX2 = 17, TX2 = 16
#endif

// Module for reading Speeduino's serial3 port and displaying it on a 20x4 LCD.
// This sketch expects the ECU secondary serial protocol to be set to
// "Generic (Fixed List)" in TunerStudio.
//
// Lex Sewuster (aka Zeiberstein)
// 20200918 | initial creation
// 20201021 | added moving bars
// 20210103 | fixed temperature reading
// 20260402 | migrated to enhanced "n" command and added fuel pressure
// 20260403 | removed moving bars and moved fuel pressure to last line
// 20260412 | cleaned up naming and payload access for display fields
// 20260412 | improved serial buffer cleanup before and after packet reads
// 20260412 | switched advance display to inline LCD degree escape and hid status 0
//
// See https://speeduino.com/wiki/index.php/Secondary_Serial_IO_interface

constexpr int NUM_DISPLAY_COLS = 20;
constexpr int NUM_DISPLAY_ROWS = 4;
constexpr byte MIN_DISPLAY_PAYLOAD_LENGTH = FUEL_PRESSURE + 1;
constexpr byte GPS_INDICATOR_WIDTH = 13;

#if ENABLE_GPS
constexpr unsigned long GPS_BAUD_RATE = 9600UL;
constexpr unsigned long GPS_STALE_INTERVAL = 3000UL;
constexpr unsigned long GPS_CLOCK_SYNC_INTERVAL = 60000UL;
constexpr unsigned long GPS_SECONDS_PER_DAY = 86400UL;
constexpr byte GPS_MAX_BYTES_PER_SERVICE = 32;
constexpr byte GPS_NMEA_BUFFER_SIZE = 96;
#endif

struct SpeeduinoPacketResult {
  byte statusCode;
  byte payloadLength;
  const byte *payload;
};

struct SpeeduinoSnapshot {
  uint16_t rpm;
  byte advance;
  int coolantC;
  int iatC;
  uint16_t mapKpa;
  byte idleValve;
  byte tps;
  byte corrections;
  byte afr10;
  byte battery10;
  byte engineStatusBits;
  long fuelPressure10;
};

enum SpeeduinoReadPhase {
  SPEEDUINO_READ_PHASE_IDLE,
  SPEEDUINO_READ_PHASE_STARTING_REQUEST,
  SPEEDUINO_READ_PHASE_READING_PACKET,
  SPEEDUINO_READ_PHASE_DONE
};

struct SpeeduinoReadState {
  SpeeduinoReadPhase phase;
  byte statusCode;
  int bytesInPacket;
  int expectedPacketSize;
  bool hasDiscardedBufferedBytes;
  unsigned long readStart;
};

struct SpeeduinoSerialFlushState {
  bool hasDiscardedBufferedBytes;
  unsigned long flushStart;
};

enum SpeeduinoPollPhase {
  SPEEDUINO_POLL_PHASE_IDLE,
  SPEEDUINO_POLL_PHASE_FLUSHING_UNWANTED_BYTES,
  SPEEDUINO_POLL_PHASE_READING_PACKET,
  SPEEDUINO_POLL_PHASE_CHECKING_ADDITIONAL_BYTES
};

enum SpeeduinoPollFinishMode {
  SPEEDUINO_POLL_FINISH_PACKET_STATUS,
  SPEEDUINO_POLL_FINISH_CURRENT_STATUS
};

struct SpeeduinoPollState {
  SpeeduinoPollPhase phase;
  SpeeduinoPollFinishMode finishMode;
  SpeeduinoSerialFlushState flushState;
  SpeeduinoPacketResult packetResult;
};

#if ENABLE_GPS
enum GpsFixState {
  GPS_FIX_NO_DATA_YET,
  GPS_FIX_NO_VALID_FIX,
  GPS_FIX_STALE,
  GPS_FIX_VALID
};

struct GpsState {
  GpsFixState fixState;
  bool hasReceivedSentence;
  bool hasValidFix;
  bool hasDate;
  byte utcHour;
  byte utcMinute;
  byte utcDay;
  byte utcMonth;
  uint16_t utcYear;
  byte utcSecond;
  unsigned int speedKmh;
  unsigned long lastSentenceMillis;
  unsigned long lastValidFixMillis;
  char sentence[GPS_NMEA_BUFFER_SIZE];
  byte sentenceLength;
};

struct GpsClockState {
  bool hasTime;
  unsigned long localSecondsOfDay;
  unsigned long setMillis;
};
#endif

byte packet[MAX_PACKET_SIZE];  // More than enough for the maximum payload plus header.
byte payloadLength = 0;
SpeeduinoReadState speeduinoReadState;
SpeeduinoPollState speeduinoPollState;
unsigned long lastSpeeduinoPollMillis = 0;

#if ENABLE_GPS
GpsState gpsState;
GpsClockState gpsClockState;
#endif

SpeeduinoPacketResult makePacketResult(byte statusCode) {
  SpeeduinoPacketResult result = {statusCode, payloadLength, packet + PAYLOAD_OFFSET};
  return result;
}

void resetSpeeduinoReadState(SpeeduinoReadState &readState, unsigned long readStart) {
  readState.phase = SPEEDUINO_READ_PHASE_IDLE;
  readState.statusCode = PACKET_STATUS_OK;
  readState.bytesInPacket = 0;
  readState.expectedPacketSize = -1;
  readState.hasDiscardedBufferedBytes = false;
  readState.readStart = readStart;
}

void resetSpeeduinoPacketBuffer() {
  memset(packet, 0, sizeof(packet));
  payloadLength = 0;
}

bool hasValidSpeeduinoPacketHeader() {
  return (packet[0] == 'n') && (packet[1] == '2');
}

bool hasMinimumSpeeduinoDisplayPayloadLength(byte packetPayloadLength) {
  return packetPayloadLength >= MIN_DISPLAY_PAYLOAD_LENGTH;
}

byte validateSpeeduinoPacketHeaderAndSetExpectedSize(int &expectedPacketSize) {
  if (!hasValidSpeeduinoPacketHeader()) {
    return PACKET_STATUS_HEADER_INVALID;
  }

  payloadLength = packet[2];
  if (!hasMinimumSpeeduinoDisplayPayloadLength(payloadLength)) {
    return PACKET_STATUS_PAYLOAD_TOO_SHORT;
  }

  expectedPacketSize = HEADER_SIZE + payloadLength;
  return PACKET_STATUS_OK;
}

void storeIncomingSpeeduinoPacketByte(byte incomingByte, int &bytesInPacket) {
  if (bytesInPacket < MAX_PACKET_SIZE) {
    packet[bytesInPacket] = incomingByte;
  }
  bytesInPacket++;
}

bool hasCompleteExpectedSpeeduinoPacket(int bytesInPacket, int expectedPacketSize) {
  return (expectedPacketSize >= HEADER_SIZE) && (bytesInPacket == expectedPacketSize);
}

bool processIncomingSpeeduinoPacketByte(SpeeduinoReadState &readState, byte incomingByte) {
  storeIncomingSpeeduinoPacketByte(incomingByte, readState.bytesInPacket);

  if (readState.bytesInPacket == HEADER_SIZE) {
    readState.statusCode = validateSpeeduinoPacketHeaderAndSetExpectedSize(readState.expectedPacketSize);
    if (readState.statusCode != PACKET_STATUS_OK) {
      readState.phase = SPEEDUINO_READ_PHASE_DONE;
      return true;
    }
  }

  if (hasCompleteExpectedSpeeduinoPacket(readState.bytesInPacket, readState.expectedPacketSize)) {
    readState.statusCode = PACKET_STATUS_OK;
    readState.phase = SPEEDUINO_READ_PHASE_DONE;
    return true;
  }

  return false;
}

byte speeduinoPacketStatusAfterRead(int bytesInPacket, int expectedPacketSize, bool hasDiscardedBufferedBytes) {
  if (bytesInPacket < HEADER_SIZE) {
    return PACKET_STATUS_HEADER_INCOMPLETE;
  }

  if (bytesInPacket < expectedPacketSize) {
    return PACKET_STATUS_INCOMPLETE;
  }

  // This code assumes Speeduino's fixed-list "n2" payload keeps the existing field
  // offsets stable and only ever grows with appended fields, not by removing fields
  // before FUEL_PRESSURE.
  if (hasDiscardedBufferedBytes) {
    return PACKET_STATUS_TOO_LONG;
  }

  return PACKET_STATUS_OK;
}

SpeeduinoSnapshot decodeSpeeduinoSnapshot(const byte *payload) {
  SpeeduinoSnapshot snapshot;

  snapshot.rpm = (uint16_t(payload[RPM_HIGH]) << 8) | payload[RPM_LOW];
  snapshot.advance = payload[ADVANCE];
  snapshot.coolantC = ((int)payload[COOLANT_WITH_OFFSET]) - TEMPERATURE_OFFSET;
  snapshot.iatC = ((int)payload[IAT_WITH_OFFSET]) - TEMPERATURE_OFFSET;
  snapshot.mapKpa = (uint16_t(payload[MAP_HIGH]) << 8) | payload[MAP_LOW];
  snapshot.idleValve = payload[IDLE_VALVE];
  snapshot.tps = payload[TPS];
  snapshot.corrections = payload[CORRECTIONS];
  snapshot.afr10 = payload[O2];
  snapshot.battery10 = payload[BATTERY10];
  snapshot.engineStatusBits = payload[ENGINE_STATUS];
  snapshot.fuelPressure10 = (long(payload[FUEL_PRESSURE]) * 6895L + 5000L) / 10000L;

  return snapshot;
}

void lcdprint(byte col, byte row, const char *text) {
  lcd.setCursor(col, row);
  lcd.print(text);
}

void lcdprint(byte col, byte row, int num, const char *fmt) {
  static char numBuf[21];
  snprintf(numBuf, sizeof(numBuf), fmt, num);
  lcdprint(col, row, numBuf);
}

void lcdprintTenths(byte col, byte row, int value10, const char *suffix, byte wholeWidth = 2) {
  static char numBuf[21];
  if (wholeWidth <= 1) {
    snprintf(numBuf, sizeof(numBuf), "%d.%1d%s", value10 / 10, abs(value10) % 10, suffix);
  }
  else {
    snprintf(numBuf, sizeof(numBuf), "%2d.%1d%s", value10 / 10, abs(value10) % 10, suffix);
  }
  lcdprint(col, row, numBuf);
}

const char *engineStatus(byte status) {
  static char buf[7] = {' ', ' ', ' ', ' ', ' ', ' ', '\0'};
  int bufPos = 5;

  for (byte bitNr = 0; bitNr < ENGINE_STATUS_CHAR_COUNT; bitNr++) {
    if (status & (1 << bitNr)) {
      buf[bufPos] = ENGINE_STATUS_CHARS[bitNr];
      bufPos--;
    }
  }

  for (int i = bufPos; i >= 0; i--) {
    buf[i] = ' ';
  }

  return buf;
}

bool isDigitChar(char c) {
  return (c >= '0') && (c <= '9');
}

void copyPaddedText(char *dest, byte width, const char *text) {
  byte i = 0;
  while ((i < width) && (text[i] != '\0')) {
    dest[i] = text[i];
    i++;
  }

  while (i < width) {
    dest[i] = ' ';
    i++;
  }

  dest[width] = '\0';
}

#if ENABLE_GPS
bool isNmeaType(const char *sentence, const char *type) {
  return (sentence[0] == '$') &&
         (sentence[3] == type[0]) &&
         (sentence[4] == type[1]) &&
         (sentence[5] == type[2]);
}

const char *nmeaFieldStart(const char *sentence, byte fieldIndex) {
  const char *field = sentence;
  if (*field == '$') {
    field++;
  }

  for (byte currentField = 0; currentField < fieldIndex; currentField++) {
    while ((*field != '\0') && (*field != ',') && (*field != '*')) {
      field++;
    }

    if (*field != ',') {
      return nullptr;
    }

    field++;
  }

  return field;
}

byte nmeaFieldLength(const char *field) {
  byte length = 0;
  while ((field[length] != '\0') && (field[length] != ',') && (field[length] != '*')) {
    length++;
  }
  return length;
}

bool parseGpsUtcTime(const char *field, byte fieldLength, byte &utcHour, byte &utcMinute, byte &utcSecond) {
  if (fieldLength < 4) {
    return false;
  }

  for (byte i = 0; i < 4; i++) {
    if (!isDigitChar(field[i])) {
      return false;
    }
  }

  byte parsedHour = ((field[0] - '0') * 10) + (field[1] - '0');
  byte parsedMinute = ((field[2] - '0') * 10) + (field[3] - '0');
  byte parsedSecond = 0;

  if (fieldLength >= 6) {
    if (!isDigitChar(field[4]) || !isDigitChar(field[5])) {
      return false;
    }
    parsedSecond = ((field[4] - '0') * 10) + (field[5] - '0');
  }

  if ((parsedHour > 23) || (parsedMinute > 59) || (parsedSecond > 59)) {
    return false;
  }

  utcHour = parsedHour;
  utcMinute = parsedMinute;
  utcSecond = parsedSecond;
  return true;
}

bool parseGpsDate(const char *field, byte fieldLength, byte &utcDay, byte &utcMonth, uint16_t &utcYear) {
  if (fieldLength < 6) {
    return false;
  }

  for (byte i = 0; i < 6; i++) {
    if (!isDigitChar(field[i])) {
      return false;
    }
  }

  byte parsedDay = ((field[0] - '0') * 10) + (field[1] - '0');
  byte parsedMonth = ((field[2] - '0') * 10) + (field[3] - '0');
  uint16_t parsedYear = 2000U + (((field[4] - '0') * 10) + (field[5] - '0'));

  if ((parsedDay < 1) || (parsedDay > 31) || (parsedMonth < 1) || (parsedMonth > 12)) {
    return false;
  }

  utcDay = parsedDay;
  utcMonth = parsedMonth;
  utcYear = parsedYear;
  return true;
}

byte dayOfWeek(byte day, byte month, uint16_t year) {
  static const byte monthOffsets[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};

  if (month < 3) {
    year--;
  }

  return (year + (year / 4) - (year / 100) + (year / 400) + monthOffsets[month - 1] + day) % 7;
}

byte lastSundayOfMonth(byte month, uint16_t year) {
  return 31 - dayOfWeek(31, month, year);
}

bool isCentralEuropeanSummerTimeUtc(byte day, byte month, uint16_t year, byte hour) {
  if ((month < 3) || (month > 10)) {
    return false;
  }

  if ((month > 3) && (month < 10)) {
    return true;
  }

  byte lastSunday = lastSundayOfMonth(month, year);

  if (month == 3) {
    if (day < lastSunday) {
      return false;
    }
    if (day > lastSunday) {
      return true;
    }
    return hour >= 1;
  }

  if (day < lastSunday) {
    return true;
  }
  if (day > lastSunday) {
    return false;
  }
  return hour < 1;
}

byte gpsLocalTimeOffsetHours(byte utcHour) {
  if (!gpsState.hasDate) {
    return 0;
  }

  if (isCentralEuropeanSummerTimeUtc(gpsState.utcDay, gpsState.utcMonth, gpsState.utcYear, utcHour)) {
    return 2;
  }

  return 1;
}

void syncGpsClock(byte localHour, byte localMinute, byte localSecond, unsigned long now) {
  gpsClockState.localSecondsOfDay = (unsigned long)localHour * 3600UL;
  gpsClockState.localSecondsOfDay += (unsigned long)localMinute * 60UL;
  gpsClockState.localSecondsOfDay += localSecond;
  gpsClockState.setMillis = now;
  gpsClockState.hasTime = true;
}

unsigned long currentGpsClockLocalSecondsOfDay(unsigned long now) {
  unsigned long elapsedSeconds = (now - gpsClockState.setMillis) / 1000UL;
  return (gpsClockState.localSecondsOfDay + elapsedSeconds) % GPS_SECONDS_PER_DAY;
}

bool shouldSyncGpsClock(unsigned long now) {
  return !gpsClockState.hasTime || ((now - gpsClockState.setMillis) >= GPS_CLOCK_SYNC_INTERVAL);
}

bool getGpsDisplayTime(byte &displayHour, byte &displayMinute) {
  if (!gpsClockState.hasTime) {
    return false;
  }

  unsigned long localSecondsOfDay = currentGpsClockLocalSecondsOfDay(millis());
  displayHour = localSecondsOfDay / 3600UL;
  displayMinute = (localSecondsOfDay % 3600UL) / 60UL;
  return true;
}

unsigned int parseGpsSpeedKmh(const char *field, byte fieldLength) {
  unsigned long knots100 = 0;
  byte decimals = 0;
  bool hasDigit = false;
  bool decimalSeen = false;

  for (byte i = 0; i < fieldLength; i++) {
    char c = field[i];
    if (c == '.') {
      decimalSeen = true;
      continue;
    }

    if (!isDigitChar(c)) {
      break;
    }

    if (!decimalSeen) {
      knots100 = (knots100 * 10UL) + (c - '0');
      hasDigit = true;
      continue;
    }

    if (decimals < 2) {
      knots100 = (knots100 * 10UL) + (c - '0');
      decimals++;
      hasDigit = true;
    }
  }

  if (!hasDigit) {
    return 0;
  }

  if (!decimalSeen) {
    knots100 *= 100UL;
  }
  else {
    while (decimals < 2) {
      knots100 *= 10UL;
      decimals++;
    }
  }

  unsigned long kmh = ((knots100 * 1852UL) + 50000UL) / 100000UL;
  if (kmh > 999UL) {
    return 999;
  }

  return (unsigned int)kmh;
}

void refreshGpsFixState(unsigned long now) {
  if (!gpsState.hasReceivedSentence) {
    gpsState.fixState = GPS_FIX_NO_DATA_YET;
    return;
  }

  if ((now - gpsState.lastSentenceMillis) > GPS_STALE_INTERVAL) {
    gpsState.fixState = GPS_FIX_STALE;
    return;
  }

  if (!gpsState.hasValidFix) {
    gpsState.fixState = GPS_FIX_NO_VALID_FIX;
    return;
  }

  if ((now - gpsState.lastValidFixMillis) > GPS_STALE_INTERVAL) {
    gpsState.fixState = GPS_FIX_STALE;
    return;
  }

  gpsState.fixState = GPS_FIX_VALID;
}

void processGpsRmcSentence(const char *sentence, unsigned long now) {
  const char *timeField = nmeaFieldStart(sentence, 1);
  const char *statusField = nmeaFieldStart(sentence, 2);
  const char *speedField = nmeaFieldStart(sentence, 7);
  const char *dateField = nmeaFieldStart(sentence, 9);

  if ((statusField == nullptr) || (nmeaFieldLength(statusField) == 0)) {
    return;
  }

  if (statusField[0] != 'A') {
    // Keep the last valid fix until it becomes stale to avoid display flicker on a single invalid RMC sentence.
    return;
  }

  gpsState.hasValidFix = true;
  gpsState.lastValidFixMillis = now;

  if (dateField != nullptr) {
    gpsState.hasDate = parseGpsDate(
      dateField,
      nmeaFieldLength(dateField),
      gpsState.utcDay,
      gpsState.utcMonth,
      gpsState.utcYear
    );
  }

  if (timeField != nullptr) {
    if (parseGpsUtcTime(
      timeField,
      nmeaFieldLength(timeField),
      gpsState.utcHour,
      gpsState.utcMinute,
      gpsState.utcSecond
    ) && gpsState.hasDate) {
      byte localHour = (gpsState.utcHour + gpsLocalTimeOffsetHours(gpsState.utcHour)) % 24;
      if (shouldSyncGpsClock(now)) {
        syncGpsClock(localHour, gpsState.utcMinute, gpsState.utcSecond, now);
      }
    }
  }

  if (speedField != nullptr) {
    gpsState.speedKmh = parseGpsSpeedKmh(speedField, nmeaFieldLength(speedField));
  }
}

void processGpsSentence(const char *sentence, unsigned long now) {
  gpsState.hasReceivedSentence = true;
  gpsState.lastSentenceMillis = now;

  if (isNmeaType(sentence, "RMC")) {
    processGpsRmcSentence(sentence, now);
  }

  refreshGpsFixState(now);
}

void processGpsChar(char incomingChar, unsigned long now) {
  if (incomingChar == '$') {
    gpsState.sentenceLength = 0;
  }

  if ((incomingChar == '\r') || (incomingChar == '\n')) {
    if (gpsState.sentenceLength > 0) {
      gpsState.sentence[gpsState.sentenceLength] = '\0';
      processGpsSentence(gpsState.sentence, now);
      gpsState.sentenceLength = 0;
    }
    return;
  }

  if ((gpsState.sentenceLength == 0) && (incomingChar != '$')) {
    return;
  }

  if (gpsState.sentenceLength >= (GPS_NMEA_BUFFER_SIZE - 1)) {
    gpsState.sentenceLength = 0;
    return;
  }

  gpsState.sentence[gpsState.sentenceLength] = incomingChar;
  gpsState.sentenceLength++;
}

void serviceGps() {
  unsigned long now = millis();
  byte processedBytes = 0;

  while ((gpsSerial.available() > 0) && (processedBytes < GPS_MAX_BYTES_PER_SERVICE)) {
    processGpsChar((char)gpsSerial.read(), now);
    processedBytes++;
  }

  refreshGpsFixState(millis());
}

void setupGps() {
  gpsState.fixState = GPS_FIX_NO_DATA_YET;
  gpsState.hasReceivedSentence = false;
  gpsState.hasValidFix = false;
  gpsState.hasDate = false;
  gpsState.utcSecond = 0;
  gpsState.sentenceLength = 0;
  gpsClockState.hasTime = false;
  gpsClockState.localSecondsOfDay = 0;
  gpsClockState.setMillis = 0;
  gpsSerial.begin(GPS_BAUD_RATE);
}
#endif

void renderGpsIndicator() {
  char indicator[GPS_INDICATOR_WIDTH + 1];

#if ENABLE_GPS
  refreshGpsFixState(millis());

  byte displayHour = 0;
  byte displayMinute = 0;
  bool hasDisplayTime = getGpsDisplayTime(displayHour, displayMinute);

  if ((gpsState.fixState == GPS_FIX_VALID) && hasDisplayTime) {
    char text[GPS_INDICATOR_WIDTH + 1];
    snprintf(
      text,
      sizeof(text),
      "%02u:%02u %3ukm/h",
      (unsigned int)displayHour,
      (unsigned int)displayMinute,
      gpsState.speedKmh
    );
    copyPaddedText(indicator, GPS_INDICATOR_WIDTH, text);
  }
  else if (hasDisplayTime) {
    char text[GPS_INDICATOR_WIDTH + 1];
    if ((gpsState.fixState == GPS_FIX_STALE) || (gpsState.fixState == GPS_FIX_NO_DATA_YET)) {
      snprintf(
        text,
        sizeof(text),
        "%02u:%02u  GPS?",
        (unsigned int)displayHour,
        (unsigned int)displayMinute
      );
    }
    else {
      snprintf(
        text,
        sizeof(text),
        "%02u:%02u",
        (unsigned int)displayHour,
        (unsigned int)displayMinute
      );
    }
    copyPaddedText(indicator, GPS_INDICATOR_WIDTH, text);
  }
  else if (gpsState.fixState == GPS_FIX_STALE) {
    copyPaddedText(indicator, GPS_INDICATOR_WIDTH, "GPS?");
  }
  else if (gpsState.fixState == GPS_FIX_NO_VALID_FIX) {
    copyPaddedText(indicator, GPS_INDICATOR_WIDTH, "");
  }
  else {
    copyPaddedText(indicator, GPS_INDICATOR_WIDTH, "GPS?");
  }
#else
  copyPaddedText(indicator, GPS_INDICATOR_WIDTH, "");
#endif

  lcdprint(7, 3, indicator);
}

void renderSpeeduinoSnapshot(const SpeeduinoSnapshot &snapshot) {
  lcdprint(0, 0, snapshot.rpm, "%4drpm ");
  lcdprint(8, 0, snapshot.advance, "%3d\xDF ");  // The HD44780 LCD expects 0xDF for the degree symbol.
  lcdprint(12, 0, snapshot.coolantC, "%3dC");
  lcdprint(16, 0, snapshot.iatC, "%3dC");

  lcdprint(0, 1, snapshot.mapKpa, "%4dkPa ");
  lcdprint(8, 1, snapshot.idleValve, "%3d ");
  lcdprint(12, 1, snapshot.tps, "%3d ");
  lcdprint(16, 1, snapshot.corrections, "%3d%%");

  lcdprintTenths(0, 2, snapshot.afr10, "afr ");
  lcdprintTenths(8, 2, snapshot.battery10, "V ");
  lcdprint(14, 2, engineStatus(snapshot.engineStatusBits));

  lcdprintTenths(0, 3, snapshot.fuelPressure10, "bar ", 1);
}

void serviceBackgroundTasks() {
#if ENABLE_GPS
  serviceGps();
#endif
}

void idleBackgroundService() {
  serviceBackgroundTasks();
}

void startSpeeduinoSerialFlush(SpeeduinoSerialFlushState &flushState, unsigned long now) {
  flushState.hasDiscardedBufferedBytes = false;
  flushState.flushStart = now;
}

bool serviceSpeeduinoSerialFlush(SpeeduinoSerialFlushState &flushState, unsigned long now) {
  while (speeduinoSerial.available() > 0) {
    flushState.hasDiscardedBufferedBytes = true;
    speeduinoSerial.read();
  }

  return (now - flushState.flushStart) >= UNEXPECTED_BYTES_WAITING_INTERVAL;
}

bool serviceSpeeduinoPacketReadStep(SpeeduinoReadState &readState) {
  if (speeduinoSerial.available() == 0) {
    idleBackgroundService();
    return false;
  }

  return processIncomingSpeeduinoPacketByte(readState, speeduinoSerial.read());
}

void startSpeeduinoPacketRequest(SpeeduinoReadState &readState) {
  readState.phase = SPEEDUINO_READ_PHASE_STARTING_REQUEST;
  speeduinoSerial.print("n");

  readState.phase = SPEEDUINO_READ_PHASE_READING_PACKET;
}

SpeeduinoPacketResult makeSpeeduinoPacketResultFromReadState(SpeeduinoReadState &readState) {
  readState.phase = SPEEDUINO_READ_PHASE_DONE;

  return makePacketResult(speeduinoPacketStatusAfterRead(
    readState.bytesInPacket,
    readState.expectedPacketSize,
    readState.hasDiscardedBufferedBytes
  ));
}

SpeeduinoPacketResult makeSpeeduinoPacketResultFromCurrentStatus(SpeeduinoReadState &readState) {
  readState.phase = SPEEDUINO_READ_PHASE_DONE;
  return makePacketResult(readState.statusCode);
}

bool hasSpeeduinoPacketReadTimedOut(const SpeeduinoReadState &readState, unsigned long now) {
  return (now - readState.readStart) >= PACKET_READ_TIMEOUT;
}

void waitWithBackgroundService(unsigned long durationMs) {
  unsigned long waitStart = millis();
  while ((millis() - waitStart) < durationMs) {
    idleBackgroundService();
  }
}

void showStartupMessages() {
  lcdprint(0, 0, "Inspuiting wordt");
  lcdprint(0, 1, "    op druk gebracht");
  waitWithBackgroundService(1000);
  lcd.clear();

  lcdprint(0, 0, "Lomax is klaar");
  waitWithBackgroundService(500);
}

void renderSpeeduinoPacketResult(const SpeeduinoPacketResult &packetResult) {
  if (packetResult.statusCode == PACKET_STATUS_OK) {
    SpeeduinoSnapshot snapshot = decodeSpeeduinoSnapshot(packetResult.payload);
    renderSpeeduinoSnapshot(snapshot);
    renderGpsIndicator();
  }
  else {
    renderGpsIndicator();
    lcdprint(19, 3, packetResult.statusCode, "%1d");
  }
}

void completeSpeeduinoPoll(const SpeeduinoPacketResult &packetResult) {
  speeduinoPollState.packetResult = packetResult;
  speeduinoPollState.phase = SPEEDUINO_POLL_PHASE_IDLE;
  renderSpeeduinoPacketResult(speeduinoPollState.packetResult);
}

void startSpeeduinoPoll(unsigned long now) {
  lastSpeeduinoPollMillis = now;
  resetSpeeduinoReadState(speeduinoReadState, now);
  resetSpeeduinoPacketBuffer();
  startSpeeduinoSerialFlush(speeduinoPollState.flushState, now);
  speeduinoPollState.phase = SPEEDUINO_POLL_PHASE_FLUSHING_UNWANTED_BYTES;
}

void startSpeeduinoAdditionalBytesCheck(SpeeduinoPollFinishMode finishMode, unsigned long now) {
  speeduinoPollState.finishMode = finishMode;
  startSpeeduinoSerialFlush(speeduinoPollState.flushState, now);
  speeduinoPollState.phase = SPEEDUINO_POLL_PHASE_CHECKING_ADDITIONAL_BYTES;
}

void completeSpeeduinoAdditionalBytesCheck() {
  speeduinoReadState.hasDiscardedBufferedBytes = speeduinoPollState.flushState.hasDiscardedBufferedBytes;

  if (speeduinoPollState.finishMode == SPEEDUINO_POLL_FINISH_CURRENT_STATUS) {
    completeSpeeduinoPoll(makeSpeeduinoPacketResultFromCurrentStatus(speeduinoReadState));
    return;
  }

  completeSpeeduinoPoll(makeSpeeduinoPacketResultFromReadState(speeduinoReadState));
}

void serviceActiveSpeeduinoPoll() {
  while (speeduinoPollState.phase != SPEEDUINO_POLL_PHASE_IDLE) {
    unsigned long now = millis();

    if (speeduinoPollState.phase == SPEEDUINO_POLL_PHASE_FLUSHING_UNWANTED_BYTES) {
      if (!serviceSpeeduinoSerialFlush(speeduinoPollState.flushState, now)) {
        return;
      }

      startSpeeduinoPacketRequest(speeduinoReadState);
      speeduinoPollState.phase = SPEEDUINO_POLL_PHASE_READING_PACKET;
      continue;
    }

    if (speeduinoPollState.phase == SPEEDUINO_POLL_PHASE_READING_PACKET) {
      if (hasSpeeduinoPacketReadTimedOut(speeduinoReadState, now)) {
        startSpeeduinoAdditionalBytesCheck(SPEEDUINO_POLL_FINISH_PACKET_STATUS, now);
        continue;
      }

      if (speeduinoSerial.available() == 0) {
        return;
      }

      if (serviceSpeeduinoPacketReadStep(speeduinoReadState)) {
        if (speeduinoReadState.statusCode != PACKET_STATUS_OK) {
          startSpeeduinoAdditionalBytesCheck(SPEEDUINO_POLL_FINISH_CURRENT_STATUS, now);
        }
        else {
          startSpeeduinoAdditionalBytesCheck(SPEEDUINO_POLL_FINISH_PACKET_STATUS, now);
        }
        continue;
      }
    }

    if (speeduinoPollState.phase == SPEEDUINO_POLL_PHASE_CHECKING_ADDITIONAL_BYTES) {
      if (!serviceSpeeduinoSerialFlush(speeduinoPollState.flushState, now)) {
        return;
      }

      completeSpeeduinoAdditionalBytesCheck();
      return;
    }
  }
}

bool isSpeeduinoPollDue(unsigned long now) {
  return (now - lastSpeeduinoPollMillis) >= POLLING_INTERVAL;
}

void serviceSpeeduinoPoll() {
  if (speeduinoPollState.phase != SPEEDUINO_POLL_PHASE_IDLE) {
    serviceActiveSpeeduinoPoll();
    return;
  }

  unsigned long now = millis();
  if (!isSpeeduinoPollDue(now)) {
    return;
  }

  startSpeeduinoPoll(now);
  serviceActiveSpeeduinoPoll();
}

void setup() {
  lcd.begin(NUM_DISPLAY_COLS, NUM_DISPLAY_ROWS);

#if ENABLE_GPS
  setupGps();
#endif

  showStartupMessages();

  lcd.backlight();
  lcd.clear();
  speeduinoSerial.begin(115200);
  lastSpeeduinoPollMillis = millis() - POLLING_INTERVAL;
}

void loop() {
  serviceSpeeduinoPoll();
  idleBackgroundService();
}
