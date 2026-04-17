#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <SpeeduinoProtocol.h>
#include <string.h>

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
HardwareSerial &speeduinoSerial = Serial1;  // RX1 = 19, TX1 = 18

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
constexpr unsigned long BACKGROUND_SERVICE_IDLE_DELAY = 1UL;

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

byte packet[MAX_PACKET_SIZE];  // More than enough for the maximum payload plus header.
byte payloadLength = 0;

SpeeduinoPacketResult makePacketResult(byte statusCode) {
  SpeeduinoPacketResult result = {statusCode, payloadLength, packet + PAYLOAD_OFFSET};
  return result;
}

void resetPacketBuffer() {
  memset(packet, 0, sizeof(packet));
  payloadLength = 0;
}

bool hasValidPacketHeader() {
  return (packet[0] == 'n') && (packet[1] == '2');
}

bool hasMinimumDisplayPayloadLength(byte packetPayloadLength) {
  return packetPayloadLength >= MIN_DISPLAY_PAYLOAD_LENGTH;
}

byte validatePacketHeaderAndSetExpectedSize(int &expectedPacketSize) {
  if (!hasValidPacketHeader()) {
    return PACKET_STATUS_HEADER_INVALID;
  }

  payloadLength = packet[2];
  if (!hasMinimumDisplayPayloadLength(payloadLength)) {
    return PACKET_STATUS_PAYLOAD_TOO_SHORT;
  }

  expectedPacketSize = HEADER_SIZE + payloadLength;
  return PACKET_STATUS_OK;
}

void storeIncomingPacketByte(byte incomingByte, int &bytesInPacket) {
  if (bytesInPacket < MAX_PACKET_SIZE) {
    packet[bytesInPacket] = incomingByte;
  }
  bytesInPacket++;
}

bool hasCompleteExpectedPacket(int bytesInPacket, int expectedPacketSize) {
  return (expectedPacketSize >= HEADER_SIZE) && (bytesInPacket == expectedPacketSize);
}

byte packetStatusAfterRead(int bytesInPacket, int expectedPacketSize, bool hasDiscardedBufferedBytes) {
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

void lcdprint(byte col, byte row, int num, const char *fmt) {
  static char numBuf[21];
  lcd.setCursor(col, row);
  snprintf(numBuf, sizeof(numBuf), fmt, num);
  lcd.print(numBuf);
}

void lcdprint(byte col, byte row, const char *text) {
  lcd.setCursor(col, row);
  lcd.print(text);
}

void lcdprintTenths(byte col, byte row, int value10, const char *suffix, byte wholeWidth = 2) {
  static char numBuf[21];
  lcd.setCursor(col, row);
  if (wholeWidth <= 1) {
    snprintf(numBuf, sizeof(numBuf), "%d.%1d%s", value10 / 10, abs(value10) % 10, suffix);
  }
  else {
    snprintf(numBuf, sizeof(numBuf), "%2d.%1d%s", value10 / 10, abs(value10) % 10, suffix);
  }
  lcd.print(numBuf);
}

const char *engineStatus(byte status) {
  static const char statusChars[] = {'R', 'C', 'A', 'W', 'a', 'd', '<', '>'};
  static char buf[7] = {' ', ' ', ' ', ' ', ' ', ' ', '\0'};
  int bufPos = 5;

  for (int bitNr = BIT_ENGINE_RUN; bitNr <= BIT_ENGINE_MAPDCC; bitNr++) {
    if (status & (1 << bitNr)) {
      buf[bufPos] = statusChars[bitNr];
      bufPos--;
    }
  }

  for (int i = bufPos; i >= 0; i--) {
    buf[i] = ' ';
  }

  return buf;
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

  lcdprintTenths(0, 3, snapshot.fuelPressure10, "bar            ", 1);

  lcdprint(19, 3, " ");
}

void serviceBackgroundTasks() {
}

void idleBackgroundService() {
  serviceBackgroundTasks();
  delay(BACKGROUND_SERVICE_IDLE_DELAY);
}

bool readExtraCharsIfAny() {
  unsigned long readStart = millis();
  bool discardedBytes = false;

  while ((millis() - readStart) < UNEXPECTED_BYTES_WAITING_INTERVAL) {
    if (speeduinoSerial.available() == 0) {
      idleBackgroundService();
      continue;
    }
    discardedBytes = true;
    speeduinoSerial.read();
  }

  return discardedBytes;
}

SpeeduinoPacketResult requestAndReadPacket() {
  int bytesInPacket = 0;
  int expectedPacketSize = -1;
  byte incomingByte = 0;
  bool hasDiscardedBufferedBytes = false;
  unsigned long readStart = millis();

  resetPacketBuffer();

  // Discard stale bytes from a previous read before requesting fresh data.
  readExtraCharsIfAny();
  speeduinoSerial.print("n");

  // Read until the expected packet length is complete or the timeout expires.
  while ((millis() - readStart) < PACKET_READ_TIMEOUT) {
    if (speeduinoSerial.available() == 0) {
      idleBackgroundService();
      continue;
    }

    incomingByte = speeduinoSerial.read();

    storeIncomingPacketByte(incomingByte, bytesInPacket);

    if (bytesInPacket == HEADER_SIZE) {
      byte headerStatusCode = validatePacketHeaderAndSetExpectedSize(expectedPacketSize);
      if (headerStatusCode != PACKET_STATUS_OK) {
        readExtraCharsIfAny();
        return makePacketResult(headerStatusCode);
      }
    }

    if (hasCompleteExpectedPacket(bytesInPacket, expectedPacketSize)) {
      break;
    }
  }

  // Wait a little longer for unexpected trailing bytes and discard them if seen.
  hasDiscardedBufferedBytes = readExtraCharsIfAny();

  return makePacketResult(packetStatusAfterRead(bytesInPacket, expectedPacketSize, hasDiscardedBufferedBytes));
}

void waitUntilNextPoll(unsigned long cycleStart) {
  while( millis() - cycleStart < POLLING_INTERVAL) {
    idleBackgroundService();
  }
}

void setup() {
  lcd.begin(NUM_DISPLAY_COLS, NUM_DISPLAY_ROWS);

  lcd.setCursor(0, 0);
  lcd.print("Inspuiting wordt");
  lcd.setCursor(0, 1);
  lcd.print("    op druk gebracht");
  delay(3000);
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("Lomax is klaar");
  delay(500);

  lcd.backlight();
  lcd.clear();
  speeduinoSerial.begin(115200);
}

void loop() {
  unsigned long cycleStart = millis();

  SpeeduinoPacketResult packetResult = requestAndReadPacket();

  if (packetResult.statusCode == PACKET_STATUS_OK) {
    SpeeduinoSnapshot snapshot = decodeSpeeduinoSnapshot(packetResult.payload);
    renderSpeeduinoSnapshot(snapshot);
  }
  else {
    lcdprint(19, 3, packetResult.statusCode, "%1d");
  }

  waitUntilNextPoll(cycleStart);
  
}
