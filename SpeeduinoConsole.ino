#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
SoftwareSerial mySerial2(10, 11); // RX, TX

// Module for reading Speeduino's serial3 port and displaying it on a 20*4 character LCD 
// Oct 2020, Lex Sewuster
//
// See https://speeduino.com/wiki/index.php/Secondary_Serial_IO_interface
//
// For displaying I used
// https://www.aliexpress.com/item/32920769382.html  (LCD Board 2004 20*4 LCD 20X4 3.3V/5V Blue/Yellow and Gree Screen LCD2004 Display LCD Module LCD 2004 for arduino)
// https://www.aliexpress.com/item/2035880451.html   (IIC/I2C Interface LCD1602 Adapter Plate Board 5V LCD Adapter Converter Module For LCD1602 2004 LCD)
// 
// An Arduino Uno is too slow to keep up with a linespeed of 115200 bps. I used a compact Mega2560
// https://www.aliexpress.com/item/4000235952850.html (Pro mini MEGA2560 <-- select the right board when ordering!)
// (another reason for using an Mega2560 is that I might want to expand it with knock detection and NEO gps reading.  
//
// In this version i fiddled with moving bars of half character height for graphical displaying of the rpm and air-fuel-ratio. 
// This is still a work-in-progress. If one needs to focus on the serial IO and the displaying only, he/she could remove anything needed for the procedure 'lcdBar()'

// Position numbers in Speeduino's Real time data block 
const byte SQUIRT = 1;
const byte ENGINE = 2;
const byte DWELL = 3;
const byte MAP_LB = 4;
const byte MAP_HB = 5;
const byte IAT_PLUS_OFFSET = 6;
const byte BATTERY10 = 9;
const byte OXIGEN = 10;
const byte COOLANT_PLUS_OFFSET = 7;
const byte BAT_CORRECTION = 8;
const byte EGO_CORRECTION = 11;
const byte IAT_CORRECTION = 12;
const byte WUE_CORRECTION = 13;
const byte RPM_LB = 14;
const byte RPM_HB = 15;
const byte TAE_AMOUNT = 16;
const byte CORRECTIONS = 17;
const byte VE = 18;

const byte AFR_TARGET = 19;
const byte PW1_LB = 20;
const byte PW1_HB = 21;
const byte TPS_DOT = 22;
const byte ADVANCE_ANGLE = 23;
const byte TPS = 24;
const byte LOOPS_PER_SECOND_LB = 25;
const byte LOOPS_PER_SECOND_HB = 26;
const byte FREE_RAM_LB = 27;
const byte FREE_RAM_HB = 28;
const byte BOOST_TARGET = 29;
const byte BOOST_DUTY = 30;
const byte SPARK = 31;
const byte RPM_DOT_LB = 32;
const byte RPM_DOT_HB = 33;
const byte ETHANOL_PCT = 34;
const byte FLEX_CORRECTION = 35;
const byte FLEX_IGN_CORRECTION = 36;
const byte IDLE_LOAD = 37;
const byte TEST_OUTPUTS = 38;
const byte OXIGEN2 = 39;
const byte BARO = 40;

const char TEMPERATURE_FORMAT[] = {'%', '3', 'd', ' ',  char(223), 'C', ' ', char(0)};

const int  WAITING_INTERVAL =  100;  // in ms
const int  POLLING_INTERVAL = 1000;  // in ms


// Character definition (only 8 possible with Hitachi HD44780) for the moving bar emulation. 
// special characters are defined by their quadrans Upper-Left, Upper-Right, Lower-Left and Lower-Right.
// A quadrant is always filled from left to right. 

// 1 quadrant in upper half, 0 quadrant in lower half
byte CHAR10[8] = {
  B11100,
  B11100,
  B11100,
  B11100,
  B00000,
  B00000,
  B00000,
  B00000
};
// 0 quadrant in upper half, 1 quadrant in lower half
byte CHAR01[8] = {
  B00000,
  B00000,
  B00000,
  B00000,
  B11100,
  B11100,
  B11100,
  B11100
};
// 1 quadrant in upper half, 1 quadrant in lower half
byte CHAR11[8] = {
  B11100,
  B11100,
  B11100,
  B11100,
  B11100,
  B11100,
  B11100,
  B11100
};
// 2 quadrants in upper half, 0 quadrant in lower half
byte CHAR20[8] = {
  B11111,
  B11111,
  B11111,
  B11111,
  B00000,
  B00000,
  B00000,
  B00000
};
// 0 quadrant in upper half, 2 quadrants in lower half
byte CHAR02[8] = {
  B00000,
  B00000,
  B00000,
  B00000,
  B11111,
  B11111,
  B11111,
  B11111
};
// 2 quadrants in upper half, 1 quadrant in lower half
byte CHAR21[8] = {
  B11111,
  B11111,
  B11111,
  B11111,
  B11100,
  B11100,
  B11100,
  B11100
};
// 1 quadrant in upper half, 2 quadrants in lower half
byte CHAR12[8] = {
  B11100,
  B11100,
  B11100,
  B11100,
  B11111,
  B11111,
  B11111,
  B11111
};
// 2 quadrants in upper half, 2 quadrant in lower half
byte CHAR22[8] = {
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111
};

const int NUM_DISPLAY_COLS = 20;
const int NUM_DISPLAY_ROWS = 4;

const byte SPACE = 32;
const byte C10 = 0;
const byte C01 = 1;
const byte C11 = 2;
const byte C20 = 3;
const byte C02 = 4;
const byte C21 = 5;
const byte C12 = 6;
const byte C22 = 7;

byte PATTERN[3][3] = {
  {SPACE, C01, C02},
  {C10, C11, C12},
  {C20, C21, C22}
};

byte reply ;
unsigned long start;
int numberOfBytesRead;
const byte messageSize = 100;
byte message[messageSize];
int timeConsumedByReadingAndDisplaying;
char numBuf[21];

boolean engineStarted = false;

void setup() {

  lcd.begin( NUM_DISPLAY_COLS, NUM_DISPLAY_ROWS);

  lcd.createChar(C10, CHAR10);
  lcd.createChar(C01, CHAR01);
  lcd.createChar(C11, CHAR11);
  lcd.createChar(C20, CHAR20);
  lcd.createChar(C02, CHAR02);
  lcd.createChar(C21, CHAR21);
  lcd.createChar(C12, CHAR12);
  lcd.createChar(C22, CHAR22);

  lcd.setCursor(0, 0);
  lcd.print("Lomax is klaar");
  lcd.backlight();

  // set the data rate for the SoftwareSerial port
  mySerial2.begin(115200);

  // Serial.begin(9200);

}

// convert Fahrenheid to Celsius
int convertFtoC( int f) {
  return (int)( ((f - 32) * 5) / 9);
}

void lcdprint(byte col, byte row, int num, char fmt[]) {
  lcd.setCursor(col, row);
  sprintf (numBuf, fmt, num);
  lcd.print(numBuf);
}


int activeCells (float value, float minValue, float maxValue, int num_cells) {
  float interval = (maxValue - minValue) / (num_cells + 1);
  int nrActiveCells = floor( (value - minValue) / interval );

  if (nrActiveCells < 0)
    nrActiveCells = 0;
  if (nrActiveCells > num_cells)
    nrActiveCells = num_cells;

  return nrActiveCells;
}

void lcdBar(float topValue, float topMinValue, float topMaxValue, float bottomValue, float bottomMinValue, float bottomMaxValue, int num_cols) {
  int num_cells = 2 * num_cols;
  int nrActiveTopCells = activeCells(topValue, topMinValue, topMaxValue, num_cells);
  int nrActiveBottomCells = activeCells(bottomValue, bottomMinValue, bottomMaxValue, num_cells);
  lcdRawBar(nrActiveTopCells, nrActiveBottomCells, num_cells);
}


int numActiveCells(int pointer, int value) {
  if ( pointer < value ) {
    return 2;
  }
  if ( pointer == value ) {
    return  1;
  }
  return  0;
}

void lcdRawBar(int p, int q, int num_cells) {
  int numTop, numBottom;
  for (int i = 1; i < num_cells; i = i + 2) {
    numTop = numActiveCells(i,p);
    numBottom = numActiveCells(i,q);
    lcd.write( byte ( PATTERN[numTop][numBottom] ) );
  }  //for
}


void loop() {

  // Output an "A" to Speeduino 
  numberOfBytesRead = 0;
  mySerial2.print("A");
  
  // Do not wait longer than WAITING_INTERVAL milliseconds for the respons to complete. 
  start = millis();
  while ( (millis() - start) < WAITING_INTERVAL ) {
    while (mySerial2.available() != 0) {
      reply = mySerial2.read();
      if (numberOfBytesRead > 0 ) { // the skip first character since it's not part of the currentStatus data block
        message[numberOfBytesRead - 1] = reply;
      }
      if (numberOfBytesRead < messageSize - 1) { // prevent buffer overrun when an unexpected large number of characters arrive
        numberOfBytesRead++;
      }
    }
  }
  // TODO change the above character reading to interrupt driven reading of the full response to keep the processor free for other tasks. 


  if (engineStarted) {
    // lcd.clear();

  // TODO The Coolant temperature is not the same as the temperature shown in TunerStudio. Probably a bug on my side.

    lcdprint( 1, 0, message[RPM_HB] * 255 + message[RPM_LB], "%4d rpm   ");
    lcdprint(12, 0, convertFtoC(message[COOLANT_PLUS_OFFSET]),  TEMPERATURE_FORMAT  );  // convert Fahrenheid to Celsius

    lcdprint( 1, 1, message[MAP_HB] * 255 + message[MAP_LB], "%4d kPa ");
    lcdprint(12, 1, message[ADVANCE_ANGLE], "%3d deg ");

    lcdprint(11, 2, message[OXIGEN] / 10, "%2d.");
    lcdprint(14, 2, message[OXIGEN] % 10, "%01d afr");

    lcd.setCursor(0, 3);
    lcdBar(float(message[RPM_HB] * 255 + message[RPM_LB]), 0.0, 7000.0, float(message[OXIGEN]) / 10.0, 10.0, 20.0, NUM_DISPLAY_COLS);   // Graphical display in moving bar of rpm en afr 
  }
  else {
    if (message[RPM_HB] > 0 ) {
      lcd.clear();
      engineStarted = true;
    }
  } 


  // correct polling time for time needed for by reading and displaying
  timeConsumedByReadingAndDisplaying = millis() - start;
  if (  timeConsumedByReadingAndDisplaying < POLLING_INTERVAL ) {            // prevent delay() with negative parameter (gives extreme long delays)
    delay(  POLLING_INTERVAL - timeConsumedByReadingAndDisplaying );
  }



}
