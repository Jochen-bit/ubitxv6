 // uBitX-WSPR-Configuration - begin
 char call[] = "YOURCALL"; // Call des WSPR-Baken-Inhabers
 char loc[] = "LOCATOR"; // 4-Stelliger Locator des Bakenstandorts - nur 6 stellig, wenn zuvor Call in spitzen Klammer: <YOURCALL> = Type 2 Message
 unsigned long _160m = 1838100l;
unsigned long _80m = 3570100l;  //33dBm
unsigned long _40m = 7040100l;  //30dBm
unsigned long _30m = 10140200l; //30dBm
unsigned long _20m = 14097100l; //33dBm
unsigned long _17m = 18106100l; //27dBm
unsigned long _15m = 21096100l; //27dBm
unsigned long _12m = 24926100l; //25dBm
unsigned long _10m = 28126100l; //25dBm
//unsigned long _6m = 50294500l; 
unsigned long bandToWSPR [] = {_10m, _12m, _15m, _17m}; // Band auf dem WSPR gearbeitet werden soll. Variable muss zuvor mit Frequenz in Herz definiert sein.
byte txPwr_dBm [] = {25, 25, 27, 27}; //Sendeleistung in dBm - Gleiche Anzahl an Werten wie bei bandToWSPR !!!
int txInterval = 4; //Sendeintervall in 2er Minutenschritte z.B. 2,4,8,12 max. 58 - Es wird der Modulus der Minute auf 0 geprüft.
int32_t calibrationWSPR = 181069; //Abweichung in Herz von VCO 875Mhz
 // uBitX-WSPR-Configuration - end
 /**
   We need to carefully pick assignment of pin for various purposes.
   There are two sets of completely programmable pins on the Raduino.
   First, on the top of the board, in line with the LCD connector is an 8-pin connector
   that is largely meant for analog inputs and front-panel control. It has a regulated 5v output,
   ground and six pins. Each of these six pins can be individually programmed
   either as an analog input, a digital input or a digital output.
   The pins are assigned as follows (left to right, display facing you):
        Pin 1 (Violet), A7, SPARE
        Pin 2 (Blue),   A6, KEYER (DATA)
        Pin 3 (Green), +5v
        Pin 4 (Yellow), Gnd
        Pin 5 (Orange), A3, PTT
        Pin 6 (Red),    A2, F BUTTON
        Pin 7 (Brown),  A1, ENC B
        Pin 8 (Black),  A0, ENC A
  Note: A5, A4 are wired to the Si5351 as I2C interface
 *       *
   Though, this can be assigned anyway, for this application of the Arduino, we will make the following
   assignment
   A2 will connect to the PTT line, which is the usually a part of the mic connector
   A3 is connected to a push button that can momentarily ground this line. This will be used for RIT/Bandswitching, etc.
   A6 is to implement a keyer, it is reserved and not yet implemented
   A7 is connected to a center pin of good quality 100K or 10K linear potentiometer with the two other ends connected to
   ground and +5v lines available on the connector. This implments the tuning mechanism
*/

//#define ENC_A (A0)
//#define ENC_B (A1)
#define FBUTTON (A2)
#define PTT   (A3)
//#define ANALOG_KEYER (A6)
//#define ANALOG_SPARE (A7)


/** pin assignments
  14  T_IRQ           2 std   changed
  13  T_DOUT              (parallel to SOD/MOSI, pin 9 of display)
  12  T_DIN               (parallel to SDI/MISO, pin 6 of display)
  11  T_CS            9   (we need to specify this)
  10  T_CLK               (parallel to SCK, pin 7 of display)
  9   SDO(MSIO) 12    12  (spi)
  8   LED       A0    8   (not needed, permanently on +3.3v) (resistor from 5v,
  7   SCK       13    13  (spi)
  6   SDI       11    11  (spi)
  5   D/C       A3    7   (changable)
  4   RESET     A4    9 (not needed, permanently +5v)
  3   CS        A5    10  (changable)
  2   GND       GND
  1   VCC       VCC

  The model is called tjctm24028-spi
  it uses an ILI9341 display controller and an  XPT2046 touch controller.
*/

#define TFT_DC  9
#define TFT_CS 10

//#define TIRQ_PIN  2
#define CS_PIN  8

// MOSI=11, MISO=12, SCK=13

//XPT2046_Touchscreen ts(CS_PIN);

//Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

/****************************************************/
#define ILI9341_BLACK       0x0000  ///<   0,   0,   0
#define ILI9341_NAVY        0x000F  ///<   0,   0, 123
#define ILI9341_DARKGREEN   0x03E0  ///<   0, 125,   0
#define ILI9341_DARKCYAN    0x03EF  ///<   0, 125, 123
#define ILI9341_MAROON      0x7800  ///< 123,   0,   0
#define ILI9341_PURPLE      0x780F  ///< 123,   0, 123
#define ILI9341_OLIVE       0x7BE0  ///< 123, 125,   0
#define ILI9341_LIGHTGREY   0xC618  ///< 198, 195, 198
#define ILI9341_DARKGREY    0x7BEF  ///< 123, 125, 123
#define ILI9341_BLUE        0x001F  ///<   0,   0, 255
#define ILI9341_GREEN       0x07E0  ///<   0, 255,   0
#define ILI9341_CYAN        0x07FF  ///<   0, 255, 255
#define ILI9341_RED         0xF800  ///< 255,   0,   0
#define ILI9341_MAGENTA     0xF81F  ///< 255,   0, 255
#define ILI9341_YELLOW      0xFFE0  ///< 255, 255,   0
#define ILI9341_WHITE       0xFFFF  ///< 255, 255, 255
#define ILI9341_ORANGE      0xFD20  ///< 255, 165,   0
#define ILI9341_GREENYELLOW 0xAFE5  ///< 173, 255,  41
#define ILI9341_PINK        0xFC18  ///< 255, 130, 198
#include <SPI.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include "ubitx.h"
// WSPP Encoder
#include <JTEncode.h>
#include <rs_common.h>

/////////////////////////
// For the Adafruit shield, these are the default.
#define TFT_CLK 13
#define TFT_MISO 12
#define TFT_MOSI 11
#define TFT_DC 9
#define TFT_CS 10
#define TFT_RST 8
// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
//Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);
// If using the breakout, change pins as desired
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);

//ubitX:
/**
    The second set of 16 pins on the Raduino's bottom connector are have the three clock outputs and the digital lines to control the rig.
    This assignment is as follows :
      Pin   1   2    3    4    5    6    7    8    9    10   11   12   13   14   15   16
           GND +5V CLK0  GND  GND  CLK1 GND  GND  CLK2  GND  D2   D3   D4   D5   D6   D7
    These too are flexible with what you may do with them, for the Raduino, we use them to :
    - TX_RX line : Switches between Transmit and Receive after sensing the PTT or the morse keyer
    - CW_KEY line : turns on the carrier for CW
*/

#define TX_RX (7)
#define CW_TONE (6)
#define TX_LPF_A (5)
#define TX_LPF_B (4)
#define TX_LPF_C (3)
#define CW_KEY (2)

/**
   These are the indices where these user changable settinngs are stored  in the EEPROM
*/
#define MASTER_CAL 0

/**
   The uBITX is an upconnversion transceiver. The first IF is at 45 MHz.
   The first IF frequency is not exactly at 45 Mhz but about 5 khz lower,
   this shift is due to the loading on the 45 Mhz crystal filter by the matching
   L-network used on it's either sides.
   The first oscillator works between 48 Mhz and 75 MHz. The signal is subtracted
   from the first oscillator to arriive at 45 Mhz IF. Thus, it is inverted : LSB becomes USB
   and USB becomes LSB.
   The second IF of 12 Mhz has a ladder crystal filter. If a second oscillator is used at
   57 Mhz, the signal is subtracted FROM the oscillator, inverting a second time, and arrives
   at the 12 Mhz ladder filter thus doouble inversion, keeps the sidebands as they originally were.
   If the second oscillator is at 33 Mhz, the oscilaltor is subtracated from the signal,
   thus keeping the signal's sidebands inverted. The USB will become LSB.
   We use this technique to switch sidebands. This is to avoid placing the lsbCarrier close to
   12 MHz where its fifth harmonic beats with the arduino's 16 Mhz oscillator's fourth harmonic
*/


#define INIT_USB_FREQ   (11059200l)
// limits the tuning and working range of the ubitx between 3 MHz and 30 MHz
#define LOWEST_FREQ   (100000l)
#define HIGHEST_FREQ (30000000l)

//we directly generate the CW by programmin the Si5351 to the cw tx frequency, hence, both are different modes
//these are the parameter passed to startTx
#define TX_SSB 0
#define TX_CW 1
/////////////////// wspr encoder start definitions
// Mode defines
#define WSPR_TONE_SPACING       146          // ~1.46 Hz
#define WSPR_DELAY              683          // Delay value for WSPR

// Hardware defines
#define BUTTON                  2
#define LED_PIN                 13

// Class instantiation
JTEncode jtencode;


// 
uint8_t dbm = 30;
uint8_t tx_buffer[255];
uint8_t symbol_count;
uint16_t tone_delay, tone_spacing;

boolean startUp = 0;// 0 when no Time is availiable on start up - 1: auto TX is running
int nextTxMin = 0; // cheduled minute of next TX
//unsigned long freq;
byte txBands = sizeof(bandToWSPR)/4;
byte currentBand = 0; // position in array is 0

/////////////////// wspr encoder end definitions


////////////////////////////

/// gps & time - start definitions ///
char Time[]  = " UTC: 00:00:00";

#include <TimeLib.h>          // include Arduino time library
#include <TinyGPS++.h>
TinyGPSPlus gps;

#include <SoftwareSerial.h>

SoftwareSerial gpsSerial( A1, A0 ); // RX, TX
unsigned long noOfGPSFails;
boolean errMsgSet = 0;
boolean gpsRead = 0;
int currentMin = 0;
String lastErrorMsg;
int gpsFailLimit = 100;
/// gps & time - end definitions ///

int btnDown(); //returns true if the encoder button is pressed

unsigned long frequency;
extern int32_t calibration;


/**
   Raduino needs to keep track of current state of the transceiver. These are a few variables that do it
*/
boolean txCAT = false;        //turned on if the transmitting due to a CAT command
char inTx = 0;                //it is set to 1 if in transmit mode (whatever the reason : cw, ptt or cat)
int splitOn = 0;             //working split, uses VFO B as the transmit frequency
char keyDown = 0;             //in cw mode, denotes the carrier is being transmitted
char isUSB = 0;               //upper sideband was selected, this is reset to the default for the
//frequency when it crosses the frequency border of 10 MHz
byte menuOn = 0;              //set to 1 when the menu is being displayed, if a menu item sets it to zero, the menu is exited
unsigned long cwTimeout = 0;  //milliseconds to go before the cw transmit line is released and the radio goes back to rx mode
unsigned long dbgCount = 0;   //not used now
unsigned char txFilter = 0;   //which of the four transmit filters are in use
boolean modeCalibrate = false;//this mode of menus shows extended menus to calibrate the oscillators and choose the proper
//beat frequency


/**
   Select the properly tx harmonic filters
   The four harmonic filters use only three relays
   the four LPFs cover 30-21 Mhz, 18 - 14 Mhz, 7-10 MHz and 3.5 to 5 Mhz
   Briefly, it works like this,
   - When KT1 is OFF, the 'off' position routes the PA output through the 30 MHz LPF
   - When KT1 is ON, it routes the PA output to KT2. Which is why you will see that
     the KT1 is on for the three other cases.
   - When the KT1 is ON and KT2 is off, the off position of KT2 routes the PA output
     to 18 MHz LPF (That also works for 14 Mhz)
   - When KT1 is On, KT2 is On, it routes the PA output to KT3
   - KT3, when switched on selects the 7-10 Mhz filter
   - KT3 when switched off selects the 3.5-5 Mhz filter
   See the circuit to understand this
*/

void setTXFilters(unsigned long freq) {

  if (freq > 21000000L) { // the default filter is with 35 MHz cut-off
    digitalWrite(TX_LPF_A, 0);
    digitalWrite(TX_LPF_B, 0);
    digitalWrite(TX_LPF_C, 0);
  }
  else if (freq >= 14000000L) { //thrown the KT1 relay on, the 30 MHz LPF is bypassed and the 14-18 MHz LPF is allowd to go through
    digitalWrite(TX_LPF_A, 1);
    digitalWrite(TX_LPF_B, 0);
    digitalWrite(TX_LPF_C, 0);
  }
  else if (freq > 7000000L) {
    digitalWrite(TX_LPF_A, 0);
    digitalWrite(TX_LPF_B, 1);
    digitalWrite(TX_LPF_C, 0);
  }
  else {
    digitalWrite(TX_LPF_A, 0);
    digitalWrite(TX_LPF_B, 0);
    digitalWrite(TX_LPF_C, 1);
  }
}

//unsigned long bandToWSPR [] = {_10m, _15m, _17m, _40m, _80m};
//byte txBands = sizeof(bandToWSPR);
//byte currentBand = 0; // position in array is 0
void startTxWSPR() {
  //if(inTx == 0){
 // unsigned long tx_freq = 0;

  // digitalWrite(TX_RX, 1);
  inTx = 1;
  frequency = bandToWSPR[currentBand];
  dbm = txPwr_dBm[currentBand];
  if(dbm > 50) { dbm = 40;}
  set_tx_buffer();
  setFrequencyMsg(0);
  //setTXFilters(frequency);
  //setFrequency(frequency); //set filters


  //  digitalWrite(TX_RX, 0);

  //turn off the second local oscillator and the bfo
  si5351bx_setfreq(0, 0);
  si5351bx_setfreq(1, 0);

  //si5351bx_setfreq(2, frequency);
  //delay(20);
  digitalWrite(TX_RX, 1); //turn on PA
  digitalWrite(CW_KEY, 1);
  uint8_t k;

  // Adjust the frequency and the output is turn on
  for (k = 0; k < symbol_count; k++)
  {
    si5351bx_setfreq(2, frequency + (( tx_buffer[k] * tone_spacing) / 100));
    delay(tone_delay);
  }
 // }
 if(currentBand == (txBands-1)) {
    currentBand = 0;
  } else {
    currentBand++;
  }
   setFrequencyMsg(1 );
}

void stopTxWSPR() {
  inTx = 0;
  digitalWrite(TX_RX, 0);//turn off PA
  si5351bx_setfreq(2, 0);//turn off the tx
  digitalWrite(CW_KEY, 0);

}



/**
   Basic User Interface Routines. These check the front panel for any activity
*/

//returns true if the button is pressed
int btnDown() {
  if (digitalRead(FBUTTON) == HIGH)
    return 0;
  else
    return 1;
}


//check if the encoder button was pressed
void checkButton() {
  int i, t1, t2, knob, new_knob;

  //only if the button is pressed
  if (!btnDown())
    return;
  delay(50);
  if (!btnDown()) //debounce
    return;

    inTx = 1;
    setTxMessage();
 
    startTxWSPR();
    
    stopTxWSPR();
    setTxMessage();
  

  //doCommands();
  //wait for the button to go up again
  while (btnDown())
    delay(10);
  //active_delay(50);//debounce
}



/**
   The settings are read from EEPROM. The first time around, the values may not be
   present or out of range, in this case, some intelligent defaults are copied into the
   variables.
*/
void initSettings() {
  byte x;
  //read the settings from the eeprom and restore them
  //if the readings are off, then set defaults


  // the screen calibration parameters : int slope_x=104, slope_y=137, offset_x=28, offset_y=29;



}

void initPorts() {

  analogReference(DEFAULT);

  //??

  pinMode(FBUTTON, INPUT_PULLUP);

  //configure the function button to use the external pull-up
  //  pinMode(FBUTTON, INPUT);
  //  digitalWrite(FBUTTON, HIGH);

  pinMode(PTT, INPUT_PULLUP);
  //  pinMode(ANALOG_KEYER, INPUT_PULLUP);

  pinMode(CW_TONE, OUTPUT);
  digitalWrite(CW_TONE, 0);

  pinMode(TX_RX, OUTPUT);
  digitalWrite(TX_RX, 0);

  pinMode(TX_LPF_A, OUTPUT);
  pinMode(TX_LPF_B, OUTPUT);
  pinMode(TX_LPF_C, OUTPUT);
  digitalWrite(TX_LPF_A, 0);
  digitalWrite(TX_LPF_B, 0);
  digitalWrite(TX_LPF_C, 0);

  pinMode(CW_KEY, OUTPUT);
  digitalWrite(CW_KEY, 0);
}

void set_tx_buffer()
{
  // Clear out the transmit buffer
  memset(tx_buffer, 0, 255);
  jtencode.wspr_encode(call, loc, dbm, tx_buffer);
}

void updateTimeOnScr(int i_hour, int i_minute, int i_second) {

  // add the offset to get local time
  //adjustTime(time_offset);
  tft.setCursor(0, 50);
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(3);
  tft.println(Time);



  // update time array
  Time[12] = second() / 10 + '0';
  Time[13] = second() % 10 + '0';
  Time[9]  = minute() / 10 + '0';
  Time[10] = minute() % 10 + '0';
  Time[6]  = hour()   / 10 + '0';
  Time[7]  = hour()   % 10 + '0';
  tft.setCursor(0, 50);
  tft.setTextColor(ILI9341_BLUE);
  tft.setTextSize(3);
  tft.println(Time);

}

int lastSec = 0;

void readFromGPS() {

  if (gpsSerial.available() > 0) {

    if (gps.encode(gpsSerial.read())) {
      if (gps.time.isValid()) {
        noOfGPSFails = 0;
        setTime(gps.time.hour(), gps.time.minute() , gps.time.second(), 1, 1, 2025);
        delay(22);
        if (lastSec != gps.time.second()) {
          lastSec = gps.time.second();
          updateTimeOnScr(gps.time.hour(), gps.time.minute() , gps.time.second() );
        }

        if (errMsgSet) {
          clearErrorMsg();
          errMsgSet = 0;
        }
      }
      else {
        if (noOfGPSFails < gpsFailLimit) {
          noOfGPSFails++;
          delay(20);
        }
      }
    }
  } else {
    if (noOfGPSFails < gpsFailLimit) {
      noOfGPSFails++;
      delay(20);
    }
  }
}



//int txInterval = 2; //Sendeintervall in 2er Minutenschritte z.B. 2,4,8,12
//boolean startUp = 0;// 0 when no Time is availiable on start up - 1: auto TX is running
//int nextTxMin = 0; // cheduled minute of next TX
void checkTxTime() {
  if(currentMin == minute() || currentMin == minute()+1) {
    return;
  }
  currentMin = minute();
  if (currentMin % txInterval == 0 && second() < 5) {
    inTx = 1;
    setTxMessage();
 
    startTxWSPR();
    
    stopTxWSPR();
    setTxMessage();
    
  }
}


void printToScreenOnStart() {
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(1);
  tft.println("Powered by Software from DG7JH");
  tft.setCursor(0, 20);
  tft.setTextColor(ILI9341_YELLOW); tft.setTextSize(2);
  tft.println("Multiband-WSPR-Transmitter");
  tft.setCursor(0, 50);
  tft.setTextColor(ILI9341_ORANGE);
  tft.setTextSize(3);
  tft.println(Time);
  tft.setCursor(0, 90);
  tft.setTextSize(2);
  tft.print("WSPR Message: ");
  tft.print(call);
  tft.print(" ");
  tft.print(loc);
  setTxMessage();
}

/*
   Before call this method set inTx
*/
void setTxMessage() {
  tft.setCursor(0, 120);
  tft.setTextSize(3);
  if (inTx == 0) {
    tft.setCursor(0, 120);
    tft.setTextColor(ILI9341_BLACK);
    tft.println("ON AIR!");
    tft.setCursor(0, 120);
    tft.setTextColor(ILI9341_DARKGREEN);
    tft.println("Standby - no TX");
  } else {
    tft.setCursor(0, 120);
    tft.setTextColor(ILI9341_BLACK);
    tft.println("Standby - no TX");
    tft.setCursor(0, 120);
    tft.setTextColor(ILI9341_RED);
    tft.println("ON AIR!");
  }
}

// del == 1: delete last msg
void setFrequencyMsg(boolean del) {
  tft.setCursor(0, 160);
  tft.setTextSize(2);
  
  if (del == 1) {
    tft.setTextColor(ILI9341_BLACK);
    tft.print("Tx Freqency: ");
    tft.print(frequency/1000);
    tft.print(" kHz");
  } else {
     tft.setTextColor(ILI9341_ORANGE);
     tft.print("Tx Freqency: ");
    tft.print(frequency/1000);
    tft.print(" kHz");
  }
}

void printErrorMsg(String msg) {
  lastErrorMsg = msg;
  tft.setCursor(0, 190);
  tft.setTextColor(ILI9341_RED);
  tft.setTextSize(2);
  tft.println(msg);
}

void clearErrorMsg() {
  tft.setCursor(0, 190);
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(2);
  tft.println(lastErrorMsg);
}



void loop(void) {
  while(gpsRead == 0 && gpsSerial.available() > 0) {
    readFromGPS();
    gpsRead = 1;
  }
  if (startUp == 0) {
    if (second() > 15) {
      startUp = 1; //assuming Time is runing now on this device
    }
  } else { // startUp = 1 ; startUp is over
    if(inTx == 0 && gpsRead == 1) {
      checkTxTime();
    }
  }
  gpsRead = 0;
  checkButton();
  //delay(800);
  
  //delay(20);
  if (noOfGPSFails > gpsFailLimit - 2 && !errMsgSet) {
    String erMsg = "GPS-Signal lost - next TX stopped";
    printErrorMsg(erMsg);
    errMsgSet = 1;
  }

}

void setup() {
  gpsSerial.begin(9600);

  tft.begin();
  // initSettings();
  EEPROM.get(MASTER_CAL, calibration);
  // Serial.println("Calibration value:");
  calibration = calibrationWSPR;
  //Serial.println(calibration);
  initPorts();
  initOscillators();
  // si5351_set_calibration(calibration);
  set_tx_buffer();
  symbol_count = WSPR_SYMBOL_COUNT; // From the library defines
  tone_spacing = WSPR_TONE_SPACING;
  tone_delay = WSPR_DELAY;
  setTime(0, 0, 0, 1, 1, 2025);
  // read diagnostics (optional but can help debug problems)
  uint8_t x = tft.readcommand8(ILI9341_RDMODE);
  tft.setRotation(1);
  printToScreenOnStart();

}


