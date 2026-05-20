// -*- mode: C++ -*-
//
// UFP-II
//
// Software designed for *Adafruit Feather M0 RFM69HCM* module
//
// 
// Release history:
//
// V1.47 20181211 handleRaceEvents (former handleRace) optimized
// V1.46 20181209 GPS support removed
// V1.45 20181103 bug fix in round trip end beep
// V1.44 20181027 bug fixes in GPS interface, base to rover transmission speed improved
// V1.43 20181027 bug fixes in GPS interface, parser speed improved
// V1.42 20181026 bug fixes in GPS interface, parser
// V1.41 20181025 bug fixes in contest engine
// V1.4 20181024  support for GPS RTK Base and Rover for GPS module SkyTra S1216F8-RTK (parser for NMEA PSTI-032 protocol)
// V1.3 20181023  faster update of OLED implemented
// V1.2 20181020  support for GPS RTK Base and Rover (data link) & GPS based distance calculation of base line; accuracy +/-1.25cm
//                Required GPS module: ublox NEO-M8P-2
// V1.1 20181002  support for supply via FCD in TX modes
// V1.0 20180925  Initial design, incl. control receiver interface, OLED display interface, and Start/Reset key input in RX modes (no FRAM, no rotary key).
//                Beep frequencies and durations fixed.
// 
// 0-3 jumpers on P7 define the operational mode as follows:
//
// 5-6  AdvancedMode:          closed: not used       open: not used
// 3-4  Engine off beep:       closed: active,        open: off
// 1-2  OperationMode:         closed: TX,            open: RX
//
// All messages sent and received by the RH_RF69 driver conform to this packet format:
//
// - 4 octets PREAMBLE
// - 2 octets SYNC 0x2a, 0x54 (configurable, so you can use this as a network filter to destinguish from other UFP-II)
// - 1 octet RH_RF69 payload length
// - 4 octets HEADER: (TO, FROM, ID, FLAGS)
// - 0 to 60 octets DATA 
// - 2 octets CRC computed with CRC16(IBM), computed on HEADER and DATA
//
//  With currently 2 byte payload (message number and crossing status), 15 bytes or 120 bits will be transmitted in a message.
//  Selecting GFSK with 55kBit/s, the on air time is about 2ms, the time from crossing event at the transmitter to status
//  change at the receiver will sum up to about 6ms. The message is resent once after a brief delay of 2ms for increased reliability.
//  On the receiver side, this resent message is ignored when the original message was received successfully.
//
//  Allowed ISM frequencies in Germany according to vfg5/2018:
//  "Allgemeinzuteilung von Frequenzen zur Nutzung durch Funkanwendungen mit geringer Reichweite für nicht näher spezifizierte Anwendungen;
//  Non-specific Short Range Devices (SRD)"
//
//    (1) 433,050 MHz - 434,790 MHz @ 10mW (0dBm)
//    (2) 865,0 MHz - 868,0 MHz @ 25mW (14dBm), on air duty cycle max. 1% (radio band typically used for radio mices and 2W RFID)
//    (3) 868,0 MHz - 868,6 MHz @ 25mW (14dBm), on air duty cycle max. 1%
//    (4) 868,7 MHz - 869,2 MHz @ 25mW (14dBm), on air duty cycle max. 0.1%
//    (5) 869,40 MHz - 869,65 MHz @ 500mW (27dBm), on air duty cycle max. 10%
//    (6) 869,7 MHz – 870,0 MHz @ 25mW (14dBm), on air duty cycle max. 1%
//
//  The duty cycle is defined as "at any time within an hour", i.e. a 0.1% duty cycle allows for 3.6s on air time in the hour. This gives 
//  up to 1800 transmissions or at least 15 flight blocks the hour (up to 60 legs, double clicked: 120ms); thats nearly impossible; so no
//  problem to use any of the frequency bands above, even the most sensitive (4). This gives up 22 possible channels with a bandwidth of 200kHz
//
//  Proposed frequency for 433MHz band: 434,685 MHz
//
//  Proposed frequency for 868MHz band: 869.525 MHz
//
//  
//  Engine shutoff command beep:
//    Requirements:   reference receiver connected to RX port (P1-1, pin 1,2,3)
//    Purpose:        F5B/F training hint to shut off the engine during climb at A Base
//    Implementation: when the engine is on for more then 1.35s (climb for 4 legs), a 200ms beep at 3000Hz is emitted
//                    when the engine is on for more then 2.25s (climb for 6 legs or 4 legs F5F), a 200ms beep at 3500Hz is emitted
//    Default engine ON detection: RX pulse duration > 1350us
//


/********* Includes *********/
#include <Wire.h>
#include <SPI.h>
#include <RH_RF69.h>
#include <U8g2lib.h>
#include <avr/dtostrf.h>
//#include <Adafruit_FRAM_I2C.h>
#include <DimmerZero.h>
// Attention: DimmerZero library had been modified

// Defines

// OLED interface (depending on controller chip)
#define SH1106_F
//#define SH1306_F

// Debug
//#define debug
//#define debug1

/************ Radio Setup ***************/
// set frequency for receiver and transmitter
// 433MHz band: channel 1..8  (transceiver with red dot)
// 868MHz band: channel 9..30 (transceiver with green dot)
// 915MHz band:

#define TXPWRHH 20
#define TXPWRH 17
#define TXPWRL 14

// I/Os for the radio modue RFM69HCM on M0
#define RFM69_CS      8
#define RFM69_INT     3
#define RFM69_RST     4

#define RH_RF69_MESSAGE_LEN   2

/****** engine off beep duration ********/
#define CLIMB4LEGTIME 1350
#define CLIMB6LEGTIME 2250

/*** duration of message bits active ****/
#define SIGNALTIME  400

/*** duration of beep tones *************/
#define BEEPTIME    400
#define MBEEPTIME   250
#define EBEEPTIME   200
#define RTEEPTIME   1500

/***** Timer setup **********************/
// ARM core operates at 48MHz
#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 1024
#define TICK_COUNTS_1S 1000
#define TICK_COUNTS_250MS 250

/***** I/O definitions ******************/
#define LED           13
#define CTRLRECEIVER  5
#define CH1IN         6
#define CH2IN         10
#define CH3IN         11
#define RELAIS        9
#define CH1OUT        A3
#define CH2OUT        A4
#define CH3OUT        A5
#define BEEPER        12
#define KEYL          A0
#define KEYR          A1
#define KEYT          A2
#define SCL           21
#define SDA           20

#define BUFLEN  3000

// start sequence defines
#define IDLE  0
#define START 1
#define RUN   2

// engine defines
#define ON    2
#define OFF   1
#define INV   0

// leg update define
#define A_TURN  0
#define B_TURN  1


// Instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Instance of the OLED driver
// default address 0x78
#ifdef SH1106_P
  U8G2_SH1106_128X64_NONAME_2_HW_I2C u8g2(U8G2_R0); // Page buffer mode
#endif
#ifdef SH1106_F
  U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0); // Full screen buffer mode
#endif
#ifdef SH1306_P
  U8G2_SSD1306_128X64_NONAME_2_HW_I2C u8g2(U8G2_R0);
#endif
#ifdef SH1306_F
  U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);
#endif


// create dimmer instance for beeper control
DimmerZero beepCh3(BEEPER);

// create FRAM instance
//Adafruit_FRAM_I2C fram = Adafruit_FRAM_I2C();


// Version
const float SWversion = 1.47;

// display buffer
char s[80];

// variables
int16_t packetnum = 0;  // packet counter, we increment per xmission
uint16_t per;
int cnt = 0;
uint16_t opMode = 0;
int txPwr = 0;
int txPwrMw = 0;
uint8_t haveFRAM = false;

// timer variables
volatile int        t_1000ms_count = TICK_COUNTS_1S;  // Counter for 1s
volatile int        t_250ms_count = TICK_COUNTS_250MS;  // Counter for 250ms
volatile uint16_t   pulseDetectTimer = 0;
volatile long       t_beeperCnt = 0;
volatile uint8_t    t_1000msFlag = false;
volatile uint8_t    t_250msFlag = false;
volatile uint8_t    t_1msFlag = false;
volatile uint32_t   t_EngineStartTime = 0;
volatile uint16_t   t_engineOffTimer1 = 0;
volatile uint16_t   t_engineOffTimer2 = 0;
volatile uint16_t   pwmFrequency = 0;


typedef struct {
  union {
    struct {
      byte ch1in : 1;
      byte ch2in : 1;
      byte ch3in : 1;
      byte b4 : 1;
      byte b5 : 1;
      byte b6 : 1;
      byte b7 : 1;
      byte b8 : 1;
    } chInbits;
    byte chInbyte;
  };
} crossing;

volatile crossing inStates;
volatile crossing recStates;
volatile uint8_t  legCount = 0;
volatile uint8_t  onCount = 0;
uint8_t           lastLegUpdate = 0;
volatile uint32_t pulseDuration = 0;
volatile uint32_t pulseStart = 0;
volatile uint32_t roundtripCounter = 0;
volatile uint32_t roundtripEndBeep = 0;
uint8_t engineReportedOn = false;

uint32_t  rangeTestTimeCnt = 0;
uint8_t   motorOffBeep = false;

uint16_t ledTime = 0;
uint16_t beepTime = 0;
uint16_t relaisTime = 0;
uint16_t ch1Time = 0;
uint16_t ch2Time = 0;
uint16_t ch3Time = 0;
volatile uint8_t engineState = INV;
uint8_t ledMode = false;
uint8_t Ch3inReleased = true;
uint8_t rangeTest = false;
uint8_t rRangeTest = false;
uint32_t rRangeTestTime = 0;
volatile uint8_t startSequence = 0;
uint16_t beepDebounce = 0;

uint8_t txBuf [RH_RF69_MAX_MESSAGE_LEN];
uint8_t recBuf[RH_RF69_MAX_MESSAGE_LEN];
uint8_t txNo = 0;
uint8_t rxNo = 0xff;
uint8_t len = sizeof(recBuf);
uint8_t txBufPtr = 0;
uint8_t freqChannel;
float radioFreq;

uint8_t tmOut = false;
uint8_t waitForFullBuffer = false;

// frequency channel table
const float channelTable[] = {
  000.000,  // 0 not usable
  433.150,  // 1  10mW            module with red dot!
  433.350,  // 2  10mW
  433.550,  // 3  10mW
  433.750,  // 4  10mW
  433.950,  // 5  10mW
  434.150,  // 6  10mW
  434.350,  // 7  10mW
  434.550,  // 8  10mW
  865.100,  // 9  1% duty, 25mW   module with green dot!
  865.300,  // 10 1% duty, 25mW
  865.500,  // 11 1% duty, 25mW
  865.700,  // 12 1% duty, 25mW
  865.900,  // 13 1% duty, 25mW
  866.100,  // 14 1% duty, 25mW
  866.300,  // 15 1% duty, 25mW
  866.500,  // 16 1% duty, 25mW
  866.700,  // 17 1% duty, 25mW
  866.900,  // 18 1% duty, 25mW
  867.100,  // 19 1% duty, 25mW
  867.300,  // 20 1% duty, 25mW
  867.500,  // 21 1% duty, 25mW
  867.700,  // 22 1% duty, 25mW
  867.900,  // 23 1% duty, 25mW
  868.100,  // 24 1% duty, 25mW
  868.300,  // 25 1% duty, 25mW
  868.500,  // 26 1% duty, 25mW
  868.800,  // 27 0.1% duty, 25mW
  869.000,  // 28 0.1% duty, 25mW
  869.525,  // 29 10% duty 500mW
  869.850,   // 30 1% duty, 25mW
  918.100,   // 31                 
  918.300,   // 32                  
  918.500,   // 33                  
  918.700,   // 34                  
  918.900,   // 35                  
  919.100,   // 36                  
  919.300,   // 37                  
  919.500,   // 38                  
  919.700,   // 39                  
  919.900,   // 40                  
  920.100,   // 41                  
  920.300,   // 42                  
  920.500,   // 43                  
  920.700,   // 44                  
  920.900,   // 45                  
  921.100,   // 46                  
  921.300,   // 47                  
  921.500,   // 48                  
  921.700,   // 49                  
  921.900    // 50                  
};

// The radio encryption key has to be the same as the one on transmitter and receiver!
uint8_t key[] = { 0xFE, 0x02, 0xFC, 0x04, 0xEE, 0x06, 0xEC, 0x08,
                  0x81, 0x92, 0xA3, 0xB4, 0xC5, 0xD6, 0xE7, 0xF8};
                  
// The radio sync word                  
uint8_t sync[] = { 0x2a, 0x54 };  // network, defaults to { 0x2d, 0xd4 }

// conversion of dBm to mW
uint8_t pwrTable[] = {
  10,   // 10dBm
  12,   // 11dBm
  16,   // 12dBm
  20,   // 13dBm
  25,   // 14dBm
  31,   // 15dBm
  40,   // 16dBm
  50,   // 17dBm
  63,   // 18dBm
  79,   // 19dBm
  100   // 20dBm
};


float fUtcTime = 0.0;
int32_t iUtcDate = 0;
char  pst32Status = ' ';
char  pst32Mode = ' ';
uint8_t chk;

uint8_t msgBuf[BUFLEN];
uint8_t rtcmBuf[BUFLEN];
uint16_t msgBufIndex = 0;
uint16_t rtcmBufIndex = 0;
uint16_t msgLength = 0;
uint16_t rtcmLength = 0;
uint8_t msgCheckSum = 0;
String inString = "";
String sUtcTime = "";
String sUtcDate = "";

static const byte hextable[] = { 
    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
    -1,-1, 0,1,2,3,4,5,6,7,8,9,-1,-1,-1,-1,-1,-1,-1,10,11,12,13,14,15,-1,
    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
    -1,-1,10,11,12,13,14,15,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1
};

//
// System setup
//
void setup() 
{
  // debug output
#if defined (debug) || defined (debug1)
  Serial.begin(115200);
  delay(2000);
#endif
  
  // pin mode definitions
  pinMode(LED, OUTPUT);     
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  pinMode(CTRLRECEIVER, INPUT_PULLUP);
  pinMode(CH1IN, INPUT_PULLUP);
  pinMode(CH2IN, INPUT_PULLUP);
  pinMode(CH3IN, INPUT_PULLUP);
  pinMode(KEYL, INPUT_PULLUP);
  pinMode(KEYR, INPUT_PULLUP);
  pinMode(KEYT, INPUT_PULLUP);
  pinMode(RELAIS, OUTPUT);
  pinMode(CH1OUT, OUTPUT);
  pinMode(CH2OUT, OUTPUT);
  pinMode(CH2OUT, OUTPUT);
  pinMode(BEEPER, OUTPUT);

  digitalWrite(RELAIS, LOW);
  digitalWrite(CH1OUT, LOW);
  digitalWrite(CH2OUT, LOW);
  digitalWrite(CH3OUT, LOW);
  digitalWrite(BEEPER, LOW);

#ifdef debug
  Serial.println("UFP2-Feather M0 RFM69 TX Test!");
  Serial.println();
#endif
  
  // manual reset of radio
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  

  // default radio configuration, channel 29
  freqChannel = 29;
  configRadio(freqChannel);

  // Initial dimmer class beeper
  beepCh3.setFrequency(500);
  beepCh3.init();
  beepCh3.setValue(2047);
  delay(250);
  beepCh3.setFrequency(2000);
  beepCh3.init();
  delay(250);
  beepCh3.setFrequency(1000);
  beepCh3.init();
  delay(250);
  beepCh3.setValue(0);
  pwmFrequency = 1000;
  

  // Get operational mode
  if (!digitalRead(KEYL))
      opMode = 0;           // TX mode
  else
      opMode = 1;           // RX mode
      
  if (!digitalRead(KEYR))
      motorOffBeep = true;  // motor off command beep active
    else
      motorOffBeep = false; // no motor off command beep

  // initialize OLED Display
  u8g2.begin();
  oledMessage1();
  delay(4000);
  u8g2.clear();



  // attach interrupts
  if (opMode == 0) {  // all Tx modes
    attachInterrupt(CH1IN, CH1Callback, RISING);
    attachInterrupt(CH2IN, CH2Callback, RISING);
    attachInterrupt(CTRLRECEIVER, RXCallback, RISING);
//    attachInterrupt(CH3IN, CH3Callback, RISING); /* Interrupt EXT0 used by transceiver! */
  }
  
  if (opMode == 1) {  // all Rx modes
    attachInterrupt(CTRLRECEIVER, RXCallback, CHANGE);
    attachInterrupt(CH1IN, CH1Callback, RISING);
//    attachInterrupt(KEYR, KEYRCallback, FALLING);
//    attachInterrupt(KEYT, KEYTCallback, FALLING); // ** this interrupt prevents from receiving **
  }

#ifdef debug
  Serial.print("Operation Mode: "); Serial.println(opMode);
#endif

  // timer @ 1000Hz (1ms)
  startTimer5(987); // fine adjustment of the ms
  
}

//
// Main program loop
//
void loop() {
  if (true) {
    if (opMode == 0) {        // TX modes
      
      // check for crossing signals
      // CH3IN w/o interrupt, but on rising edge only
      if (digitalRead(CH3IN)) {
        if (Ch3inReleased) {
          inStates.chInbits.ch3in = 1;
          Ch3inReleased = false;
        }
      }
      else {
        Ch3inReleased = true;
      } 
      if (inStates.chInbyte) {
        // Send a message!
        txBuf[0] = txNo++;
        txBuf[1] = inStates.chInbyte;
        rf69.send(txBuf, RH_RF69_MESSAGE_LEN);
        // done, clear crossing flags
        inStates.chInbyte = 0;
        beep(MBEEPTIME,1000,15);
        rf69.waitPacketSent();
        // resend message once after short delay
        delay(2);
        rf69.send(txBuf, RH_RF69_MESSAGE_LEN);
        handleLED(50);
        rf69.waitPacketSent();
      }
    }
    else {        // all RX modes   
      // Handle received message
      if (rf69.available()) {
        len = RH_RF69_MAX_MESSAGE_LEN;
        // a message should be received
        if (rf69.recv(recBuf, &len)) {
          if (len == RH_RF69_MESSAGE_LEN) {
            if (recBuf[0] != rxNo) {
              // new message received, not just a copy
              rxNo = recBuf[0];
              if (recBuf[1] >= 128) {
                rRangeTest = true;
                rRangeTestTime = millis();
              }
              recBuf[1] = recBuf[1] & 0x7F;
              recStates.chInbyte = recBuf[1];
              if (recStates.chInbyte)
                handleRelais();
              if (recStates.chInbits.ch1in) 
                handleCh1();
              if (recStates.chInbits.ch2in)
                handleCh2();
              if (recStates.chInbits.ch3in)
                handleCh3();
              recStates.chInbyte = 0;
              handleRaceEvents(2); // got B signal
              handleLED(100);
            }
            else {
              ;
              //Serial.println("Backup message dropped.");
            }
          }
          else {
#ifdef debug
            Serial.println("Invalid message length");
#endif
          }
        } else {
#ifdef debug
          Serial.println("Receive failed");
#endif
        }
      }
      
      //
      // check for local commands (typically at A-Base)
      //
      if (digitalRead(CH3IN)) {    // start signal on channel 3 at A-Base
        if (Ch3inReleased) {
          Ch3inReleased = false;
          handleRaceEvents(0);    // got Start signal
        }
      }
      else {
        Ch3inReleased = true;
      }
      // clear crossing flags
      inStates.chInbyte = 0;
    }
  }

  
  //
  // general tasks in main loop
  //
  
  // life sign, blink LED
  if (t_1000msFlag) {
    t_1000msFlag = false;
    // Range Test
    if (rangeTest) {
      txBuf[0] = txNo++;
      txBuf[1] = 0x83;  // msb set
      rf69.send(txBuf, RH_RF69_MESSAGE_LEN);
      rf69.waitPacketSent();
      beep(MBEEPTIME,1000,15);
      handleLED(100);
    } else
      handleLED(10);
  }
  
  if (t_250msFlag) {
    t_250msFlag = false;
    // Update OLED display
    if (opMode == 1) {   //  Rx modes
      oledMessage2();
      pulseDuration = 0;
    }
  }

} //*** End of Main *********************************************************//



void CH1Callback(void) {

    // sw debounce
    shortDebounceDelay();
    if (digitalRead(CH1IN)) {
      inStates.chInbits.ch1in = 1;   // indicate channel 1 input signal interrupt
    if (opMode == 1) {    // all Rx modes
      handleRelais();
      handleCh1();
      inStates.chInbits.ch1in = 0;
      handleRaceEvents(1);  // got A signal
    }
  }
}
void CH2Callback(void){
  // sw debounce
  shortDebounceDelay();
  if (digitalRead(CH2IN)) {
    inStates.chInbits.ch2in = 1;   // indicate channel 2 input signal interrupt
  }
}
void CH3Callback(void){
  // sw debounce
  shortDebounceDelay();
  if (digitalRead(CH3IN)) {
    inStates.chInbits.ch3in = 1;   // indicate channel 3 input signal interrupt
  }
}

void RXCallback(void){
  if (opMode == 1) {    // all RX modes
    if (digitalRead(CTRLRECEIVER)) {
      pulseStart = micros();
      pulseDetectTimer = 150;   // ms
    }
    else {
      pulseDuration = micros() - pulseStart;
      if ((pulseDuration < 1350) && (pulseDuration > 995)) {
        // on/off filter (delay of one receiver servo cycle)
        if (onCount)
          onCount--;
        if ((engineState == 2) && (!onCount)) {
          engineState = 1;     // engine stopped
          t_engineOffTimer1 = 0;
          t_engineOffTimer2 = 0;
          t_EngineStartTime = 0;
        }
      }
      else {
        if ((pulseDuration >= 1350) && (pulseDuration < 3000)) {
          if (onCount < 2)
            onCount++;
          if ((engineState == 1) && (onCount == 2)) {
            engineState = 2;
            t_EngineStartTime = (millis());
            if ((opMode == 1) && (motorOffBeep)) {
              t_engineOffTimer1 = CLIMB4LEGTIME;
              t_engineOffTimer2 = CLIMB6LEGTIME;
            }
            handleRaceEvents(3);  // got motor ON signal
          }
        }
        else {
          // ignore
        }
      }
    }
  }
  else
  { // Tx modes
    // use RX channel as an alternative channel 1 input, so FCD can supply UFP2
    inStates.chInbits.ch1in = 1;    // indicate ch 1 interrupt
  }
}

void KEYRCallback(void){
  
}
void KEYTCallback(void){
  
}

// signal timing methods

// duration: 2..2000
// freq:     500..6000
// duty:     0..50      (volume 0..100%)

void beep(uint16_t duration, uint16_t freq, uint16_t duty) {
  beepCh3.setValue(0);  // stop current beep
  if (duration < 2) duration = 2;
  if (duration > 2000) duration = 2000;
  if (freq < 1000) freq = 500;
  else if (freq < 2000) freq = 1000;
  else if (freq < 6000) freq = 2000;
  if (duty > 50) duty = 50;
  beepTime = duration;
  if (pwmFrequency != freq){
//    pwm12Init(freq);
    beepCh3.setFrequency(freq);
    beepCh3.init();
    pwmFrequency = freq;
  }
  if (duty < 5)
    beepCh3.setValue(0);
  else if (duty < 25)
    beepCh3.setValue(1000);
  else
    beepCh3.setValue(2047);
  //pwm12SetDuty(duty);
}

void handleLED(uint16_t ms) {
  if (ms < 2) ms = 2;
  digitalWrite(LED,HIGH);
  ledTime = ms;
}
void handleRelais(void) {
  digitalWrite(RELAIS, HIGH);
  relaisTime = SIGNALTIME;
}
void handleCh1(void) {
  digitalWrite(CH1OUT, HIGH);
  ch1Time = SIGNALTIME;
}
void handleCh2(void) {
  digitalWrite(CH2OUT, HIGH);
  ch2Time = SIGNALTIME;
}
void handleCh3(void) {
  digitalWrite(CH3OUT, HIGH);
  ch3Time = SIGNALTIME;
}

void handleTimings(void) {
  // LED
  if (ledTime > 1) {
    ledTime--;
  } else {
    if (ledTime == 1) {
      ledTime = 0;
      digitalWrite(LED, LOW);
    }
  }
  // beeper
  if (beepTime > 1) {
    beepTime--;
  } else {
    if (beepTime == 1) {
      beepTime = 0;
//      pwm12SetDuty(0);
      beepCh3.setValue(0);
    }
  }
  // relais
  if (relaisTime > 1) {
    relaisTime--;
  } else {
    if (relaisTime == 1) {
      relaisTime = 0;
      digitalWrite(RELAIS, LOW);
    }
  }
  // ch1 signal
  if (ch1Time > 1) {
    ch1Time--;
  } else {
    if (ch1Time == 1) {
      ch1Time = 0;
      digitalWrite(CH1OUT, LOW);
    }
  }
  // ch2 signal
  if (ch2Time > 1) {
    ch2Time--;
  } else {
    if (ch2Time == 1) {
      ch2Time = 0;
      digitalWrite(CH2OUT, LOW);
    }
  }
  // ch3 signal
  if (ch3Time > 1) {
    ch3Time--;
  } else {
    if (ch3Time == 1) {
      ch3Time = 0;
      digitalWrite(CH3OUT, LOW);
    }
  }

  if (pulseDetectTimer) {
    pulseDetectTimer--;
    if (!engineState)
      engineState = 1;
  }
  else
    engineState = 0;     // no engine signal from reference receiver

  // engine off timer
  if (t_EngineStartTime) {
    if (t_engineOffTimer1 > 1)
      t_engineOffTimer1--;
    else
      if (t_engineOffTimer1 == 1) {
        t_engineOffTimer1 = 0;
        beep(EBEEPTIME,3500,50);
        handleLED(200);
      }
    if (t_engineOffTimer2 > 1)
      t_engineOffTimer2--;
    else
      if (t_engineOffTimer2 == 1) {
        t_engineOffTimer2 = 0;
        beep(EBEEPTIME,3500,50);
        handleLED(200);
      }
  }
  handleRaceEvents(4);
}

//
// timer functions
//
void setTimerFrequency(int frequencyHz) {
  int compareValue = (CPU_HZ / (TIMER_PRESCALER_DIV * frequencyHz)) - 1;
  TcCount16* TC = (TcCount16*) TC5;
  // Make sure the count is in a proportional position to where it was
  // to prevent any jitter or disconnect when changing the compare value.
  TC->COUNT.reg = map(TC->COUNT.reg, 0, TC->CC[0].reg, 0, compareValue);
  TC->CC[0].reg = compareValue;
  while (TC->STATUS.bit.SYNCBUSY == 1);
}

void startTimer5(int frequencyHz) {
  REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TC4_TC5) ;
  while ( GCLK->STATUS.bit.SYNCBUSY == 1 ); // wait for sync

  TcCount16* TC = (TcCount16*) TC5;

  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use the 16-bit timer
  TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use match mode so that the timer counter resets when the count matches the compare register
  TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Set prescaler to 1024
  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  setTimerFrequency(frequencyHz);

  // Enable the compare interrupt
  TC->INTENSET.reg = 0;
  TC->INTENSET.bit.MC0 = 1;

  NVIC_EnableIRQ(TC5_IRQn);

  TC->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
}

void TC5_Handler() {
  
  TcCount16* TC = (TcCount16*) TC5;
  if (TC->INTFLAG.bit.MC0 == 1) {
    TC->INTFLAG.bit.MC0 = 1;
    //
    // Callback application code
    //
    t_1msFlag = true;                                // reset in main loop
    
    handleTimings();
    
    if (!(--t_1000ms_count))                          // Count to 1s
    {
      t_1000ms_count = TICK_COUNTS_1S;             // Reload
      t_1000msFlag = true;
    }
    if (!(--t_250ms_count))                          // Count to 250ms
    {
      t_250ms_count = TICK_COUNTS_250MS;             // Reload
      t_250msFlag = true;
    }
  }
}


void oledMessage1(void) {
#ifdef SH1106_P
  u8g2.firstPage();
  do {
#endif
#ifdef SH1106_F
  u8g2.clearBuffer();
#endif
    u8g2.setFont(u8g2_font_mercutio_sc_nbp_tr);
    u8g2.drawStr(0, 10, "UFP-II    Advanced    V");
    u8g2.drawStr(104, 10, dtostrf(SWversion, 4, 2, s));
    if (opMode == 1)
      u8g2.drawStr(5, 30, "Mode:    Rx");
    else
      u8g2.drawStr(5, 30, "Mode:    Tx");
    if (motorOffBeep)
      u8g2.drawStr(60, 30, "MB");
    u8g2.drawStr(5, 45, "Freq:");
    u8g2.drawStr(44, 45, dtostrf(radioFreq, 7, 3, s));
    u8g2.drawStr(86, 45, "MHz");
    sprintf(s," CH: %d",freqChannel);
    u8g2.drawStr(76, 30, s);
    sprintf(s,"TxPwr:  %d mW",txPwrMw);
    u8g2.drawStr(5, 60, s);
#ifdef SH1106_P
  } while (u8g2.nextPage());
#endif
#ifdef SH1106_F
  u8g2.sendBuffer();
#endif
}


void oledMessage2(void) {
  int16_t rssi;
  
#ifdef SH1106_P
  u8g2.firstPage();
  do {
#endif
#ifdef SH1106_F
  u8g2.clearBuffer();
#endif
    u8g2.setFont(u8g2_font_mercutio_sc_nbp_tr);
    u8g2.drawStr(0, 10, "UFP-II    Advanced    V");
    u8g2.drawStr(104, 10, dtostrf(SWversion, 4, 2, s));
    u8g2.drawStr(0, 45, "State:");

    u8g2.drawStr(0, 30, "Legs:");
    if (startSequence == RUN) {
        u8g2.drawStr(50, 45, "RUN  ");
        sprintf(s,"%d.%d s",roundtripCounter/1000,(roundtripCounter%1000)/100);
        u8g2.drawStr(85, 45, s);
        if (engineReportedOn ) {
          u8g2.drawStr(50, 30, "*");
        }
    }
    if (startSequence == START) {
        u8g2.drawStr(50, 45, "START");
    }
    if (startSequence == IDLE) {
      u8g2.drawStr(50, 45, "IDLE");
    }
    if (rRangeTest) {
      u8g2.drawStr(50, 45, "RANGE TEST");
      u8g2.drawStr(0, 60, "RSSI:");
      rssi = rf69.lastRssi();
      sprintf(s,"%d dBm",rssi/2);
      u8g2.drawStr(50,60,s);
    } else {
      sprintf(s,"RxPulse:  %d us",pulseDuration);
      u8g2.drawStr(0, 60, s);
      if (engineState == ON) 
        u8g2.drawStr(98, 60, "On");
      else
        if (engineState == OFF) 
          u8g2.drawStr(98, 60, "Off");
        else
          if (engineState == INV)
            u8g2.drawStr(98, 60, "inv");
    }
    u8g2.setFont(u8g2_font_logisoso16_tn);
    sprintf(s,"%d",legCount);
    u8g2.drawStr(78, 31, s);

#ifdef SH1106_P
  } while (u8g2.nextPage());
#endif
#ifdef SH1106_F
  u8g2.sendBuffer();
#endif
}


//uint8_t getParamsFromFram(void) {
//  // t.b.d.
//  return false;
//}

void configRadio(uint8_t ch) {
  if (!rf69.init()) {
#ifdef debug
    Serial.println("RFM69 radio init failed");
#endif
    while (1);
  }
#ifdef debug
  Serial.println("RFM69 radio init OK!");
#endif  
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption


//  if (!rf69.setModemConfig(rf69.GFSK_Rb125Fd125)) {   // 125kbit/s, about 1ms on air time
  if (!rf69.setModemConfig(rf69.GFSK_Rb55555Fd50)) {    // 55kbit/s, about 2.5ms on air time
#ifdef debug
    Serial.println("setModemConfig failed");
#endif
  }

  radioFreq = channelTable[ch];
  if (!rf69.setFrequency(radioFreq)) {
#ifdef debug
    Serial.println("setFrequency failed");
#endif
  }
  // set TX power
  if (ch < 9) {
    // 10mW
    rf69.setTxPower(TXPWRL, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW
    txPwr = TXPWRL;
  }
  else {
    if (ch != 29) {
      // 25mW
      rf69.setTxPower(TXPWRH, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW
      txPwr = TXPWRH;
    }
    else {
      // 100mW
      rf69.setTxPower(TXPWRHH, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW
      txPwr = TXPWRHH;
    }
  }
  txPwrMw = pwrTable[txPwr-13]; // accounts for power setting error in RFM69
  
  rf69.setEncryptionKey(key);

  // must be the same network for Tx and Rx
  rf69.setSyncWords(sync, 2);
#ifdef debug  
  Serial.print("RFM69 radio @");  Serial.print((int)radioFreq);  Serial.println(" MHz");
#endif
}

void shortDebounceDelay(void) {
  volatile uint32_t cnt=0;
  for (int i = 0; i<5000; i++) {
    cnt++;
  }
}


//
//  handleRaceEvents Function called on various events:
//
//    - (0) START signal (ready to start race)
//    - (1) A signal
//    - (2) B signal
//    - (3) engine ON signal
//    - (4) update rount trip timer, called every ms
//
//  startSequence:
//    0: IDLE
//    1: START    (waiting for first engine ON signal)
//    2: RUNNING  (200s round trip down counter started)
//
//  engineState:
//    0: no receiver/motor channel detected
//    1: engine OFF
//    2: engine ON
//
//  roundtripCounter: 200s down counter, updated every ms
//
//  legCount: counts valid legs (A->B and B->A)
//
void handleRaceEvents(uint8_t eventType) {

  switch (eventType) {
    case 0:
      // got START signal
      if (startSequence == IDLE) {
        startSequence = START;      // set to START
        roundtripCounter = 0;
        legCount = 0;
        lastLegUpdate = A_TURN;
      }
      else {
        if ((startSequence == RUN)||(startSequence == START)){
          startSequence = IDLE;      // reset to IDLE
        }
      }
      engineReportedOn = false;
    break;
    case 1:
      // got A signal
      noInterrupts();
      if (startSequence == RUN) {
        if (engineState == OFF) {
          // check for possible leg count update
          if (lastLegUpdate == B_TURN) {
            if (!engineReportedOn) {
              legCount++;
              beep(BEEPTIME,1000,50); // normal beep
            }
            else {
              beep(BEEPTIME,500,50);  // alert beep, forbidden engine ON detected
              engineReportedOn = false;
            }
          } 
          beep(BEEPTIME,1000,50);     // normal beep
          engineReportedOn = false;
        }
        else {
          beep(BEEPTIME,500,50);      // alert beep, forbidden engine ON at A-turn detected
        }
        lastLegUpdate = A_TURN;
      }
      interrupts();
      if (startSequence == START) {
        beep(BEEPTIME,500,50);        // alert beep, round not in RUN state
      }
      if (startSequence == IDLE) {
        beep(BEEPTIME,1000,50);       // normal beep
      }
    break;
    case 2:
      // got B signal
      noInterrupts();
      if (startSequence == RUN) {
        if (lastLegUpdate == A_TURN) {
          if (!engineReportedOn) {
            legCount++;
            lastLegUpdate = B_TURN;
            beep(BEEPTIME,1000,50);   // normal beep
          }
          else {
            beep(BEEPTIME,500,50);    // alert beep, forbidden engine ON from A-Turn to B-turn
          }
        }
        beep(BEEPTIME,1000,50);       // normal beep
      }
      interrupts();
      if (startSequence == START) {
        beep(BEEPTIME,500,50);        // alert beep, round not in RUN state
      }
      if (startSequence == IDLE) {
        beep(BEEPTIME,1000,50);       // normal beep
      }
    break;
    case 3:
      // Got engine ON signal
      if (startSequence == START) {
        // Round trip automatic start
        if (roundtripCounter == 0) {
          roundtripCounter = 200001;    // set counter/timer to 200s
          startSequence = RUN;          // engine ON sets start sequence to RUN
          engineReportedOn = false;
        }
      }
      if (startSequence == RUN) {
        engineReportedOn = true;
      }
    break;
    case 4:
      // update round trip timer
      if (startSequence == RUN) { 
        if (roundtripCounter > 1)
          roundtripCounter--;
        else {
          if (roundtripCounter == 1) {
            roundtripCounter = 0;
            beep(RTEEPTIME, 2500, 50);  // finish beep, end of round!
            startSequence = IDLE;
          }
        }
      }
    break;
  }
}

