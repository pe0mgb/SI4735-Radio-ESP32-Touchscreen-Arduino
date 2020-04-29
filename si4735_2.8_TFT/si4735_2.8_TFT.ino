//  Version 1.1 PE0MGB                            17-04-2020
//  MUTE added. Volume controle with encoder      17-04-2020
//  Version 1.0 PE0MGB                            13-04-2020
//  Minor changes and bugs                        12-04-2020
//  Minor changes and bugs                        09-04-2020
//  RDS added                                     31-03-2020
//  Preset bug solved                             30-03-2020
//  Wake-up display also with encoder             29-03-2020
//  Some bugs solved                              25-03-2020
//  AGC corrected                                 24-03-2020

//  This sketch is based on the si4735 Library of Ricardo PU2CLR. Thanks for the very nice work.

//  This sketch uses  a 2.8 inch 240*320 touch-screen with ILI9341, ESP32 WROOM-32 and Rotary Encoder.
//  The radio is fully controlled by the (Touch)Screen.

//  This sketch uses the Rotary Encoder Class implementation from Ben Buxton (the source code is included
//  together with this sketch).
//  For the touch-screen the library TFT_eSPI is used. The configuration setup-file: setup1_ILI9341 is also
//  included.
//  Also a schematic drawing is available.

//  ABOUT SSB PATCH:
//  This sketch will download a SSB patch to your SI4735 device (patch_init.h). It will take about 8KB of the Arduino memory.

//  In this context, a patch is a piece of software used to change the behavior of the SI4735 device.
//  There is little information available about patching the SI4735. The following information is the understanding of the author of
//  this project and it is not necessarily correct. A patch is executed internally (run by internal MCU) of the device.
//  Usually, patches are used to fixes bugs or add improvements and new features of the firmware installed in the internal ROM of the device.
//  Patches to the SI4735 are distributed in binary form and have to be transferred to the internal RAM of the device by
//  the host MCU (in this case Arduino). Since the RAM is volatile memory, the patch stored into the device gets lost when you turn off the system.
//  Consequently, the content of the patch has to be transferred again to the device each time after turn on the system or reset the device.

//  ATTENTION: The author of this project does not guarantee that procedures shown here will work in your development environment.
//  Given this, it is at your own risk to continue with the procedures suggested here.
//  This library works with the I2C communication protocol and it is designed to apply a SSB extension PATCH to CI SI4735-D60.
//  Once again, the author disclaims any liability for any damage this procedure may cause to your SI4735 or other devices that you are using.
//
//  *********************************
//  **   Display connections etc.  **
//  *********************************
//  |------------|------------------|------------|------------|------------|
//  |Display 2.8 |      ESP32       |   Si4735   |  Encoder   |  Beeper    |
//  |  ILI9341   |                  |            |            |            |        Encoder        1,2,3
//  |------------|------------------|------------|------------|------------|        Encoder switch 4,5
//  |   Vcc      |     3V3     | 01 |    Vcc     |            |            |        pin 33 with 18K to 3.3 volt and 18K to ground.
//  |   GND      |     GND     | 02 |    GND     |     2,4    |            |        pin 32 (Beeper) via 2K to base V1  BC547
//  |   CS       |     15      | 03 |            |            |            |        Collector via beeper to 5v
//  |   Reset    |      4      | 04 |            |            |            |        Emmitor to ground
//  |   D/C      |      2      | 05 |            |            |            |
//  |   SDI      |     23      | 06 |            |            |            |        Encoder        1,2,3
//  |   SCK      |     18      | 07 |            |            |            |        Encoder switch 4,5
//  |   LED Coll.|     14 2K   | 08 |            |            |            |        Display LED
//  |   SDO      |     19      | 09 |            |            |            |        Emmitor  V2 BC557 to 3.3 V
//  |   T_CLK    |     18      | 10 |            |            |            |        Base with 2K to pin 14 (Display_Led)
//  |   T_CS     |      5      | 11 |            |            |            |        Collector to led pin display
//  |   T_DIN    |     23      | 12 |            |            |            |
//  |   T_DO     |     19      | 13 |            |            |            |
//  |   T_IRQ    |     34      | 14 |            |            |            |
//  |            |     12      |    |   Reset    |            |            |
//  |            |     21      |    |    SDA     |            |            |
//  |            |     22      |    |    SCL     |            |            |
//  |            |     16      |    |            |      1     |            |
//  |            |     17      |    |            |      3     |            |
//  |            |     33      |    |            |      5     |            |
//  |            |     32 2K   |    |            |            |     In     |
//  |            |             |    |            |            |            |
//  |------------|-------------|----|------------|------------|------------|


#include <TFT_eSPI.h>
#include <SPI.h>
#include <SI4735.h>
#include "Rotary.h"

// Test it with patch_init.h or patch_full.h. Do not try load both.
#include "patch_init.h" // SSB patch for whole SSBRX initialization string
//#include "patch_full.h"    // SSB patch for whole SSBRX full download

const uint16_t size_content = sizeof ssb_patch_content; // see ssb_patch_content in patch_full.h or patch_init.h

#define ESP32_I2C_SDA    21  // I2C bus pin on ESP32
#define ESP32_I2C_SCL    22  // I2C bus pin on ESP32
#define RESET_PIN        12
#define ENCODER_PIN_A    17  // http://www.buxtronix.net/2011/10/rotary-encoders-done-properly.html
#define ENCODER_PIN_B    16
#define ENCODER_SWITCH   33
#define BEEPER           32
#define Display_Led      14
#define displayon         0
#define displayoff        1
#define beepOn            1
#define beepOff           0
#define AUDIO_MUTE       27

#define FM_BAND_TYPE 0
#define MW_BAND_TYPE 1
#define SW_BAND_TYPE 2
#define LW_BAND_TYPE 3

#define MIN_ELAPSED_TIME       100
#define MIN_ELAPSED_RSSI_TIME  150
#define MIN_ELAPSED_DISPL_TIME 1000
#define MIN_ELAPSED_RDS_TIME   5
#define DEFAULT_VOLUME         45 // change it for your favorite start sound volume
#define MIN_ELAPSED_VOLbut_TIME 1000

#define FM          0
#define LSB         1
#define USB         2
#define AM          3

bool bfoOn          = false;
bool ssbLoaded      = false;

bool FirstLayer = true;
bool SecondLayer = false;
bool ThirdLayer = false;
bool ForthLayer = false;
bool HamBand = false;
bool Modebut = false;
bool FREQbut = false;
bool Decipoint = false;
bool STEPbut = false;
bool BroadBand;
bool BandWidth;
bool MISCbut = false;
bool PRESbut = false;
bool VOLbut = false;
bool DISplay = false;
bool Mutestat = false;

bool pressed;
bool press1;
bool audioMuteOn = true;
bool audioMuteOff = false;
bool RDS = true; // RDS on  or  off
bool SEEK =  false;

int currentBFO    = 0;
int previousBFO   = 0;
int currentPRES   = 0;
int previousPRES  = 0;
int currentVOL    = 0;
int previousVOL   = 0;
int MaxVOL        = 63;
int MinVOL        = 20;

int nrbox = 0;
int OldRSSI;
int NewRSSI;
int NewSNR;
int encBut;
long elapsedRSSI        = millis();
long elapsedRDS         = millis();
long DisplayOnTime      = millis();
long VOLbutOnTime       = millis();


// Encoder control variables
volatile int encoderCount = 0;

uint16_t previousFrequency;
uint8_t currentStep = 1;

uint8_t currentBFOStep = 25;
uint8_t currentPRESStep = 1;
uint8_t currentVOLStep = 1;

uint8_t currentAGCAtt = 0;
uint8_t bwIdxSSB;
uint8_t bwIdxAM;
uint8_t bandIdx;
uint8_t stepIdx;
uint8_t currentMode = FM;
uint8_t previousMode;
uint16_t x = 0, y = 0; // To store the touch coordinates


float Displayfreq = 0;
float currentFrequency = 0;
float dpfrq = 0;
float fact = 1;

String BWtext;

const char *bandwidthSSB[] = {"1.2", "2.2", "3.0", "4.0", "0.5", "1.0"};
const char *bandwidthAM[]  = {"6", "4", "3", "2", "1", "1.8", "2.5"};
const char *Keypathtext[]  = {"1", "2", "3", "4", "5", "6", "7", "8", "9", ".", "0", "Send", "Clear"};
const char *misctext[]     = {"STATUS", "Seekup", "Seekdown", "SSB avc-on", "SSB avc-off", "RDS on/off"};
const char *bandModeDesc[] = {"FM ", "LSB", "USB", "AM "};

char buffer[64];          // Useful to handle string

char *rdsMsg;
char *stationName;
char *prevstationName;
char *rdsTime;
char bufferStatioName[255];
char bufferRdsMsg[255];
char bufferRdsTime[32];


//=======================================================   Buttons First Layer   ==========================
typedef struct // Buttons first layer
{
  const char *ButtonNam;
  uint8_t     ButtonNum;       // Button location at display from 0 to 11. To move around buttons freely at first layer.
  const char *ButtonNam1;
  uint8_t     ButtonNum1;       // Button location at display from 0 to 11. To move around buttons freely at first layer.
  uint16_t    XButos;          // Xoffset
  uint16_t    YButos;          // Yoffset
} Button;
int ytotoffset = 0;
//  Button table
int Xbutst  =   0;               // X Start location Buttons
int Ybutst  = 160 + ytotoffset;  // Y Start location Buttons

int Xsmtr   =   0;
int Ysmtr   =  85 + ytotoffset;  // S meter

int XVolInd =   0;
int YVolInd = 135 + ytotoffset;  // Volume indicator

int XFreqDispl  =   0;
int YFreqDispl  =   0 + ytotoffset;  // display

int Xbutsiz =  80;  //size of buttons first layer
int Ybutsiz =  40;

int Xbut0  = 0 * Xbutsiz ; int Ybut0  = 0 * Ybutsiz; // location calqualation for 12 first layer buttons
int Xbut1  = 1 * Xbutsiz ; int Ybut1  = 0 * Ybutsiz;
int Xbut2  = 2 * Xbutsiz ; int Ybut2  = 0 * Ybutsiz;
int Xbut3  = 0 * Xbutsiz ; int Ybut3  = 1 * Ybutsiz;
int Xbut4  = 1 * Xbutsiz ; int Ybut4  = 1 * Ybutsiz;
int Xbut5  = 2 * Xbutsiz ; int Ybut5  = 1 * Ybutsiz;
int Xbut6  = 0 * Xbutsiz ; int Ybut6  = 2 * Ybutsiz;
int Xbut7  = 1 * Xbutsiz ; int Ybut7  = 2 * Ybutsiz;
int Xbut8  = 2 * Xbutsiz ; int Ybut8  = 2 * Ybutsiz;
int Xbut9  = 0 * Xbutsiz ; int Ybut9  = 3 * Ybutsiz;
int Xbut10 = 1 * Xbutsiz ; int Ybut10 = 3 * Ybutsiz;
int Xbut11 = 2 * Xbutsiz ; int Ybut11 = 3 * Ybutsiz;

#define HAM       0
#define BFO       1
#define FREQ      2
#define AGC       3
#define MUTE      4
#define VOL       5
#define MODE      6
#define BANDW     7
#define STEP      8
#define BROAD     9
#define PRESET   10
#define NEXT     11

#define SEEKUP    0
#define SEEKDN    1
#define STATUS    2
#define NR2       3
#define NR3       4
#define NR4       5
#define NR5       6
#define NR6       7
#define NR7       8
#define NR8       9
#define NR9      10
#define PREV     11

Button bt[] = {
  { "HAM"   ,  0 , "SEEKUP", 0 , Xbut0 , Ybut0  }, //     |----|----|----|       |----|----|----|
  { "BFO"   ,  3 , "SEEKDN", 3 , Xbut1 , Ybut1  }, //     |  0 |  1 |  2 |
  { "FREQ"  ,  2 , "STATUS",10 , Xbut2 , Ybut2  }, //     |----|----|----|
  { "AGC"   ,  4 , ""      , 1 , Xbut3 , Ybut3  }, //     |  3 |  4 |  5 |
  { "MUTE"  ,  8 , ""      , 4 , Xbut4 , Ybut4  }, //     |----|----|----|
  { "VOL"   ,  7 , ""      , 5 , Xbut5 , Ybut5  }, //     |  6 |  7 |  8 |
  { "MODE"  ,  9 , ""      , 6 , Xbut6 , Ybut6  }, //     |----|----|----|
  { "BANDW" ,  5 , ""      , 7 , Xbut7 , Ybut7  }, //     |  9 | 10 | 12 |
  { "STEP"  ,  6 , ""      , 8 , Xbut8 , Ybut8  }, //     |----|----|----|
  { "BROAD" ,  1 , ""      , 9 , Xbut9 , Ybut9  },
  { "PRESET", 10 , ""      , 2 , Xbut10, Ybut10 },
  { "NEXT"  , 11 , "PREV", 11 , Xbut11, Ybut11 }
};


//======================================================= End  Buttons First Layer   ======================


//======================================================= Tunings Steps     ===============================
typedef struct // Tuning steps
{
  uint16_t stepFreq;
  uint16_t Xstepos;          //Xoffset
  uint16_t Xstepsr;          //X size rectang
  uint16_t Ystepos;          //Yoffset
  uint16_t Ystepsr;          //Y size rectang
  uint16_t Ystepnr;          //Y next rectang
} Step;

//  Tuning steps table

int Xfstep = 80;
int Yfstep = 90;

Step sp[] = {
  { 1 , Xfstep, 100, Yfstep, 30, 0},
  { 5 , Xfstep, 100, Yfstep, 30, 30},
  { 9 , Xfstep, 100, Yfstep, 30, 60},
  { 10, Xfstep, 100, Yfstep, 30, 90}
};
//======================================================= End Tunings Steps     ===============================

//======================================================= Modulation Types     ================================
typedef struct // MODULATION
{
  uint16_t Modenum;
  uint16_t Xmodos;          //Xoffset
  uint16_t Xmodsr;          //X size rectang
  uint16_t Ymodos;          //Yoffset
  uint16_t Ymodsr;          //Y size rectang
  uint16_t Ymodnr;          //Y next rectang
} Mode;

//  Modulation table

int Xfmod = 70;
int Yfmod = 90;

Mode md[] = {
  { 0  , Xfmod, 100, Yfmod, 30, 0},
  { 1  , Xfmod, 100, Yfmod, 30, 30},
  { 2  , Xfmod, 100, Yfmod, 30, 60},
  { 3  , Xfmod, 100, Yfmod, 30, 90}
};
//======================================================= End Modulation Types     ============================

//======================================================= Keypath     =========================================
typedef struct // Keypath
{
  uint16_t KeypNum;
  uint16_t Xkeypos;          //Xoffset
  uint16_t Xkeypsr;          //X size rectang
  uint16_t Xkeypnr;          //X next rectang
  uint16_t Ykeypos;          //Yoffset
  uint16_t Ykeypsr;          //X size rectang
  uint16_t Ykeypnr;          //Y next rectang
} Keypath;

//  Keypath table

int Xpath = 30;
int Ypath = 75;

Keypath kp[] = {
  {  0 , Xpath,  60 ,   0 , Ypath , 60 ,   0},
  {  1 , Xpath,  60 ,  60 , Ypath , 60 ,   0},
  {  2 , Xpath,  60 , 120 , Ypath , 60 ,   0},
  {  3 , Xpath,  60 ,   0 , Ypath , 60 ,  60},
  {  4 , Xpath,  60 ,  60 , Ypath , 60 ,  60},
  {  5 , Xpath,  60 , 120 , Ypath , 60 ,  60},
  {  6 , Xpath,  60 ,   0 , Ypath , 60 , 120},
  {  7 , Xpath,  60 ,  60 , Ypath , 60 , 120},
  {  8 , Xpath,  60 , 120 , Ypath , 60 , 120},
  {  9 , Xpath,  60 ,   0 , Ypath , 60 , 180},
  { 10 , Xpath,  60 ,  60 , Ypath , 60 , 180},
  { 11 , Xpath,  60 , 120 , Ypath , 60 , 180},
};
//======================================================= End Keypath     =====================================

//======================================================= Bandwidth AM & FM     ===============================
typedef struct // Bandwidth AM & SSB
{
  uint16_t BandWidthAM;
  uint16_t BandWidthSSB;
  uint16_t Xos;          //Xoffset
  uint16_t Xsr;          //X size rectang
  uint16_t Yos;          //Yoffset
  uint16_t Ysr;          //X size rectang
  uint16_t Ynr;    //Y next rectang
} Bandwidth;

//  Bandwidth table

int XfBW = 70;
int YfBW = 60;

Bandwidth bw[] = {
  { 4 , 4 , XfBW, 100, YfBW, 30, 0},
  { 5 , 5 , XfBW, 100, YfBW, 30, 30},
  { 3 , 0 , XfBW, 100, YfBW, 30, 60},
  { 6 , 1 , XfBW, 100, YfBW, 30, 90},
  { 2 , 2 , XfBW, 100, YfBW, 30, 120},
  { 1 , 3 , XfBW, 100, YfBW, 30, 150},
  { 0 , 0 , XfBW, 100, YfBW, 30, 180}
};
//======================================================= End Bandwidth AM & FM     ===========================

//======================================================= Broad Band Definitions     ==========================
typedef struct // Broad-Band switch
{
  uint16_t BbandNum; // bandIdx
  uint16_t Xbbandos;          //Xoffset
  uint16_t Xbbandsr;          //X size rectang
  uint16_t Xbbandnr;          //X next rectang
  uint16_t Ybbandos;          //Yoffset
  uint16_t Ybbandsr;          //X size rectang
  uint16_t Ybbandnr;          //Y next rectang
} BBandnumber;

//  Bandnumber table for the broad-bands

int Xfbband = 10;
int Yfbband = 30;

BBandnumber bb[] = {
  {  0 , Xfbband, 110 ,   0 , Yfbband , 30 ,   0}, // 0
  {  1 , Xfbband, 110 ,   0 , Yfbband , 30 ,  30}, // 1
  {  2 , Xfbband, 110 ,   0 , Yfbband , 30 ,  60}, // 2
  {  6 , Xfbband, 110 ,   0 , Yfbband , 30 ,  90}, // 3
  {  7 , Xfbband, 110 ,   0 , Yfbband , 30 , 120}, // 4
  {  9 , Xfbband, 110 ,   0 , Yfbband , 30 , 150}, // 5
  { 11 , Xfbband, 110 ,   0 , Yfbband , 30 , 180}, // 6
  { 13 , Xfbband, 110 ,   0 , Yfbband , 30 , 210}, // 7
  { 14 , Xfbband, 110 ,   0 , Yfbband , 30 , 240}, // 8
  { 16 , Xfbband, 110 , 110 , Yfbband , 30 ,   0}, // 9
  { 17 , Xfbband, 110 , 110 , Yfbband , 30 ,  30}, //10
  { 19 , Xfbband, 110 , 110 , Yfbband , 30 ,  60}, //11
  { 21 , Xfbband, 110 , 110 , Yfbband , 30 ,  90}, //12
  { 22 , Xfbband, 110 , 110 , Yfbband , 30 , 120}, //13
  { 24 , Xfbband, 110 , 110 , Yfbband , 30 , 150}, //14
  { 26 , Xfbband, 110 , 110 , Yfbband , 30 , 180}, //15
  { 27 , Xfbband, 110 , 110 , Yfbband , 30 , 210}, //16
  { 29 , Xfbband, 110 , 110 , Yfbband , 30 , 240}, //17

};
//======================================================= End Broad Band Definitions     ======================

//======================================================= Ham Band Definitions     ============================
typedef struct // Ham Band switch
{
  uint16_t BandNum; // bandIdx
  uint16_t HamBandTxt;
  uint16_t Xbandos;          //Xoffset
  uint16_t Xbandsr;          //X size rectang
  uint16_t Xbandnr;          //X next rectang
  uint16_t Ybandos;          //Yoffset
  uint16_t Ybandsr;          //Y size rectang
  uint16_t Ybandnr;          //Y next rectang
} Bandnumber;

//  Bandnumber table for the hambands

int Xfband = 10;
int Yfband = 30;

Bandnumber bn[] = {
  {  3 , 0 , Xfband, 110 ,   0 , Yfband , 30 ,   0},
  {  4 , 1 , Xfband, 110 ,   0 , Yfband , 30 ,  30},
  {  5 , 2 , Xfband, 110 ,   0 , Yfband , 30 ,  60},
  {  8 , 3 , Xfband, 110 ,   0 , Yfband , 30 ,  90},
  { 10 , 4 , Xfband, 110 ,   0 , Yfband , 30 , 120},
  { 12 , 5 , Xfband, 110 ,   0 , Yfband , 30 , 150},
  { 15 , 6 , Xfband, 110 , 110 , Yfband , 30 ,   0},
  { 18 , 7 , Xfband, 110 , 110 , Yfband , 30 ,  30},
  { 20 , 8 , Xfband, 110 , 110 , Yfband , 30 ,  60},
  { 23 , 9 , Xfband, 110 , 110 , Yfband , 30 ,  90},
  { 25 , 10 , Xfband, 110 , 110 , Yfband , 30 , 120},
  { 28 , 11 , Xfband, 110 , 110 , Yfband , 30 , 150}
};
//======================================================= End Ham Band Definitions     ========================

//======================================================= THE Band Definitions     ============================
typedef struct // Band data
{
  const char *bandName; // Bandname
  uint8_t  bandType;    // Band type (FM, MW or SW)
  uint16_t prefmod;     // Pref. modulation
  uint16_t minimumFreq; // Minimum frequency of the band
  uint16_t maximumFreq; // maximum frequency of the band
  uint16_t currentFreq; // Default frequency or current frequency
  uint16_t currentStep; // Default step (increment and decrement)
} Band;

//   Band table

Band band[] = {
  {   "FM", FM_BAND_TYPE, FM,  8750, 10800,  9890, 10}, //  FM          0
  {   "LW", LW_BAND_TYPE, AM,   153,   279,   198, 9},  //  LW          1
  {   "MW", MW_BAND_TYPE, AM,   522,  1701,  1008, 9},  //  MW          2
  {"BACON", LW_BAND_TYPE, AM,   280,   470,   284, 5},  // Ham          3
  { "630M", SW_BAND_TYPE, LSB,   472,   479,   475, 1}, // Ham  630M    4
  { "160M", SW_BAND_TYPE, LSB,  1800,  1910,  1899, 1}, // Ham  160M    5
  { "120M", SW_BAND_TYPE, AM,  2300,  2495,  2400, 5},  //      120M    6
  {  "90M", SW_BAND_TYPE, AM,  3200,  3400,  3300, 5},  //       90M    7
  {  "80M", SW_BAND_TYPE, LSB,  3500,  3800,  3700, 1}, // Ham   80M    8
  {  "75M", SW_BAND_TYPE, AM,  3900,  4000,  3950, 5},  //       75M    9
  {  "60M", SW_BAND_TYPE, USB,  5330,  5410,  5375, 1}, // Ham   60M   10
  {  "49M", SW_BAND_TYPE, AM,  5900,  6200,  6000, 5},  //       49M   11
  {  "40M", SW_BAND_TYPE, LSB,  7000,  7200,  7132, 1}, // Ham   40M   12
  {  "41M", SW_BAND_TYPE, AM,  7200,  7450,  7210, 5},  //       41M   13
  {  "31M", SW_BAND_TYPE, AM,  9400,  9900,  9600, 5},  //       31M   14
  {  "30M", SW_BAND_TYPE, USB, 10100, 10150, 10125, 1}, // Ham   30M   15
  {  "25M", SW_BAND_TYPE, AM, 11600, 12100, 11700, 5},  //       25M   16
  {  "22M", SW_BAND_TYPE, AM, 13570, 13870, 13700, 5},  //       22M   17
  {  "20M", SW_BAND_TYPE, USB, 14000, 14350, 14200, 1}, // Ham   20M   18
  {  "19M", SW_BAND_TYPE, AM, 15100, 15830, 15700, 5},  //       19M   19
  {  "17M", SW_BAND_TYPE, USB, 18068, 18168, 18100, 1}, // Ham   17M   20
  {  "16M", SW_BAND_TYPE, AM, 17480, 17900, 17600, 5},  //       16M   21
  {  "15M", SW_BAND_TYPE, AM, 18900, 19020, 18950, 5},  //       15M   22
  {  "15M", SW_BAND_TYPE, USB, 21000, 21450, 21350, 1}, // Ham   15M   23
  {  "13M", SW_BAND_TYPE, AM, 21450, 21850, 21500, 5},  //       13M   24
  {  "12M", SW_BAND_TYPE, USB, 24890, 24990, 24940, 1}, // Ham   12M   25
  {  "11M", SW_BAND_TYPE, AM, 25670, 26100, 25800, 5},  //       11M   26
  {   "CB", SW_BAND_TYPE, AM, 26200, 27990, 27200, 1},  // CB band     27
  {  "10M", SW_BAND_TYPE, USB, 28000, 30000, 28500, 1}, // Ham   10M   28
  {   "SW", SW_BAND_TYPE, AM,  1730, 30000, 15500, 5}   // Whole SW    29
};
//======================================================= End THE Band Definitions     ========================

//======================================================= FM Presets     ======================================
typedef struct // Preset data
{
  float      presetIdx;
  const char *PresetName;
} FM_Preset ;

FM_Preset preset[] = {

  8930  , "West",      // 00 West
  9890  , "NPO R1",    // 01 NPO R1
  9260  , "NPO R2",    // 02 NPO R2
  9680  , "NPO R3-FM", // 03 NPO R3-FM
  9470  , "NPO R4",    // 04 NPO R4
  9340  , "RIJNMOND",  // 05 RIJNMOND
  9050  , "SUBLIME",   // 06 SUBLIME
  9130  , "BNR",       // 07 BNR
  9520  , "SLAM",      // 08 SLAM
  9620  , "ZFM",       // 09 ZFM
  9760  , "DECIBEL",   // 10 DECIBEL
  10040 , "QMUSIC",    // 11 QMUSIC
  10150 , "SKYRADIO",  // 12 SKYRADIO
  10270 , "RADIO 538", // 13 RADIO 538
  10320 , "VERONICA",  // 14 VERONICA
  10380 , "RADIO 10",  // 15 RADIO 10
  10460 , "100% NL",   // 16 100% NL
  9220  , "L-FM"       // 17 L-FM
};


//======================================================= END FM Presets     ======================================



const int lastButton = (sizeof bt / sizeof(Button)) - 1;
const int lastBand   = (sizeof band / sizeof(Band)) - 1;
const int lastHam = (sizeof bn / sizeof(Bandnumber)) - 1;
const int lastBroad = (sizeof bb / sizeof(BBandnumber)) - 1;
const int lastMod = (sizeof md / sizeof(Mode)) - 1;
const int lastBW = (sizeof bw / sizeof(Bandwidth)) - 1;
const int lastStep = (sizeof sp / sizeof(Step)) - 1;
const int lastKPath = (sizeof kp / sizeof(Keypath)) - 1;
const int lastPreset = (sizeof preset / sizeof (FM_Preset)) - 1;


uint8_t rssi = 0;
uint8_t stereo = 1;
uint8_t volume = DEFAULT_VOLUME;

// Devices class declarations
Rotary encoder = Rotary(ENCODER_PIN_A, ENCODER_PIN_B);

TFT_eSPI tft = TFT_eSPI();

SI4735 si4735;


void FreqDispl()
{
    Serial.print("FirstLayer = ");
    Serial.println(FirstLayer);
    Serial.print("ThirdLayer = ");
    Serial.println(ThirdLayer);
    Serial.print("SEEK = ");
    Serial.println(SEEK);
  if ((FirstLayer) or (ThirdLayer)) {
    Serial.print("FirstLayer = ");
    Serial.println(FirstLayer);
    Serial.print("ThirdLayer = ");
    Serial.println(ThirdLayer);
    Serial.print("SEEK = ");
    Serial.println(SEEK);
   
    currentFrequency = si4735.getFrequency();  
    tft.setTextSize(4);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.setTextDatum(BC_DATUM);
    tft.setTextPadding(0);

    tft.fillRect( XFreqDispl + 6, YFreqDispl + 28 , 228, 32, TFT_BLACK); // Black freq. field
    if (VOLbut) {
      tft.setTextSize(4);
      tft.drawString(String(map(currentVOL, 20, 63, 0, 100)) + " %", XFreqDispl + 120, YFreqDispl + 61);
    } else {
      if (band[bandIdx].bandType == MW_BAND_TYPE || band[bandIdx].bandType == LW_BAND_TYPE) {

        if (bfoOn) {
          tft.setTextSize(4);
          tft.drawString(String(currentBFO), XFreqDispl + 120, YFreqDispl + 61);
        } else {
          Displayfreq =  currentFrequency;
          tft.setTextSize(4);
          tft.drawString(String(Displayfreq, 0), XFreqDispl + 120, YFreqDispl + 61);
          tft.setTextSize(2);
          tft.drawString("KHz", XFreqDispl + 215, YFreqDispl + 61);
        }
      }
      if (band[bandIdx].bandType == FM_BAND_TYPE) {
        Displayfreq =  currentFrequency / 100;

        tft.setTextSize(4);

        tft.drawString(String(Displayfreq, 1), XFreqDispl + 120, YFreqDispl + 61);

        tft.setTextSize(2);
        tft.drawString("MHz", XFreqDispl + 215, YFreqDispl + 61);
      }
      if (band[bandIdx].bandType == SW_BAND_TYPE) {
        if (bfoOn) {
          tft.setTextSize(4);
          tft.drawString(String(currentBFO), XFreqDispl + 120, YFreqDispl + 61);
        }
        else {
          Displayfreq =  currentFrequency / 1000;
          tft.setTextSize(4);
          tft.drawString(String(Displayfreq, 3), XFreqDispl + 120, YFreqDispl + 61);
          tft.setTextSize(2);
          tft.drawString("MHz", XFreqDispl + 215, YFreqDispl + 61);
        }
      }
    }
  }
}

void SeekFreq (uint16_t freq)  {
  if ((FirstLayer)or(ThirdLayer))  {
    currentFrequency = freq;
    tft.setTextSize(4);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.setTextDatum(BC_DATUM);
    tft.setTextPadding(0);
    tft.fillRect( XFreqDispl + 6, YFreqDispl +28 , 228, 32, TFT_BLACK);// Black freq. field
    if (band[bandIdx].bandType == MW_BAND_TYPE || band[bandIdx].bandType == LW_BAND_TYPE) {
        Displayfreq =  currentFrequency;
        tft.setTextSize(4);
        tft.drawString(String(Displayfreq,0), XFreqDispl +120,YFreqDispl +61);
        tft.setTextSize(2);
        tft.drawString("KHz", XFreqDispl +215,YFreqDispl +61);
      }
    if (band[bandIdx].bandType == FM_BAND_TYPE){
      Displayfreq =  currentFrequency/100;
      tft.setTextSize(4);
      tft.drawString(String(Displayfreq,1), XFreqDispl +120,YFreqDispl +61);
      tft.setTextSize(2);
      tft.drawString("MHz", XFreqDispl +215,YFreqDispl +61);
    } 
    if (band[bandIdx].bandType == SW_BAND_TYPE){
        Displayfreq =  currentFrequency/1000;
        tft.setTextSize(4);
        tft.drawString(String(Displayfreq,3), XFreqDispl +120,YFreqDispl +61);
        tft.setTextSize(2);
        tft.drawString("MHz", XFreqDispl +215,YFreqDispl +61);
      }
     }    
   }





void DrawDispl()
{
  tft.fillRect(XFreqDispl, YFreqDispl, 240, 90, TFT_WHITE);
  tft.fillRect(XFreqDispl + 5, YFreqDispl + 5, 230, 80, TFT_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setTextDatum(BC_DATUM);
  tft.drawString(band[bandIdx].bandName, XFreqDispl + 160, YFreqDispl + 20);
  Serial.println("Freqdispl");
  FreqDispl();
  if (band[bandIdx].bandType != FM_BAND_TYPE) {
    tft.setTextSize(1);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setTextDatum(BC_DATUM);
    tft.drawString("BFO", XFreqDispl + 20, YFreqDispl + 16);
    tft.drawString("AGC", XFreqDispl + 50, YFreqDispl + 16);
    tft.setTextPadding(tft.textWidth(" -88"));

    if (bfoOn) {
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
      tft.drawString(String(currentBFOStep), XFreqDispl + 20, YFreqDispl + 26);
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
    } else {
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
      tft.drawString("   ", XFreqDispl + 20, YFreqDispl + 26);
    }

    si4735.getAutomaticGainControl();
    if (si4735.isAgcEnabled()) {
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.drawString("On", XFreqDispl + 50, YFreqDispl + 26);
      tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    } else {
      tft.drawString("Off", XFreqDispl + 50, YFreqDispl + 26);
    }
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.drawString(bandModeDesc[currentMode], XFreqDispl + 80, YFreqDispl + 20);
    tft.setTextPadding(tft.textWidth("2.2kHz"));
    if (currentMode == AM) BWtext = bandwidthAM[bwIdxAM];
    else BWtext = bandwidthSSB[bwIdxSSB];
    tft.drawString(BWtext + "KHz", XFreqDispl + 120, YFreqDispl + 20);
    tft.drawString(String(band[bandIdx].currentStep) + "KHz", XFreqDispl + 200, YFreqDispl + 20);
  }
}

//=======================================================================================
//=======================================================================================


void setup() {
  Serial.begin(115200);
  Serial.println("SI4735 Radio");
  pinMode(Display_Led, OUTPUT);
  pinMode(BEEPER, OUTPUT);
  digitalWrite(Display_Led, displayon);
  DISplay = true;
  Beep(1, 200);

  tft.init();
  tft.setRotation(2);
  //tft.setRotation(0); // Rotate 0
  //tft.setRotation(1); // Rotate 90
  //tft.setRotation(2); // Rotate 180
  //tft.setRotation(3); // Rotate 270

  tft.fillScreen(TFT_BLACK);
  tft.setCursor(7, 100);
  tft.setTextSize(2);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.print("SI4735 Radio");
  tft.setCursor(7, 120);
  tft.print("ESP32");
  delay(1000);
  tft.setCursor(7, 130);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);

  // Calibration code for touchscreen : for 2.8 inch & Rotation = 2
  uint16_t calData[5] = { 258, 3566, 413, 3512, 2 };
  tft.setTouch(calData);

  // Calibration code for touchscreen : Rotation = 0
  //uint16_t calData[5] = { 310, 3518, 354, 3493, 4 };
  //tft.setTouch(calData);

  Wire.begin(ESP32_I2C_SDA, ESP32_I2C_SCL); //I2C for SI4735

  // Encoder pins
  pinMode(ENCODER_PIN_A , INPUT_PULLUP); //Rotary encoder Freqency/bfo/preset
  pinMode(ENCODER_PIN_B , INPUT_PULLUP);

  // Encoder interrupt
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), RotaryEncFreq, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), RotaryEncFreq, CHANGE);

  si4735.setAudioMuteMcuPin(AUDIO_MUTE);

  int16_t si4735Addr = si4735.getDeviceI2CAddress(RESET_PIN);

  tft.fillScreen(TFT_BLACK);
  delay(500);
  tft.setCursor(7, 100);
  tft.setTextSize(2);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);

  if ( si4735Addr == 0 ) {
    tft.print("Si4735 not detected");
    Serial.println("Si4735 not detected");
    while (1);
  } else {
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.print("Si473X addr :  ");
    tft.println(si4735Addr, HEX);
  }
  delay(500);

  if (si4735Addr == 17)
  {
    si4735.setDeviceI2CAddress(0);
  }
  else
  {
    si4735.setDeviceI2CAddress(1);
  }

  si4735.setup(RESET_PIN, 0); // Start in FM

  // Set up the radio for the start band (see index table variable bandIdx )
  si4735.setVolume(volume);
  bandIdx = 0; // FM
  currentMode = FM;
  BandSet();
  currentFrequency = previousFrequency = si4735.getFrequency();
  Beep(2, 200);
  encBut = 600;
  x = y = 0;
  DrawFila();
  si4735.setSeekFmSpacing(10);         // all in setup
  si4735.setSeekFmLimits(6500,10800);
  si4735.setSeekAmRssiThreshold(50);
  si4735.setSeekAmSrnThreshold(20);
  si4735.setSeekFmRssiThreshold(5);
  si4735.setSeekFmSrnThreshold(5);
}// end setup
//=========================================================================================================
//*********************************************************************************************************

void BandSet()
{
  if (bandIdx == 0) currentMode = 0;// only mod FM in FM band
  if ((currentMode == AM) or (currentMode == FM)) ssbLoaded = false;

  if ((currentMode == LSB) or  (currentMode == USB))
  {
    if (ssbLoaded == false) {
      loadSSB();
    }
  }
  useBand();
  setBandWidth();
}


void useBand()
{

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  if (band[bandIdx].bandType == FM_BAND_TYPE)
  {
    bfoOn = false;
    si4735.setTuneFrequencyAntennaCapacitor(0);
    delay(100);
    si4735.setFM(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq, band[bandIdx].currentFreq, band[bandIdx].currentStep);
    ssbLoaded = false;
    si4735.RdsInit();
    si4735.setRdsConfig(1, 2, 2, 2, 2);
  }
  else
  {
    if (band[bandIdx].bandType == MW_BAND_TYPE || band[bandIdx].bandType == LW_BAND_TYPE) {
      si4735.setTuneFrequencyAntennaCapacitor(0);
    } else { //SW_BAND_TYPE
      si4735.setTuneFrequencyAntennaCapacitor(1);
    }
    if (ssbLoaded)
    {
      si4735.setSSB(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq, band[bandIdx].currentFreq, band[bandIdx].currentStep, currentMode);
      si4735.setSSBAutomaticVolumeControl(1);
      si4735.setSsbSoftMuteMaxAttenuation(0); // Disable Soft Mute for SSB
      //si4735.setSBBSidebandCutoffFilter(0);
    }
    else
    {
      si4735.setAM(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq, band[bandIdx].currentFreq, band[bandIdx].currentStep);
      //si4735.setAutomaticGainControl(1, 0);
      si4735.setAmSoftMuteMaxAttenuation(0); // // Disable Soft Mute for AM
      bfoOn = false;
    }

  }
  delay(100);
}// end useband

void setBandWidth()
{
  if (currentMode == LSB || currentMode == USB)
  {
    si4735.setSSBAudioBandwidth(bwIdxSSB);
    // If audio bandwidth selected is about 2 kHz or below, it is recommended to set Sideband Cutoff Filter to 0.
    if (bwIdxSSB == 0 || bwIdxSSB == 4 || bwIdxSSB == 5)
      si4735.setSBBSidebandCutoffFilter(0);
    else
      si4735.setSBBSidebandCutoffFilter(1);
  }
  else if (currentMode == AM)
  {
    si4735.setBandwidth(bwIdxAM, 0);
  }
}


void loadSSB()
{
  si4735.reset();
  si4735.queryLibraryId(); // Is it really necessary here? I will check it.
  si4735.patchPowerUp();
  delay(50);
  si4735.setI2CFastMode(); // Recommended
  //si4735.setI2CFastModeCustom(500000); // It is a test and may crash.
  si4735.downloadPatch(ssb_patch_content, size_content);
  si4735.setI2CStandardMode(); // goes back to default (100KHz)

  // delay(50);
  // Parameters
  // AUDIOBW - SSB Audio bandwidth; 0 = 1.2KHz (default); 1=2.2KHz; 2=3KHz; 3=4KHz; 4=500Hz; 5=1KHz;
  // SBCUTFLT SSB - side band cutoff filter for band passand low pass filter ( 0 or 1)
  // AVC_DIVIDER  - set 0 for SSB mode; set 3 for SYNC mode.
  // AVCEN - SSB Automatic Volume Control (AVC) enable; 0=disable; 1=enable (default).
  // SMUTESEL - SSB Soft-mute Based on RSSI or SNR (0 or 1).
  // DSP_AFCDIS - DSP AFC Disable or enable; 0=SYNC MODE, AFC enable; 1=SSB MODE, AFC disable.
  si4735.setSSBConfig(bwIdxSSB, 1, 0, 0, 0, 1);
  delay(25);
  ssbLoaded = true;
}

void Freqcalq(int keyval)
{
  if (Decipoint) {
    dpfrq = dpfrq + keyval / fact;
  }
  else Displayfreq = (Displayfreq + keyval) * 10;
  fact = fact * 10;
  tft.setTextSize(2);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setCursor(50, 0);
  if (Decipoint) {
    tft.print((Displayfreq / 10) + dpfrq, 4);
    tft.print(" Mhz");
  }
  else {
    tft.print((Displayfreq / 10) + dpfrq, 0);
    if (((Displayfreq / 10) + dpfrq) <= 30000) tft.print(" KHz");
    else tft.print(" Hz");
  }
}

void Smeter() {
  tft.fillRect(Xsmtr + 15, Ysmtr + 38 , (2 + rssi), 6, TFT_RED);
  tft.fillRect(Xsmtr + 17 + rssi, Ysmtr + 38 , 212 - (2 + rssi), 6, TFT_GREEN);
}

void VolumeIndicator(int vol) {

  vol = map(vol, 20, 63, 0, 212);
  tft.fillRect(XVolInd + 15, YVolInd + 16 , (2 + vol), 6, TFT_RED);
  tft.fillRect(XVolInd + 17 + vol, YVolInd + 16 , 212 - (2 + vol), 6, TFT_GREEN);
}

//============================================================================================
void loop() {  //*****************************************************************************
  //******************************************************************************************


  unsigned long now = millis();

  if ((FirstLayer == true) or (ThirdLayer == true)) VolumeIndicator(si4735.getVolume());

  // Pressed will be set true is there is a valid touch on the screen
  while ((pressed == false) and (encoderCount == 0) and (encBut > 500) ) {  // wait loop
    pressed = tft.getTouch(&x, &y);
    encBut = analogRead(ENCODER_SWITCH);
    showtimeRSSI();
    DisplayRDS();
    Dispoff();
  }

  encoderCheck();        // Check if the encoder has moved.
  encoderButtonCheck();  // Check if encoderbutton is pressed

  if (pressed) {
    pressed = false;
    PRESbut = false; // Preset stopped after other button is pressed

    DisplayOnTime = millis();
    if (DISplay == false) {
      Beep(1, 0);
      delay(400);
      digitalWrite(Display_Led, displayon);
      DISplay = true;
      x = y = 0; // no valid touch only for switch on display led
    }
    if (FirstLayer) { //==================================================
      //Check which button is pressed in First Layer.
      for (int n = 0 ; n <= lastButton; n++) {
        if ((x > (Xbutst + (bt[bt[n].ButtonNum].XButos))) and (x < Xbutst + (bt[bt[n].ButtonNum].XButos) + Xbutsiz) and (y > Ybutst + (bt[bt[n].ButtonNum].YButos)) and (y < Ybutst + (bt[bt[n].ButtonNum].YButos) + Ybutsiz)) {
          Beep(1, 0);
          delay(400);
          x = 0;
          y = 0;

          if ((VOLbut == true) and (n != VOL)) {
            VOLbut = false;
            drawVOL();
            DrawDispl ();
          }

          if ((Mutestat == true) and (n != MUTE)) {
            Mutestat = false;
            drawMUTE();
          }


          if (n == HAM) {
            delay(400);  //HamBand button
            HamBand = true;
            HamBandlist();
            FirstLayer = false;
            SecondLayer = true;
          }

          if (n == BFO) {           //==============================  // BFO button
            delay(400);

            if (currentMode == LSB || currentMode == USB)  {
              if (bfoOn == false) {
                bfoOn = true;
              } else {
                bfoOn = false;
              }
              drawBFO();
              DrawDispl ();
            } else Beep(4, 100);
          }

          if (n == FREQ) {          //============================  // Frequency input
            delay(400);
            FREQbut = true;
            Decipoint = false;
            Displayfreq = 0;
            dpfrq = 0;
            drawKeyPath();
            FirstLayer = false;
            SecondLayer = true;
          }

          if (n == AGC) {           //============================//AGC switch
            si4735.getAutomaticGainControl();
            if  (si4735.isAgcEnabled()) {
              si4735.setAutomaticGainControl(1, 0);     //    disabled
            } else {
              si4735.setAutomaticGainControl(0, 0);      //   enabled
            }
            drawAGC();
            DrawDispl ();
          }

          if (n == MODE) {    //============================= MODE
            if (currentMode != FM)  {
              delay(400);// Mode
              Modebut = true;
              Modelist();
              FirstLayer = false;
              SecondLayer = true;
            } else Beep(4, 100);
          }


          if (n == BANDW) {        //=========================BANDWIDTH
            delay(400);
            if (currentMode != FM)  {
              BandWidth = true;
              BWList();
              FirstLayer = false;
              SecondLayer = true;
            } else Beep(4, 100);
          }


          if (n == STEP) {            //========================= STEPS for tune and bfo
            delay(400);
            if (currentMode != FM)  {
              if (bfoOn) setStep();
              else {
                STEPbut = true;
                Steplist();
                FirstLayer = false;
                SecondLayer = true;
              }
            } else Beep(4, 100);
          }

          if (n == BROAD)  {
            delay(400);
            BroadBand = true;
            BroadBandlist();
            FirstLayer = false;
            SecondLayer = true;
          }
          if (n == PRESET) {
            delay(400);
            x = 0;
            y = 0;
            PRESbut = true;
            //tft.fillScreen(TFT_BLACK);
            FirstLayer = false;
            SecondLayer = true;
          }

          if (n == VOL) {
            delay(400);
            x = 0;
            y = 0;
            if (VOLbut == false) {
              VOLbut = true;
              currentVOL = si4735.getVolume();
              previousVOL = currentVOL;
              FirstLayer = false;
              SecondLayer = true;
            }
            else {
              VOLbut = false;
              FirstLayer = true;
              SecondLayer = false;
            }
            FreqDispl();
            drawVOL();
          }





          if (n == MUTE) {
            delay(200);
            x = 0;
            y = 0;
            if (Mutestat == false)  {
              Mutestat = true;
            }
            else  {
              Mutestat = false;
            }
            drawMUTE();
          }

          if (n == NEXT) {
            delay(200);
            x = 0;
            y = 0;
            FirstLayer  = false;
            SecondLayer = false;
            ThirdLayer  = true;
            ForthLayer  = false;
            DrawThla();
          }
        }
      }
    } // end FirstLayer

    if (SecondLayer) {  //===============================================================
      if (Modebut == true) {
        for (int n = 1 ; n <= lastMod; n++) {
          if ((x > md[n].Xmodos) and (x < ((md[n].Xmodos) + (md[n].Xmodsr))) and (y > ((md[n].Ymodos) + (md[n].Ymodnr))) and (y < ((md[n].Ymodos) + (md[n].Ymodsr) + (md[n].Ymodnr)))) {
            Beep(1, 0);
            delay(400);
            Modebut = false;
            currentMode = n;
            BandSet();
            DrawFila();
          }
        }
      }

      if (BandWidth == true) {
        if ( currentMode == AM) {
          for (int n = 0 ; n < 7; n++) {
            if ((x > bw[n].Xos) and (x < ((bw[n].Xos) + (bw[n].Xsr))) and (y > ((bw[n].Yos) + (bw[n].Ynr))) and (y < ((bw[n].Yos) + (bw[n].Ysr) + (bw[n].Ynr)))) {
              Beep(1, 0);
              delay(400);
              bwIdxAM = bw[n].BandWidthAM;
              BandWidth = false;
              BandSet();
              //setBandWidth();
              DrawFila();

            }
          }
        }
        else {
          for (int n = 0 ; n < 6; n++) {
            if ((x > bw[n].Xos) and (x < ((bw[n].Xos) + (bw[n].Xsr))) and (y > ((bw[n].Yos) + (bw[n].Ynr))) and (y < ((bw[n].Yos) + (bw[n].Ysr) + (bw[n].Ynr)))) {
              Beep(1, 0);
              delay(400);
              bwIdxSSB = bw[n].BandWidthSSB;
              BandWidth = false;
              BandSet();
              DrawFila();
            }
          }
        }
      }

      if (STEPbut == true) {
        for (int n = 0 ; n < 4; n++) {
          if ((x > sp[n].Xstepos) and (x < ((sp[n].Xstepos) + (sp[n].Xstepsr))) and (y > ((sp[n].Ystepos) + (sp[n].Ystepnr))) and (y < ((sp[n].Ystepos) + (sp[n].Ystepsr) + (sp[n].Ystepnr)))) {
            Beep(1, 0);
            delay(400);
            STEPbut = false;
            currentStep = sp[n].stepFreq;
            setStep();
            DrawFila();
          }
        }
      }

      if (BroadBand == true) {
        for (int n = 0 ; n <= lastBroad; n++) {
          if ((x > ((bb[n].Xbbandos) + (bb[n].Xbbandnr))) and (x < ((bb[n].Xbbandos) + (bb[n].Xbbandsr) + (bb[n].Xbbandnr))) and (y > ((bb[n].Ybbandos) + (bb[n].Ybbandnr))) and (y < ((bb[n].Ybbandos) + (bb[n].Ybbandsr) + (bb[n].Ybbandnr)))) {
            Beep(1, 0);
            delay(400);
            BroadBand = false;
            bandIdx = bb[n].BbandNum;
            si4735.setAM();
            delay(50);
            currentMode = band[bandIdx].prefmod;
            bwIdxAM =  3;
            BandSet();
            DrawFila(); //Draw first layer
          }
        }
      }

      if (HamBand == true) {
        for (int n = 0 ; n <= lastHam; n++) {
          if ((x > ((bn[n].Xbandos) + (bn[n].Xbandnr))) and (x < ((bn[n].Xbandos) + (bn[n].Xbandsr) + (bn[n].Xbandnr))) and (y > ((bn[n].Ybandos) + (bn[n].Ybandnr))) and (y < ((bn[n].Ybandos) + (bn[n].Ybandsr) + (bn[n].Ybandnr)))) {
            Beep(1, 0);
            delay(400);
            HamBand = false;
            bandIdx = bn[n].BandNum;
            if (ssbLoaded == false) {
              si4735.setAM();
              delay(50);
            }
            currentMode = band[bandIdx].prefmod;
            bwIdxSSB = 1;
            BandSet();
            DrawFila();
          }
        }
      }


      if (PRESbut == true) {
        delay(200);
        if (currentMode != 0) { // geen fm ?
          bandIdx = 0;
          currentMode = 0;
          bfoOn = false;
          drawBFO();
          previousPRES = -1;
        }
        FirstLayer  =  true;
        SecondLayer = false;
        previousPRES = -2;
      }

      if (VOLbut == true) {
        delay(200);
        currentVOL = si4735.getVolume();
        previousVOL = currentVOL;
        FirstLayer  =  true;
        SecondLayer = false;
        FreqDispl();
      }

      if (FREQbut == true) {
        for (int n = 0 ; n < 12; n++) { // which keys are pressed?
          if ((x > ((kp[n].Xkeypos) + (kp[n].Xkeypnr))) and (x < ((kp[n].Xkeypos) + (kp[n].Xkeypsr) + (kp[n].Xkeypnr))) and (y > ((kp[n].Ykeypos) + (kp[n].Ykeypnr))) and (y < ((kp[n].Ykeypos) + (kp[n].Ykeypsr) + (kp[n].Ykeypnr)))) {
            tft.fillRect((kp[n].Xkeypos + kp[n].Xkeypnr + 3) , (kp[n].Ykeypos + kp[n].Ykeypnr + 3), (kp[n].Xkeypsr - 6) , (kp[n].Ykeypsr - 6), TFT_WHITE);
            tft.setTextColor(TFT_RED, TFT_WHITE ); // Temporary white button when touched
            tft.setTextSize(2);
            tft.setTextDatum(BC_DATUM);
            tft.setTextPadding(0);
            tft.drawString((Keypathtext[kp[n].KeypNum]), ( kp[n].Xkeypos + kp[n].Xkeypnr + 30), (kp[n].Ykeypos + kp[n].Ykeypnr  + 37));
            Beep(1, 0);
            delay(100);
            tft.fillRect((kp[n].Xkeypos + kp[n].Xkeypnr + 3) , (kp[n].Ykeypos + kp[n].Ykeypnr + 3), (kp[n].Xkeypsr - 6) , (kp[n].Ykeypsr - 6), TFT_BLUE);
            tft.setTextColor(TFT_YELLOW, TFT_BLUE );
            tft.setTextSize(2);
            tft.setTextDatum(BC_DATUM);
            tft.setTextPadding(0);
            tft.drawString((Keypathtext[kp[n].KeypNum]), ( kp[n].Xkeypos + kp[n].Xkeypnr + 30), (kp[n].Ykeypos + kp[n].Ykeypnr  + 37));
            if ((n >= 0) and (n <= 8)) Freqcalq(n + 1);
            if (n == 10) Freqcalq(0);
            if (n == 9) {
              Decipoint = true;
              fact = 10;
            }
            if (n == 11) {// Send button
              FREQbut = false;
              Displayfreq = (Displayfreq / 10) + dpfrq;
              if ((Displayfreq <= 30000) and (Displayfreq >= 100) and (Decipoint == false ))   Displayfreq = Displayfreq / 1000;
              if (Displayfreq > 30000) Displayfreq = Displayfreq / 1000000;
              if (Displayfreq <= 0.1) { // Freq niet kleiner dan 100KHz
                Displayfreq = 0;
                tft.fillRect(7, 55 , 233, 20, TFT_YELLOW);
                tft.setCursor(7, 25);
                tft.print("Freqency < 100KHz");
                Beep(5, 100);
                delay(3000);
              }

              if ((Displayfreq > 30) and (Displayfreq < 87.5)) {
                Displayfreq = 0;
                tft.fillRect(7, 55 , 233, 20, TFT_BLACK);
                tft.setCursor(7, 25);
                tft.print("Freqency > 30Mhz and < 87.5 MHz");
                Beep(4, 100);
                delay(3000);
              }



              if ((Displayfreq >= 87.5) and (Displayfreq <= 108)) {
                currentFrequency = Displayfreq * 100;
                bandIdx = 0;
                band[bandIdx].currentFreq = currentFrequency;

              } else {
                currentFrequency = Displayfreq * 1000;
                for (int q = 1 ; q <= lastBand; q++) {
                  if (((currentFrequency) >= band[q].minimumFreq) and ((currentFrequency) <= band[q].maximumFreq)) {
                    bandIdx = q;
                    currentMode = band[q].prefmod;
                    currentStep = band[bandIdx].currentStep;
                    break;
                  }
                }
                delay(100);
                band[bandIdx].currentFreq = currentFrequency;
              }
              BandSet();
              DrawFila();
            }//   End   n=11 Send button
          }
        }
      }//end freq
    }// end second layer

    if (ThirdLayer) { //==================================================
      //Check which button is pressed in Third Layer.
      for (int n = 0 ; n <= lastButton; n++) {
        if ((x > (Xbutst + (bt[bt[n].ButtonNum1].XButos))) and (x < Xbutst + (bt[bt[n].ButtonNum1].XButos) + Xbutsiz) and (y > Ybutst + (bt[bt[n].ButtonNum1].YButos)) and (y < Ybutst + (bt[bt[n].ButtonNum1].YButos) + Ybutsiz)) {
          Beep(1, 0);
          delay(400);
          x = 0;
          y = 0;

          if (n == SEEKUP) {
          delay(200);
          x = 0;
          y = 0;
          SEEK = true;
          //si4735.setSeekFmSpacing(10);         // all in setup
          //si4735.setSeekFmLimits(6500,10800);
          //si4735.setSeekAmRssiThreshold(50);
          //si4735.setSeekFmRssiThreshold(5);
          //si4735.setSeekFmSrnThreshold(5);
            if ((currentMode != LSB) and (currentMode != USB))   {
              if (currentMode != FM) {     // No FM
                if (band[bandIdx].bandType == MW_BAND_TYPE || band[bandIdx].bandType == LW_BAND_TYPE) {
                  si4735.setSeekAmSpacing(band[bandIdx].currentStep);     //9 KHz
                  si4735.setSeekAmLimits(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq);
                } 
                else {
                  bandIdx = 29;// all sw
                  si4735.setSeekAmSpacing(band[bandIdx].currentStep);     // 5 KHz
                  si4735.setSeekAmLimits(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq);  
                }
              }
              si4735.seekStationProgress(SeekFreq,1);// 1 is up
              delay(300);
              currentFrequency = si4735.getFrequency();
              if (currentFrequency != previousFrequency)
              {
                previousFrequency = currentFrequency;
                DrawDispl();
                delay(300); 
              }
            }  
            SEEK = false;
          }

         if (n == SEEKDN) {
             delay(200);
          x = 0;
          y = 0;
          SEEK = true;
          //si4735.setSeekFmSpacing(10);         // all in setup
          //si4735.setSeekFmLimits(6500,10800);
          //si4735.setSeekAmRssiThreshold(50);
          //si4735.setSeekFmRssiThreshold(50);
          //si4735.setSeekAmSrnThreshold(20);
          //si4735.setSeekFmSrnThreshold(20);
          if ((currentMode != LSB) and (currentMode != USB))   {
          if (currentMode != FM) {     // No FM
            if (band[bandIdx].bandType == MW_BAND_TYPE || band[bandIdx].bandType == LW_BAND_TYPE) {
              si4735.setSeekAmSpacing(band[bandIdx].currentStep);     //9 KHz
              si4735.setSeekAmLimits(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq);
            } else {
              bandIdx = 29;// all sw
              si4735.setSeekAmSpacing(band[bandIdx].currentStep);     // 5 KHz
              si4735.setSeekAmLimits(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq);  
            }
          }
        
          si4735.seekStationProgress(SeekFreq,0);
          delay(300);
          currentFrequency = si4735.getFrequency();
          if (currentFrequency != previousFrequency)
          {
            previousFrequency = currentFrequency;
            DrawDispl();
            delay(300); 
          }
          }
          SEEK = false;
        }



          if (n == STATUS) {
            delay(200);
            x = 0;
            y = 0;
            subrstatus();
            DrawThla();
          }

          if (n == PREV) {
            delay(200);
            x = 0;
            y = 0;
            FirstLayer  = true;
            SecondLayer = false;
            ThirdLayer  = false;
            ForthLayer  = false;
            DrawFila();
          }
        }
      }
    } // end ThirdLayer

    if (ForthLayer) { //===============================================================

    }
  }// end pressed

  if (currentMode == LSB || currentMode == USB) // set BFO value in si4735
  {
    if (currentBFO != previousBFO)
    {
      previousBFO = currentBFO;
      si4735.setSSBBfo(currentBFO);
      FreqDispl();
    }
  }

  if (currentPRES != previousPRES)
  {
    si4735.getCurrentReceivedSignalQuality();
    if (si4735.isCurrentTuneFM() == false) {
      bandIdx = 0;
      band[bandIdx].currentFreq = ((preset[currentPRES].presetIdx));
      BandSet();
    }
    tft.setCursor(0, 20);
    if (currentPRES > lastPreset) currentPRES = 0;
    if (currentPRES < 0) currentPRES = lastPreset;
    previousPRES = currentPRES;
    DrawDispl();
    tft.fillRect(6, 28 , 228, 32, TFT_BLACK);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK );
    tft.setTextSize(2);
    tft.setTextDatum(BC_DATUM);

    tft.drawString(String(currentPRES) + ") " + String(((preset[currentPRES].presetIdx) / 100), 1), 60, 51);
    tft.setTextColor(TFT_WHITE, TFT_BLACK );
    tft.drawString(String(preset[currentPRES].PresetName), 175, 51);
    bandIdx = 0;
    si4735.setFrequency((preset[currentPRES].presetIdx));
  }

  if (currentVOL != previousVOL)
  {
    currentVOL = currentVOL + (currentVOL - previousVOL);
    tft.setCursor(0, 20);
    if (currentVOL > MaxVOL) currentVOL = MaxVOL;
    if (currentVOL < MinVOL) currentVOL = MinVOL;

    previousVOL = currentVOL;
    //DrawDispl();
    tft.fillRect(6, 28 , 228, 32, TFT_BLACK);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK );
    tft.setTextSize(2);
    tft.setTextDatum(BC_DATUM);

    si4735.setVolume(currentVOL);
    FreqDispl();
  }


}// end loop
//===========================================================================================================
//===========================================================================================================
//===========================================================================================================
void Dispoff()  {
  if (((millis() - DisplayOnTime) > MIN_ELAPSED_DISPL_TIME * 300) and (DISplay == true)) {
    DISplay = false;
    digitalWrite(Display_Led, displayoff);
    PRESbut = false;
    //VOLbut = false;
    DrawDispl();
    DisplayOnTime = millis();
  }
}

void VOLbutoff()  {
  if (((millis() - VOLbutOnTime) > MIN_ELAPSED_VOLbut_TIME * 30) and (VOLbut == true)) {
    VOLbut = false;
    drawVOL();
    FreqDispl();
    //VOLbutOnTime = millis();
  }
  if (VOLbut == false) VOLbutOnTime = millis();
}




void DisplayRDS()
{
  if ((millis() - elapsedRDS) > MIN_ELAPSED_RDS_TIME * 10) // 5 * 10  = 50 Ms  refresh time RDS
  {
    if ( currentMode == FM) {
      if ( currentFrequency != previousFrequency ) {
        previousFrequency = currentFrequency;
        tft.fillRect(XFreqDispl + 60, YFreqDispl + 60, 140, 20, TFT_BLACK);
      }

      if (RDS) checkRDS();
    }
    elapsedRDS = millis();
  }
}

void showtimeRSSI() {
  // Show RSSI status only if this condition has changed
  if ((millis() - elapsedRSSI) > MIN_ELAPSED_RSSI_TIME * 10) // 150 * 10  = 1.5 sec refresh time RSSI
  {
    si4735.getCurrentReceivedSignalQuality();
    NewRSSI = si4735.getCurrentRSSI();
    NewSNR = si4735.getCurrentSNR();
    if (OldRSSI != NewRSSI)
    {
      OldRSSI = NewRSSI;
      showRSSI();
    }
    elapsedRSSI = millis();
  }
}

void showRSSI() {
  if ((  currentMode == FM ) and ((FirstLayer) or (ThirdLayer))) {
    sprintf(buffer, "%s", (si4735.getCurrentPilot()) ? "STEREO" : "MONO");
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setTextSize(1);
    tft.setTextDatum(BC_DATUM);
    tft.setTextPadding(0);
    tft.fillRect(XFreqDispl + 30, YFreqDispl + 10 , 50, 12, TFT_BLACK); // STEREO MONO
    tft.drawString(buffer, XFreqDispl + 50, YFreqDispl + 20);
  }
  rssi = map(NewRSSI, 0, 95, 0, 212);
  if ((FirstLayer) or (ThirdLayer)) Smeter();
  tft.setTextSize(1);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  if ((FirstLayer) or (ThirdLayer)) {  // dBuV and volume at freq. display
    tft.fillRect(XFreqDispl + 7, YFreqDispl + 75 , 40, 10, TFT_BLACK);
    tft.fillRect(XFreqDispl + 219, YFreqDispl + 75 , 10, 10, TFT_BLACK);
    tft.setTextDatum(TL_DATUM);
    tft.drawString(String(NewRSSI) + " dBuV" , XFreqDispl + 8, YFreqDispl + 75);
    tft.setTextDatum(TR_DATUM);
    tft.drawString(String(NewSNR) + "dB", XFreqDispl + 230, YFreqDispl + 75);
  }
  VOLbutoff();
}


void encoderCheck()
{
  if (encoderCount != 0)
  {
    if (DISplay == false) { //  Wake-up  Display
      DisplayOnTime = millis();
      digitalWrite(Display_Led, displayon);
      DISplay = true;
    }
    int mainpurp = 1;
    if (bfoOn)  {
      currentBFO = (encoderCount == 1) ? (currentBFO + currentBFOStep) : (currentBFO - currentBFOStep);
      mainpurp = 0;
    }

    if (PRESbut) {     // FM preset
      currentPRES = (encoderCount == 1) ? (currentPRES + currentPRESStep) : (currentPRES - currentPRESStep);
      mainpurp = 0;
    }

    if (VOLbut) {     // Volume control
      currentVOL = (encoderCount == 1) ? (currentVOL + currentVOLStep) : (currentVOL - currentVOLStep);
      mainpurp = 0;
    }

    if (mainpurp == 1)
    {
      if (encoderCount == 1) {
        si4735.frequencyUp();
      } else {
        si4735.frequencyDown();
      }
      FreqDispl();
      band[bandIdx].currentFreq = si4735.getFrequency();
    }
    encoderCount = 0;
  }
}

void encoderButtonCheck()
{
  //Encoder button
  encBut = analogRead(ENCODER_SWITCH);
  if (encBut < 500) {
    Beep(1, 0);
    delay(400);
    if (DISplay == false) { //  Wake-up  Display
      DisplayOnTime = millis();
      digitalWrite(Display_Led, displayon);
      DISplay = true;
    }
    if (PRESbut == true) {// FM preset selection
      PRESbut = false;
      DrawDispl();
    }
    else {
      if (ssbLoaded) {  // SSB is on
        if (bfoOn) {
          bfoOn = false;
        }
        else {
          bfoOn = true;
        }
        //if (currentMode == FM) bfoOn = false;
        drawBFO();
        DrawDispl();
      }
    }
  }
}


void setStep()
{
  // This command should work only for SSB mode
  if (bfoOn && (currentMode == LSB || currentMode == USB))
  {
    currentBFOStep = (currentBFOStep == 25) ? 10 : 25;
    DrawDispl();
  }
  else
  {
    si4735.setFrequencyStep(currentStep);
    band[bandIdx].currentStep = currentStep;
  }
  DrawDispl();
}


void Beep(int cnt, int tlb) {
  int tla = 100;
  for (int i = 0; i < cnt; i++) {
    digitalWrite(BEEPER, beepOn);
    delay(tla);
    digitalWrite(BEEPER, beepOff);
    delay(tlb);
  }
}


void DrawFila()// Draw of first layer
{
  FirstLayer = true;
  SecondLayer = false;
  tft.fillScreen(TFT_BLACK);
  DrawButFila();
  Serial.println("Drawdisplay");
  
  DrawDispl();
  DrawSmeter();
  DrawVolumeIndicator();
}

void DrawThla()// Draw of Third layer
{
  ThirdLayer = true;
  ForthLayer = false;
  tft.fillScreen(TFT_BLACK);
  DrawButThla();
  DrawDispl();
  DrawSmeter();
  DrawVolumeIndicator();
}


void DrawButFila() { // Buttons first layer
  tft.fillScreen(TFT_BLACK);
  for (int n = 0 ; n <= lastButton; n++) {
    tft.fillRect(bt[bt[n].ButtonNum].XButos + Xbutst, bt[bt[n].ButtonNum].YButos + Ybutst , Xbutsiz , Ybutsiz, TFT_WHITE);
    tft.fillRect((bt[bt[n].ButtonNum].XButos + Xbutst + 3) , (bt[bt[n].ButtonNum].YButos + Ybutst + 3), (Xbutsiz - 6) , (Ybutsiz - 6), TFT_BLUE);
    tft.setTextColor(TFT_YELLOW, TFT_BLUE);
    tft.setTextSize(2);
    tft.setTextDatum(BC_DATUM);
    tft.setTextPadding(0);
    tft.drawString((bt[n].ButtonNam), ( bt[bt[n].ButtonNum].XButos + Xbutst + (Xbutsiz / 2) ), (bt[bt[n].ButtonNum].YButos + Ybutst  + ((Ybutsiz) / 2 + 9)  ));
    //tft.drawCentreString((bt[n].ButtonNam),( bt[bt[n].ButtonNum].XButos + Xbutst +(Xbutsiz/2) ),(bt[bt[n].ButtonNum].YButos+Ybutst  + ((Ybutsiz)/2+9)  ),1);
  }
  drawBFO();
  drawAGC();
}

void DrawButThla() { // Buttons Third layer
  tft.fillScreen(TFT_BLACK);
  for (int n = 0 ; n <= lastButton; n++) {
    tft.fillRect(bt[bt[n].ButtonNum1].XButos + Xbutst, bt[bt[n].ButtonNum1].YButos + Ybutst , Xbutsiz , Ybutsiz, TFT_WHITE);
    tft.fillRect((bt[bt[n].ButtonNum1].XButos + Xbutst + 3) , (bt[bt[n].ButtonNum1].YButos + Ybutst + 3), (Xbutsiz - 6) , (Ybutsiz - 6), TFT_BLUE);
    tft.setTextColor(TFT_YELLOW, TFT_BLUE);
    tft.setTextSize(2);
    tft.setTextDatum(BC_DATUM);
    tft.setTextPadding(0);
    tft.drawString((bt[n].ButtonNam1), ( bt[bt[n].ButtonNum1].XButos + Xbutst + (Xbutsiz / 2) ), (bt[bt[n].ButtonNum1].YButos + Ybutst  + ((Ybutsiz) / 2 + 9)  ));
    //tft.drawCentreString((bt[n].ButtonNam),( bt[bt[n].ButtonNum].XButos + Xbutst +(Xbutsiz/2) ),(bt[bt[n].ButtonNum].YButos+Ybutst  + ((Ybutsiz)/2+9)  ),1);
  }
}



void DrawVolumeIndicator()
{
  tft.setTextSize(1);
  tft.fillRect(XVolInd, YVolInd, 240, 30, TFT_WHITE);
  tft.fillRect(XVolInd + 5, YVolInd + 5, 230, 20, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(XVolInd +  11, YVolInd + 7);
  tft.print("0%");
  tft.setCursor(XVolInd + 116, YVolInd + 7);
  tft.print("50%");
  tft.setCursor(XVolInd + 210, YVolInd + 7);
  tft.print("100%");
}


void DrawSmeter()
{
  String IStr;
  tft.setTextSize(1);
  tft.fillRect(Xsmtr, Ysmtr, 240, 55, TFT_WHITE);
  tft.fillRect(Xsmtr + 5, Ysmtr + 5, 230, 45, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(BC_DATUM);
  for (int i = 0; i < 10; i++) {
    tft.fillRect(Xsmtr + 15 + (i * 12), Ysmtr + 24, 4, 8, TFT_YELLOW);
    IStr = String(i);
    tft.setCursor((Xsmtr + 14 + (i * 12)), Ysmtr + 13);
    tft.print(i);
  }
  for (int i = 1; i < 6; i++) {
    tft.fillRect((Xsmtr + 123 + (i * 20)), Ysmtr + 24, 4, 8, TFT_RED);
    IStr = String(i * 10);
    tft.setCursor((Xsmtr + 117 + (i * 20)), Ysmtr + 13);
    tft.print(i * 10);
    tft.print("+");
  }
  tft.fillRect(Xsmtr + 15, Ysmtr + 32 , 112, 4, TFT_YELLOW);
  tft.fillRect(Xsmtr + 127, Ysmtr + 32 , 100, 4, TFT_RED);
  // end Smeter
}

void drawVOL()
{
  int VOLbutcol;
  if (VOLbut) {
    VOLbutcol = TFT_DARKGREEN;
  } else {
    VOLbutcol = TFT_BLUE;
  }
  tft.fillRect(bt[bt[5].ButtonNum].XButos + Xbutst, bt[bt[5].ButtonNum].YButos + Ybutst , Xbutsiz , Ybutsiz, TFT_WHITE);
  tft.fillRect((bt[bt[5].ButtonNum].XButos + Xbutst + 3) , (bt[bt[5].ButtonNum].YButos + Ybutst + 3), (Xbutsiz - 6) , Ybutsiz - 6, VOLbutcol);
  tft.setTextColor(TFT_YELLOW, VOLbutcol);
  tft.setTextSize(2);
  tft.setTextDatum(BC_DATUM);
  tft.setTextPadding(0);
  tft.drawString((bt[5].ButtonNam), ( bt[bt[5].ButtonNum].XButos + Xbutst + (Xbutsiz / 2)), (bt[bt[5].ButtonNum].YButos + Ybutst  + (Ybutsiz / 2 + 9)));
}



void drawMUTE()
{
  int MUTEbutcol;
  if (Mutestat) {
    MUTEbutcol = TFT_DARKGREEN;
    si4735.setAudioMute(audioMuteOn);
  } else {
    MUTEbutcol = TFT_BLUE;
    si4735.setAudioMute(audioMuteOff);
  }
  tft.fillRect(bt[bt[4].ButtonNum].XButos + Xbutst, bt[bt[4].ButtonNum].YButos + Ybutst , Xbutsiz , Ybutsiz, TFT_WHITE);
  tft.fillRect((bt[bt[4].ButtonNum].XButos + Xbutst + 3) , (bt[bt[4].ButtonNum].YButos + Ybutst + 3), (Xbutsiz - 6) , Ybutsiz - 6, MUTEbutcol);
  tft.setTextColor(TFT_YELLOW, MUTEbutcol);
  tft.setTextSize(2);
  tft.setTextDatum(BC_DATUM);
  tft.setTextPadding(0);
  tft.drawString((bt[4].ButtonNam), ( bt[bt[4].ButtonNum].XButos + Xbutst + (Xbutsiz / 2)), (bt[bt[4].ButtonNum].YButos + Ybutst  + (Ybutsiz / 2 + 9)));
}


void drawAGC()
{
  int AGCbutcol;
  si4735.getAutomaticGainControl();
  if (si4735.isAgcEnabled()) {
    AGCbutcol = TFT_RED;
  } else {
    AGCbutcol = TFT_BLUE;
  }
  tft.fillRect(bt[bt[3].ButtonNum].XButos + Xbutst, bt[bt[3].ButtonNum].YButos + Ybutst , Xbutsiz , Ybutsiz, TFT_WHITE);
  tft.fillRect((bt[bt[3].ButtonNum].XButos + Xbutst + 3) , (bt[bt[3].ButtonNum].YButos + Ybutst + 3), (Xbutsiz - 6) , Ybutsiz - 6, AGCbutcol);
  tft.setTextColor(TFT_YELLOW, AGCbutcol);
  tft.setTextSize(2);
  tft.setTextDatum(BC_DATUM);
  tft.setTextPadding(0);
  tft.drawString((bt[3].ButtonNam), ( bt[bt[3].ButtonNum].XButos + Xbutst + (Xbutsiz / 2)), (bt[bt[3].ButtonNum].YButos + Ybutst  + (Ybutsiz / 2 + 9)));
}

void drawBFO ()
{
  int BFObutcol;
  if (bfoOn) BFObutcol = TFT_RED;
  else BFObutcol = TFT_BLUE;
  tft.fillRect(bt[bt[1].ButtonNum].XButos + Xbutst, bt[bt[1].ButtonNum].YButos + Ybutst , Xbutsiz , Ybutsiz, TFT_WHITE);
  tft.fillRect((bt[bt[1].ButtonNum].XButos + Xbutst + 3) , (bt[bt[1].ButtonNum].YButos + Ybutst + 3), Xbutsiz - 6 , (Ybutsiz - 6), BFObutcol);
  tft.setTextColor(TFT_YELLOW, BFObutcol);
  tft.setTextSize(2);
  tft.setTextDatum(BC_DATUM);
  tft.setTextPadding(0);
  tft.drawString((bt[1].ButtonNam), ( bt[bt[1].ButtonNum].XButos + Xbutst + (Xbutsiz / 2)), (bt[bt[1].ButtonNum].YButos + Ybutst  + (Ybutsiz / 2 + 9)));
}

void drawKeyPath() {
  int Sendbutcol;
  tft.fillScreen(TFT_BLACK);
  for (int n = 0 ; n <= lastKPath; n++) {
    tft.fillRect(kp[n].Xkeypos + kp[n].Xkeypnr, kp[n].Ykeypos + kp[n].Ykeypnr , kp[n].Xkeypsr , kp[n].Ykeypsr, TFT_WHITE);
    if ( n == 11) { // Send button is red
      Sendbutcol = TFT_RED;
    } else {
      Sendbutcol = TFT_BLUE;
    }
    tft.fillRect((kp[n].Xkeypos + kp[n].Xkeypnr + 3) , (kp[n].Ykeypos + kp[n].Ykeypnr + 3), (kp[n].Xkeypsr - 6) , (kp[n].Ykeypsr - 6), Sendbutcol);
    tft.setTextColor(TFT_YELLOW, Sendbutcol );
    tft.setTextSize(2);
    tft.setTextDatum(BC_DATUM);
    tft.setTextPadding(0);
    tft.drawString((Keypathtext[kp[n].KeypNum]), ( kp[n].Xkeypos + kp[n].Xkeypnr + 30), (kp[n].Ykeypos + kp[n].Ykeypnr  + 37));
  }
}

void MISClist() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_YELLOW, TFT_BLUE);
  tft.setTextSize(2);
  tft.setTextDatum(BC_DATUM);
  tft.setTextPadding(0);

  for (int n = 0 ; n <= 5; n++) {
    tft.fillRect((bn[n].Xbandos + bn[n].Xbandnr) , (bn[n].Ybandos) + (bn[n].Ybandnr), (bn[n].Xbandsr + 20) , (bn[n].Ybandsr), TFT_WHITE);
    tft.fillRect((bn[n].Xbandos + bn[n].Xbandnr + 3) , (bn[n].Ybandos) + (bn[n].Ybandnr + 3), (bn[n].Xbandsr - 6 + 20) , (bn[n].Ybandsr - 6), TFT_BLUE);
    tft.drawString((misctext[kp[n].KeypNum]), (bn[n].Xbandos + bn[n].Xbandnr) + 65, (bn[n].Ybandos) + (bn[n].Ybandnr) + 25);
  }
}


void HamBandlist() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  tft.setTextSize(2);
  tft.setTextDatum(BC_DATUM);
  tft.setTextPadding(0);

  for (int n = 0 ; n <= lastHam; n++) {
    tft.fillRect((bn[n].Xbandos + bn[n].Xbandnr) , (bn[n].Ybandos) + (bn[n].Ybandnr), (bn[n].Xbandsr) , (bn[n].Ybandsr), TFT_RED);
    tft.fillRect((bn[n].Xbandos + bn[n].Xbandnr + 3) , (bn[n].Ybandos) + (bn[n].Ybandnr + 3), (bn[n].Xbandsr - 6) , (bn[n].Ybandsr - 6), TFT_WHITE);
    tft.drawString(band[bn[n].BandNum].bandName, (bn[n].Xbandos + bn[n].Xbandnr) + 55, (bn[n].Ybandos) + (bn[n].Ybandnr) + 25);
  }
}

void BroadBandlist() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  tft.setTextSize(2);
  tft.setTextDatum(BC_DATUM);
  tft.setTextPadding(0);
  for (int n = 0 ; n <= lastBroad; n++) {
    tft.fillRect((bb[n].Xbbandos + bb[n].Xbbandnr) , (bb[n].Ybbandos) + (bb[n].Ybbandnr), (bb[n].Xbbandsr) , (bb[n].Ybbandsr), TFT_RED);
    tft.fillRect((bb[n].Xbbandos + bb[n].Xbbandnr + 3) , (bb[n].Ybbandos) + (bb[n].Ybbandnr + 3), (bb[n].Xbbandsr - 6) , (bb[n].Ybbandsr - 6), TFT_WHITE);
    tft.drawString(band[bb[n].BbandNum].bandName, (bb[n].Xbbandos + bb[n].Xbbandnr) + 55, (bb[n].Ybbandos) + (bb[n].Ybbandnr) + 25);
  }
}

void Steplist() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  tft.setTextSize(2);
  tft.setTextDatum(BC_DATUM);
  tft.setTextPadding(0);
  for (int n = 0 ; n <= lastStep; n++) {
    tft.fillRect( (sp[n].Xstepos), (sp[n].Ystepos) + (sp[n].Ystepnr), (sp[n].Xstepsr), (sp[n].Ystepsr), TFT_RED);
    tft.fillRect( (sp[n].Xstepos + 5), sp[n].Ystepos + 5 + (sp[n].Ystepnr) , sp[n].Xstepsr - 10 , sp[n].Ystepsr - 10, TFT_WHITE);
    tft.drawString(String(sp[n].stepFreq) + "KHz", (sp[n].Xstepos) + 50, (sp[n].Ystepos) + (sp[n].Ystepnr) + 24);
  }
}

void Modelist() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  tft.setTextSize(2);
  tft.setTextDatum(BC_DATUM);
  tft.setTextPadding(0);
  for (int n = 1 ; n <= lastMod; n++) {
    tft.fillRect( (md[n].Xmodos), (md[n].Ymodos) + (md[n].Ymodnr), (md[n].Xmodsr), (md[n].Ymodsr), TFT_RED);
    tft.fillRect( (md[n].Xmodos + 5), md[n].Ymodos + 5 + (md[n].Ymodnr) , md[n].Xmodsr - 10 , md[n].Ymodsr - 10, TFT_WHITE);
    tft.drawString(bandModeDesc[md[n].Modenum], (md[n].Xmodos) + 50, (md[n].Ymodos) + (md[n].Ymodnr) + 24);
  }
}

void BWList()
{
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  tft.setTextSize(2);
  tft.setTextDatum(BC_DATUM);
  tft.setTextPadding(0);
  if ( currentMode == AM) nrbox = 7;
  else nrbox = 6;
  for (int n = 0 ; n < nrbox; n++) {
    tft.fillRect( (bw[n].Xos), (bw[n].Yos) + (bw[n].Ynr), (bw[n].Xsr), (bw[n].Ysr), TFT_RED);
    tft.fillRect( (bw[n].Xos + 3), bw[n].Yos + 3 + (bw[n].Ynr) , bw[n].Xsr - 6 , bw[n].Ysr - 6, TFT_WHITE);
    if ( currentMode == AM) tft.drawString(bandwidthAM[bw[n].BandWidthAM], (bw[n].Xos) + 50, (bw[n].Yos) + (bw[n].Ynr) + 24);
    else tft.drawString(bandwidthSSB[bw[n].BandWidthSSB], (bw[n].Xos) + 50, (bw[n].Yos) + (bw[n].Ynr) + 24);
  }
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  if ( currentMode == AM)  tft.drawString("AM modulation"  , XfBW + 50, YfBW - 30);
  if ( currentMode == USB) tft.drawString("USB modulation" , XfBW + 50, YfBW - 30);
  if ( currentMode == LSB) tft.drawString("LSB modulation" , XfBW + 50, YfBW - 30);
  tft.drawString("Bandwidth in KHz" , XfBW + 50, YfBW - 10);
}

void RotaryEncFreq()
{ // rotary encoder events
  uint8_t encoderStatus = encoder.process();
  if (encoderStatus)
  {
    if (encoderStatus == DIR_CW)
    {
      encoderCount = 1;
    }
    else
    {
      encoderCount = -1;
    }
  }
}

void subrstatus() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(0, 0);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.drawString("Mod.     : " + String(bandModeDesc[band[bandIdx].prefmod]), 5, 10);
  if ( currentMode != FM)  tft.drawString("Freq.    : " + String(currentFrequency, 0) + " KHz", 5, 35);
  else tft.drawString("Freq.    : " + String(currentFrequency / 100, 1) + " MHz", 5, 35);
  si4735.getCurrentReceivedSignalQuality();
  tft.drawString("RSSI     : " + String(si4735.getCurrentRSSI()) + "dBuV", 5, 60); // si4735.getCurrentSNR()
  tft.drawString("SNR      : " + String(si4735.getCurrentSNR()) + "uV", 5, 85);
  if (  currentMode == FM ) {
    sprintf(buffer, "%s", (si4735.getCurrentPilot()) ? "STEREO" : "MONO");
    tft.drawString(buffer, 160, 97);
  }

  si4735.getAutomaticGainControl();
  si4735.getCurrentReceivedSignalQuality();
  tft.drawString("LNA GAIN index: " + String(si4735.getAgcGainIndex()) + "/" + String(currentAGCAtt), 5, 110);
  tft.drawString("Volume   : )" + String(si4735.getVolume()), 5, 135);
  sprintf(buffer, "%s", (si4735.isAgcEnabled()) ? "AGC ON " : "AGC OFF");
  tft.drawString(buffer, 5, 160);
  if (bfoOn) tft.drawString("BFO ON  ", 180, 160);
  else tft.drawString("BFO OFF ", 180, 160);


  tft.drawString("AVC max GAIN  : " + String(si4735.getCurrentAvcAmMaxGain()), 5, 172);

  tft.drawString("Ant. Cap = " + String(si4735.getAntennaTuningCapacitor()) , 5, 185);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("BandIdx  : " + String(bandIdx) + "  " + String(band[bandIdx].bandName) , 5, 210);
  tft.drawString("BwIdxSSB : " + String(bandwidthSSB[bwIdxSSB]) + " KHz", 5, 235);
  tft.drawString("BwIdxAM  : " + String(bandwidthAM[bwIdxAM]) + " KHz", 5, 260);
  tft.drawString("Stepsize : " + String(currentStep), 5, 285);
  int vsupply = analogRead(ENCODER_SWITCH);
  tft.drawString("Power Supply : " + String(((1.78 / 2140)*vsupply) * 1.842) + " V.", 5, 310);

  while (x == 0) {
    press1 = tft.getTouch(&x, &y);
    int m = 0;
  }
  Beep(1, 0);
  delay(400);
}



void showRDSMsg() {
  rdsMsg[35] = bufferRdsMsg[35] = '\0';
  if (strcmp(bufferRdsMsg, rdsMsg) == 0) return;
  if ((FirstLayer) or (ThirdLayer)) tft.drawString(String(rdsMsg) , XFreqDispl + 12, YFreqDispl + 380);
  delay(250);
}

void showRDSStation() {
  if (strcmp(bufferStatioName, stationName) == 0 ) return;
  if ((FirstLayer) or (ThirdLayer)) {
    //tft.setCursor(XFreqDispl +120, YFreqDispl +80);
    //tft.print(stationName);
    tft.drawString(String(stationName) , XFreqDispl + 120, YFreqDispl + 80);
  }
  delay(250);
}

void showRDSTime() {
  if (strcmp(bufferRdsTime, rdsTime) == 0 ) return;
  if ((FirstLayer) or (ThirdLayer)) tft.drawString(String(rdsTime) , XFreqDispl + 12, YFreqDispl + 400);
  //delay(250);
}

void checkRDS() {

  si4735.getRdsStatus();
  if (si4735.getRdsReceived()) {
    if (si4735.getRdsSync() && si4735.getRdsSyncFound() ) {
      rdsMsg = si4735.getRdsText2A();
      stationName = si4735.getRdsText0A();
      rdsTime = si4735.getRdsTime();
      tft.setTextSize(2);
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
      tft.setTextDatum(BC_DATUM);
      //if ( rdsMsg != NULL )   showRDSMsg();
      if ( stationName != NULL )   showRDSStation();
      //if ( rdsTime != NULL ) showRDSTime();
    }
  }
}
