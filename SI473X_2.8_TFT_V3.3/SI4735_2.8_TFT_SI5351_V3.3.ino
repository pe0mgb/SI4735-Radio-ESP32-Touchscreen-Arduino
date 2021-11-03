//  V3.3    02-11-2021
//  V3.2.6b 29-10-2021  5351 calibration
//  V3.2.4  03-10-2021 100 Hz & 10 Hz added in SSB. Many changes in control interface. Bug support.
//                    SI5351 added, replacing crystal and also used as BFO. All steps and bandwidth per modulation
//                    are now stored in memory.
//  V3.2.3 29-09-2021 Added Sprite buttons from Jim Yasuda.
//  This sketch is based on the si4735 Library of Ricardo PU2CLR. Thanks for the very nice work.

//  This sketch uses  a 2.8 inch 240*320 touch-screen with ILI9341, ESP32 WROOM-32 and Rotary Encoder.
//  The radio is fully controlled by the (Touch)Screen and Rotary Encoder
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
//  Library TFT_eSPI you may download from here : https://github.com/Bodmer/TFT_eSPI
//  Library Rotary is provided with the program
//  Library SI4735 you may download from here   : https://github.com/pu2clr/SI4735
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
//  |   SDO      |             | 09 |            |            |            |        Emmitor  V2 BC557 to 3.3 V
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
//  |            |     27 Mute |    |see schematics           |            |
//  |------------|-------------|----|------------|------------|------------|


#include <TFT_eSPI.h>
#include <SPI.h>
#include <SI4735.h>
#include "EEPROM.h"
#include "Rotary.h"
#include <si5351wire.h>
#include "DSEG7_Classic_Mini_Regular_34.h"
#include "TFT_Colors.h"
#include "Button.h"


// ==================Display========================
//#define IhaveVertTFT
#define IhaveHoriTFT
// =================================================

// ==================Oscillator=====================
//#define IhaveCrystal
#define IhaveSI5351
// =================================================

//#include "patch_init.h"    // SSB patch for whole SSBRX initialization string
#include "patch_full.h"    // SSB patch for whole SSBRX full download

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


#define MIN_ELAPSED_TIME             100
#define MIN_ELAPSED_RSSI_TIME        150
#define MIN_ELAPSED_AudMut_TIME        0  // Noise surpression SSB in mSec. 0 mSec = off
#define MIN_ELAPSED_DISPL_TIME      1000
#define MIN_ELAPSED_RDS_TIME           5
#define DEFAULT_VOLUME                45   // change it for your favorite start sound volume
#define MIN_ELAPSED_VOLbut_TIME     1000
#define CLK_Xtal                    SI5351wire_CLK0

#define FM          0
#define LSB         1
#define USB         2
#define AM          3
#define CW          4
#define TFT_GREY 0x5AEB

bool bfoOn          = false;
bool ssbLoaded      = false;
bool FirstLayer     = true;
bool FirstTime      = true;
bool SecondLayer    = false;
bool ThirdLayer     = false;
bool ForthLayer     = false;
bool HamBand        = false;
bool Modebut        = false;
bool FREQbut        = false;
bool Decipoint      = false;
bool STEPbut        = false;
bool encsw          = false;
bool BroadBand;
bool BandWidth;
bool MISCbut        = false;
bool PRESbut        = false;
bool VOLbut         = false;
bool AudioMut       = false;
bool DISplay        = false;
bool Mutestat       = false;
bool AGCgainbut     = false;
bool writingEeprom  = false;

bool pressed;
bool presStat;
bool audioMuteOn  = true;
bool audioMuteOff = false;
bool RDS          = true; // RDS on  or  off
bool SEEK         = false;
bool bright       = false;
bool CWShift      = false;
bool fstShift     = false;
bool calibratSI5351 = false;

int currentBFO;
int currentBFOmanu;
int previousBFO     = 0;
int previousBFOmanu = 0;
int nrbox           = 0;
int OldRSSI;
int NewRSSI;
int NewSNR;
int encBut;
int AGCgain         = 0;
int PrevRSSI        = 0;
int strongup        = 0;

long elapsedRSSI        = millis();
long elapsedAudMut      = millis();
long elapsedRDS         = millis();
long stationNameElapsed = millis();
long DisplayOnTime      = millis();
long VOLbutOnTime       = millis();

volatile int encoderCount  = 0;
volatile int encoderButton = 0;


uint16_t previousFrequency;
uint8_t currentBFOStep     = 25;
uint8_t currentPRES        =  0;
uint8_t previousPRES       =  0;
uint8_t currentPRESStep    =  1;

uint8_t currentAGCgain     =  1;
uint8_t previousAGCgain    =  1;
uint8_t currentAGCgainStep =  1;
uint8_t MaxAGCgain;
uint8_t MaxAGCgainFM       = 26;
uint8_t MaxAGCgainAM       = 37;
uint8_t MinAGCgain         =  1;


uint8_t currentVOL         =  0;
uint8_t previousVOL        =  0;
uint8_t currentVOLStep     =  1;
uint8_t MaxVOL             = 63;
uint8_t MinVOL             = 20;

uint8_t currentAGCAtt      =  0;
uint8_t bwIdxSSB;
uint8_t bwIdxAM;
uint8_t bwIdxFM;
uint8_t ssIdxMW;
uint8_t ssIdxAM;
uint8_t ssIdxFM;
uint8_t bandIdx;
uint8_t currentMode        = FM;
uint8_t previousMode       =  0;
uint16_t x = 0, y = 0; // To store the touch coordinates
uint8_t encoderStatus;
uint16_t freqstep;
uint8_t freqstepnr         = 0; //1kHz
int freqDec                = 0;

const int LedFreq          = 5000;
const int LedResol         = 8;
const int LedChannelforTFT = 0;
uint16_t currentBrightness;
uint16_t previousBrightness;
uint16_t MaxBrightness     = 20;
uint16_t MinBrightness     = 250;
uint8_t stepsizesynth      = 10;

float Displayfreq          = 0;
float currentFrequency     = 0;
float dpfrq                = 0;
float fact                 = 1;
float RSSIfact             = 3;

String BWtext;
String Modtext;
String RDSbuttext;
String AGCgainbuttext;

//===============================================================================
const char *bandwidthSSB[] = {"1.2", "2.2", "3.0", "4.0", "0.5", "1.0"};
const char *bandwidthAM[]  = {"6.0", "4.0", "3.0", "2.0", "1.0", "1.8", "2.5"};
const char *bandwidthFM[]  = {"AUT","110","84","60","40"};
const char *stepsize[]     = {"1","5","9","10"};
const char *stepsizeFM[]   = {"100","10"};
 
const char *Keypathtext[]  = {"1", "2", "3", "4", "5", "6", "7", "8", "9", ".", "0", "Send", "Clear"};
const char *bandModeDesc[] = {"FM ", "LSB", "USB", "AM ", "CW"};

char buffer[64]; // Useful to handle string
char buffer1[64];

char *stationName;
char bufferStatioName[40];

unsigned long FreqSI5351 = 3276800;
unsigned long calibratvalSI5351;
//=======================================================   Buttons First and Third Layer   ==========================
typedef struct // Buttons first layer
{
  const char *ButtonNam;
  uint8_t     ButtonNum;       // Button location at display from 0 to 11. To move around buttons freely at first layer.
  const char *ButtonNam1;
  uint8_t     ButtonNum1;      // Button location at display from 0 to 11. To move around buttons freely at third layer.
  uint16_t    XButos;          // Xoffset
  long        YButos;          // Yoffset
} Button;
int ytotoffset = 0;

//  Button table
int Xbutst  =   0;               // X Start location Buttons
int Ybutst  = 160 + ytotoffset;  // Y Start location Buttons

int Xsmtr   =   0;
int Ysmtr   =  80 + ytotoffset;  // S meter 85

int XVolInd =   0;
int YVolInd = 135 + ytotoffset;  // Volume indicator

int XFreqDispl  =   0;
int YFreqDispl  =   0 + ytotoffset;  // display

int Xbutsiz =  80;  //size of buttons first & third layer
int Ybutsiz =  40;

#ifdef IhaveVertTFT
uint16_t Xbut0  = 0 * Xbutsiz ; uint16_t Ybut0  = 0 * Ybutsiz; // location calqualation for 12 first layer buttons
uint16_t Xbut1  = 1 * Xbutsiz ; uint16_t Ybut1  = 0 * Ybutsiz;
uint16_t Xbut2  = 2 * Xbutsiz ; uint16_t Ybut2  = 0 * Ybutsiz;
uint16_t Xbut3  = 0 * Xbutsiz ; uint16_t Ybut3  = 1 * Ybutsiz;
uint16_t Xbut4  = 1 * Xbutsiz ; uint16_t Ybut4  = 1 * Ybutsiz;
uint16_t Xbut5  = 2 * Xbutsiz ; uint16_t Ybut5  = 1 * Ybutsiz;
uint16_t Xbut6  = 0 * Xbutsiz ; uint16_t Ybut6  = 2 * Ybutsiz;
uint16_t Xbut7  = 1 * Xbutsiz ; uint16_t Ybut7  = 2 * Ybutsiz;
uint16_t Xbut8  = 2 * Xbutsiz ; uint16_t Ybut8  = 2 * Ybutsiz;
uint16_t Xbut9  = 0 * Xbutsiz ; uint16_t Ybut9  = 3 * Ybutsiz;
uint16_t Xbut10 = 1 * Xbutsiz ; uint16_t Ybut10 = 3 * Ybutsiz;
uint16_t Xbut11 = 2 * Xbutsiz ; uint16_t Ybut11 = 3 * Ybutsiz;
#endif

#ifdef IhaveHoriTFT
uint16_t Xbut0  = 0 * Xbutsiz ; uint16_t Ybut0  =  0 * Ybutsiz; // location calqualation for 12 first layer buttons
uint16_t Xbut1  = 1 * Xbutsiz ; uint16_t Ybut1  =  0 * Ybutsiz;
uint16_t Xbut2  = 2 * Xbutsiz ; uint16_t Ybut2  =  0 * Ybutsiz;
uint16_t Xbut3  = 3 * Xbutsiz ; uint16_t Ybut3  =  0 * Ybutsiz;
uint16_t Xbut4  = 0 * Xbutsiz ; uint16_t Ybut4  =  1 * Ybutsiz;
uint16_t Xbut5  = 1 * Xbutsiz ; uint16_t Ybut5  =  1 * Ybutsiz;
uint16_t Xbut6  = 2 * Xbutsiz ; uint16_t Ybut6  =  1 * Ybutsiz;
uint16_t Xbut7  = 3 * Xbutsiz ; uint16_t Ybut7  =  1 * Ybutsiz;
uint16_t Xbut8  = 3 * Xbutsiz ; long     Ybut8  = -4 * Ybutsiz;
uint16_t Xbut9  = 3 * Xbutsiz ; long     Ybut9  = -3 * Ybutsiz;
uint16_t Xbut10 = 3 * Xbutsiz ; long     Ybut10 = -2 * Ybutsiz;
uint16_t Xbut11 = 3 * Xbutsiz ; long     Ybut11 = -1 * Ybutsiz;
#endif

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
#define RDSbut    3
#define AGCset    4
#define NR5       5
#define Displbut  6
#define NR7       7
#define ChipType  8    //                                                 |----|
#define NR10      9    //                                                 |  8 |
#define NR9      10    //                                                 |----|
#define PREV     11    //                                                 |  9 |  
                       //                                                 |----|
                       //                                                 | 10 |
#ifdef IhaveHoriTFT    //                                                 |----|
Button bt[] = {        //                                                 | 11 |
  { "HAM"   ,  0 , "SEEKUP",  0 , Xbut0 , Ybut0  }, //     |----|----|----|----|
  { "BFO"   ,  4 , "SEEKDN",  4 , Xbut1 , Ybut1  }, //     |  0 |  1 |  2 |  3 |
  { "FREQ"  ,  2 , "STATUS",  6 , Xbut2 , Ybut2  }, //     |----|----|----|----|
  { "AGC"   ,  9 , "RDS"   ,  8 , Xbut3 , Ybut3  }, //     |  4 |  5 |  6 |  7 |
  { "MUTE"  ,  8 , "AGCset",  1 , Xbut4 , Ybut4  }, //     |----|----|----|----|
  { "VOL"   ,  5 , ""      , 10 , Xbut5 , Ybut5  },
  { "MODE"  ,  3 , "DISPL" ,  2 , Xbut6 , Ybut6  },
  { "BANDW" ,  6 , ""      , 11 , Xbut7 , Ybut7  },
  { "STEP"  , 11 , "CHIP"  ,  3 , Xbut8 , Ybut8  },
  { "BROAD" ,  1 , ""      ,  5 , Xbut9 , Ybut9  },
  { "PRESET", 10 , ""      ,  9 , Xbut10, Ybut10 },
  { "NEXT"  ,  7 , "PREV"  ,  7 , Xbut11, Ybut11 }
};
#endif

#ifdef IhaveVertTFT                                                  
Button bt[] = {         
  { "HAM"   ,  0 , "SEEKUP", 0 , Xbut0 , Ybut0  }, //     |----|----|----|
  { "BFO"   ,  3 , "SEEKDN", 3 , Xbut1 , Ybut1  }, //     |  0 |  1 |  2 |
  { "FREQ"  ,  2 , "STATUS",10 , Xbut2 , Ybut2  }, //     |----|----|----|
  { "AGC"   ,  4 , "RDS"   , 9 , Xbut3 , Ybut3  }, //     |  3 |  4 |  5 |
  { "MUTE"  ,  8 , "AGCset", 2 , Xbut4 , Ybut4  }, //     |----|----|----|
  { "VOL"   ,  7 , ""      , 5 , Xbut5 , Ybut5  }, //     |  6 |  7 |  8 |
  { "MODE"  ,  9 , "Displ" , 6 , Xbut6 , Ybut6  }, //     |----|----|----|
  { "BANDW" ,  5 , ""      , 7 , Xbut7 , Ybut7  }, //     |  9 | 10 | 11 |
  { "STEP"  ,  6 , "CHIP"  , 8 , Xbut8 , Ybut8  }, //     |----|----|----|
  { "BROAD" ,  1 , ""      , 1 , Xbut9 , Ybut9  },
  { "PRESET", 10 , ""      , 4 , Xbut10, Ybut10 },
  { "NEXT"  , 11 , "PREV"  ,11 , Xbut11, Ybut11 }
};
#endif


// You may freely move around the button (blue) position on the display to your flavour by changing the position in ButtonNum and ButtonNum1
// You have to stay in the First or Third Layer
//======================================================= End  Buttons First  and Third Layer   ======================

//======================================================= Tunings Steps     ===============================
typedef struct // Tuning steps
{
  uint8_t stepFreq;
  double stepFreqFM;
  uint16_t Xstepos;          //Xoffset
  uint16_t Xstepsr;          //X size rectang
  uint16_t Ystepos;          //Yoffset
  uint16_t Ystepsr;          //Y size rectang
  uint16_t Ystepnr;          //Y next rectang
} Step;

//  Tuning steps table

#ifdef IhaveHoriTFT
uint16_t Xfstep = 110;
uint16_t Yfstep = 60;
#endif

#ifdef IhaveVertTFT
uint16_t Xfstep =  70;
uint16_t Yfstep = 100;
#endif

Step sp[] = {
  { 1 , 10   ,Xfstep, 100, Yfstep, 30,  0},
  { 5 ,  1   ,Xfstep, 100, Yfstep, 30, 30},
  { 9 ,  0   ,Xfstep, 100, Yfstep, 30, 60},
  { 10,  0   ,Xfstep, 100, Yfstep, 30, 90}
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

#ifdef IhaveHoriTFT
uint16_t Xfmod = 110;
uint16_t Yfmod = 45;
#endif

#ifdef IhaveVertTFT
uint16_t Xfmod = 70;
uint16_t Yfmod = 90;
#endif

Mode md[] = {
  { 0  , Xfmod, 100, Yfmod, 30,  0},
  { 1  , Xfmod, 100, Yfmod, 30, 30},
  { 2  , Xfmod, 100, Yfmod, 30, 60},
  { 3  , Xfmod, 100, Yfmod, 30, 90},
  { 4  , Xfmod, 100, Yfmod, 30,120}
};
//======================================================= End Modulation Types     ============================

//======================================================= Keypath     =========================================
typedef struct // Keypath
{
  uint16_t KeypNum;
  uint16_t Xkeypos;          //Xoffset
  uint16_t Xkeypsr;          //X size rectang
  uint16_t Xkeypnr;          //Y next rectang
  uint16_t Ykeypos;          //Yoffset
  uint16_t Ykeypsr;          //X size rectang
  uint16_t Ykeypnr;          //Y next rectang
} Keypath;

//  Keypath table

#ifdef IhaveHoriTFT
uint16_t Xpath = 82;
uint16_t Ypath = 20;

Keypath kp[] = {
  {  0 , Xpath,  50 ,   0 , Ypath , 50 ,   0},
  {  1 , Xpath,  50 ,  50 , Ypath , 50 ,   0},
  {  2 , Xpath,  50 , 100 , Ypath , 50 ,   0},
  {  3 , Xpath,  50 ,   0 , Ypath , 50 ,  50},
  {  4 , Xpath,  50 ,  50 , Ypath , 50 ,  50},
  {  5 , Xpath,  50 , 100 , Ypath , 50 ,  50},
  {  6 , Xpath,  50 ,   0 , Ypath , 50 , 100},
  {  7 , Xpath,  50 ,  50 , Ypath , 50 , 100},
  {  8 , Xpath,  50 , 100 , Ypath , 50 , 100},
  {  9 , Xpath,  50 ,   0 , Ypath , 50 , 150},
  { 10 , Xpath,  50 ,  50 , Ypath , 50 , 150},
  { 11 , Xpath,  50 , 100 , Ypath , 50 , 150},
};
#endif

#ifdef IhaveVertTFT
uint16_t Xpath = 35;
uint16_t Ypath = 47;

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
#endif
//======================================================= End Keypath     =====================================

//======================================================= Bandwidth AM, SSB, FM     ===============================
typedef struct // Bandwidth AM & SSB & FM
{
  uint16_t BandWidthAM;
  uint16_t BandWidthSSB;
  uint16_t BandWidthFM;
  uint16_t Xos;          //Xoffset
  uint16_t Xsr;          //X size rectang
  uint16_t Yos;          //Yoffset
  uint16_t Ysr;          //X size rectang
  uint16_t Ynr;          //Y next rectang
} Bandwidth;

//  Bandwidth table
#ifdef IhaveHoriTFT
uint16_t XfBW = 110;
uint16_t YfBW = 20;
#endif

#ifdef IhaveVertTFT
uint16_t XfBW = 70;
uint16_t YfBW = 40;                                                
#endif                         

Bandwidth bw[] = {                                 //  AM    SSB   FM
  { 4 , 4 , 0 , XfBW, 100, YfBW, 30,   0},         //  1.0   0.5   Aut
  { 5 , 5 , 1 , XfBW, 100, YfBW, 30,  30},         //  1.8   1.0   110
  { 3 , 0 , 2 , XfBW, 100, YfBW, 30,  60},         //  2.0   1.2    84
  { 6 , 1 , 3 , XfBW, 100, YfBW, 30,  90},         //  2.5   2.2    60
  { 2 , 2 , 4 , XfBW, 100, YfBW, 30, 120},         //  3.0   3.0    40
  { 1 , 3 , 0 , XfBW, 100, YfBW, 30, 150},         //  4.0   4.0   ---
  { 0 , 0 , 0 , XfBW, 100, YfBW, 30, 180}          //  6.0   ---   ---
};
//======================================================= End Bandwidth AM & FM & SSB ===========================

//=======================================================    Broad Band Definitions   ==========================
typedef struct // Broad-Band switch
{
  uint16_t BbandNum;          // bandIdx
  uint16_t Xbbandos;          //Xoffset
  uint16_t Xbbandsr;          //X size rectang
  uint16_t Xbbandnr;          //X next rectang
  uint16_t Ybbandos;          //Yoffset
  uint16_t Ybbandsr;          //X size rectang
  uint16_t Ybbandnr;          //Y next rectang
} BBandnumber;

//  Bandnumber table for the broad-bands

#ifdef IhaveVertTFT
uint16_t Xfbband = 0;
uint16_t Yfbband = 45;
#endif

#ifdef IhaveHoriTFT
uint16_t Xfbband = 40;
uint16_t Yfbband = 15;
#endif


BBandnumber bb[] = {
  {  0 , Xfbband, 80 ,   0 , Yfbband , 30 ,   0}, // 0
  {  1 , Xfbband, 80 ,   0 , Yfbband , 30 ,  30}, // 1
  {  2 , Xfbband, 80 ,   0 , Yfbband , 30 ,  60}, // 2
  {  6 , Xfbband, 80 ,   0 , Yfbband , 30 ,  90}, // 3
  {  7 , Xfbband, 80 ,   0 , Yfbband , 30 , 120}, // 4
  {  9 , Xfbband, 80 ,   0 , Yfbband , 30 , 150}, // 5
  { 11 , Xfbband, 80 ,   0 , Yfbband , 30 , 180}, // 6
  { 13 , Xfbband, 80 ,  80 , Yfbband , 30 ,   0}, // 7
  { 14 , Xfbband, 80 ,  80 , Yfbband , 30 ,  30}, // 8
  { 16 , Xfbband, 80 ,  80 , Yfbband , 30 ,  60}, // 9
  { 17 , Xfbband, 80 ,  80 , Yfbband , 30 ,  90}, //10
  { 19 , Xfbband, 80 ,  80 , Yfbband , 30 , 120}, //11
  { 21 , Xfbband, 80 ,  80 , Yfbband , 30 , 150}, //12
  { 22 , Xfbband, 80 ,  80 , Yfbband , 30 , 180}, //13
  { 24 , Xfbband, 80 , 160 , Yfbband , 30 ,   0}, //14
  { 26 , Xfbband, 80 , 160 , Yfbband , 30 ,  30}, //15
  { 27 , Xfbband, 80 , 160 , Yfbband , 30 ,  60}, //16
  { 29 , Xfbband, 80 , 160 , Yfbband , 30 ,  90}  //17
};
//======================================================= End Broad Band Definitions     ======================

//======================================================= Ham Band Definitions     ============================
typedef struct // Ham Band switch
{
  uint16_t BandNum;          // bandIdx
  uint16_t HamBandTxt;
  uint16_t Xbandos;          //Xoffset
  uint16_t Xbandsr;          //X size rectang
  uint16_t Xbandnr;          //X next rectang
  uint16_t Ybandos;          //Yoffset
  uint16_t Ybandsr;          //Y size rectang
  uint16_t Ybandnr;          //Y next rectang
} Bandnumber;

//  Bandnumber table for the hambands

#ifdef IhaveVertTFT
uint16_t Xfband = 10;
uint16_t Yfband = 30;
#endif

#ifdef IhaveHoriTFT
uint16_t Xfband = 50;
uint16_t Yfband = 30;
#endif

Bandnumber bn[] = {
  {  3 ,  0 , Xfband, 110 ,   0 , Yfband , 30 ,   0},
  {  4 ,  1 , Xfband, 110 ,   0 , Yfband , 30 ,  30},
  {  5 ,  2 , Xfband, 110 ,   0 , Yfband , 30 ,  60},
  {  8 ,  3 , Xfband, 110 ,   0 , Yfband , 30 ,  90},
  { 10 ,  4 , Xfband, 110 ,   0 , Yfband , 30 , 120},
  { 12 ,  5 , Xfband, 110 ,   0 , Yfband , 30 , 150},
  { 15 ,  6 , Xfband, 110 , 110 , Yfband , 30 ,   0},
  { 18 ,  7 , Xfband, 110 , 110 , Yfband , 30 ,  30},
  { 20 ,  8 , Xfband, 110 , 110 , Yfband , 30 ,  60},
  { 23 ,  9 , Xfband, 110 , 110 , Yfband , 30 ,  90},
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
  uint8_t  currentStep; // Default step (increment and decrement)
  int      lastBFO ;    // Last BFO per band
  
} Band;

//   Band table

Band band[] = {
  {   "FM", FM_BAND_TYPE,  FM,  8750, 10800,  8930, 10, 0}, //  FM          0
  {   "LW", LW_BAND_TYPE,  AM,   130,   279,   198,  9, 0}, //  LW          1
  {   "MW", MW_BAND_TYPE,  AM,   522,  1701,  1395,  9, 0}, //  MW          2
  {"BACON", LW_BAND_TYPE,  AM,   280,   470,   284,  1, 0}, // Ham          3
  { "630M", SW_BAND_TYPE, LSB,   472,   479,   475,  1, 0}, // Ham  630M    4
  { "160M", SW_BAND_TYPE, LSB,  1800,  1910,  1899,  1, 0}, // Ham  160M    5
  { "120M", SW_BAND_TYPE,  AM,  2300,  2495,  2400,  5, 0}, //      120M    6
  {  "90M", SW_BAND_TYPE,  AM,  3200,  3400,  3300,  5, 0}, //       90M    7
  {  "80M", SW_BAND_TYPE, LSB,  3500,  3800,  3630,  1, 0}, // Ham   80M    8
  {  "75M", SW_BAND_TYPE,  AM,  3900,  4000,  3950,  5, 0}, //       75M    9
  {  "60M", SW_BAND_TYPE, USB,  5330,  5410,  5375,  1, 0}, // Ham   60M   10
  {  "49M", SW_BAND_TYPE,  AM,  5900,  6200,  6000,  5, 0}, //       49M   11
  {  "40M", SW_BAND_TYPE, LSB,  7000,  7200,  7185,  1, 0}, // Ham   40M   12
  {  "41M", SW_BAND_TYPE,  AM,  7200,  7450,  7210,  5, 0}, //       41M   13
  {  "31M", SW_BAND_TYPE,  AM,  9400,  9900,  9600,  5, 0}, //       31M   14
  {  "30M", SW_BAND_TYPE, USB, 10100, 10150, 10125,  1, 0}, // Ham   30M   15
  {  "25M", SW_BAND_TYPE,  AM, 11600, 12100, 11700,  5, 0}, //       25M   16
  {  "22M", SW_BAND_TYPE,  AM, 13570, 13870, 13700,  5, 0}, //       22M   17
  {  "20M", SW_BAND_TYPE, USB, 14000, 14350, 14250,  1, 0}, // Ham   20M   18
  {  "19M", SW_BAND_TYPE,  AM, 15100, 15830, 15700,  5, 0}, //       19M   19
  {  "17M", SW_BAND_TYPE, USB, 18068, 18168, 18100,  1, 0}, // Ham   17M   20
  {  "16M", SW_BAND_TYPE,  AM, 17480, 17900, 17600,  5, 0}, //       16M   21
  {  "15M", SW_BAND_TYPE,  AM, 18900, 19020, 18950,  5, 0}, //       15M   22
  {  "15M", SW_BAND_TYPE, USB, 21000, 21450, 21350,  1, 0}, // Ham   15M   23
  {  "13M", SW_BAND_TYPE,  AM, 21450, 21850, 21500,  5, 0}, //       13M   24
  {  "12M", SW_BAND_TYPE, USB, 24890, 24990, 24940,  1, 0}, // Ham   12M   25
  {  "11M", SW_BAND_TYPE,  AM, 25670, 26100, 25800,  5, 0}, //       11M   26
  {   "CB", SW_BAND_TYPE,  AM, 26200, 27990, 27200,  1, 0}, // CB band     27
  {  "10M", SW_BAND_TYPE, USB, 28000, 30000, 28500,  1, 0}, // Ham   10M   28
  {   "SW", SW_BAND_TYPE,  AM,  1730, 30000, 15500,  5, 0}  // Whole SW    29
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
  9220  , "L-FM",      // 17 L-FM
  10760 , "FEELGOOD"   // 18 FEELGOOD Radio
};
//======================================================= END FM Presets     ======================================

//======================================================= Tuning Digit selection ====================
typedef struct // Tuning digit
{
  uint8_t  digit;
  uint16_t Xdignumos;          //Xoffset
  uint16_t Xdignumsr;          //X size rectang
  uint16_t Ydignumos;          //Yoffset
  uint16_t Ydignumsr;          //Y size rectang
  uint16_t Xdignumnr;          //X next rectang
} DigNum;

uint8_t Xdignum = 139;
uint8_t Ydignum = 25;

//  Tuning digit table

DigNum dn[] = {
  { 0 ,Xdignum, 21, Ydignum, 35,  0},
  { 1 ,Xdignum, 21, Ydignum, 35, 30},
  { 2 ,Xdignum, 21, Ydignum, 35, 59}

};
//======================================================= End Tuning Digit selection     ===============================

const int lastButton = (sizeof bt / sizeof(Button)) - 1;
const int lastBand   = (sizeof band / sizeof(Band)) - 1;
const int lastHam    = (sizeof bn / sizeof(Bandnumber)) - 1;
const int lastBroad  = (sizeof bb / sizeof(BBandnumber)) - 1;
const int lastMod    = (sizeof md / sizeof(Mode)) - 1;
const int lastBW     = (sizeof bw / sizeof(Bandwidth)) - 1;
const int lastStep   = (sizeof sp / sizeof(Step)) - 1;
const int lastdignum = (sizeof dn / sizeof(DigNum)) - 1;
const int lastKPath  = (sizeof kp / sizeof(Keypath)) - 1;
const int lastPreset = (sizeof preset / sizeof (FM_Preset)) - 1;

#define offsetEEPROM       0x20
#define EEPROM_SIZE        265

struct StoreStruct {
  byte     chkDigit;
  byte     bandIdx;
  uint16_t Freq;
  uint8_t  currentMode;
  uint8_t  bwIdxSSB;
  uint8_t  bwIdxAM;
  uint8_t  bwIdxFM;
  uint8_t  ssIdxMW;
  uint8_t  ssIdxAM;
  uint8_t  ssIdxFM;
  int      currentBFO;
  int      currentBFOmanu;
  uint8_t  currentAGCAtt;
  uint8_t  currentVOL;
  uint8_t  currentBFOStep;
  uint8_t  RDS;
  unsigned long FreqSI5351;
  uint16_t currentBrightness;
  uint8_t  currentAGCgain;
  unsigned long calibratvalSI5351;
};

StoreStruct storage = {
      'A',  //First time check
        0,  //bandIdx
     8930,  //Freq
        0,  //mode
        1,  //bwIdxSSB
        1,  //bwIdxAM
        0,  //bwIdxFM
        9,  //ssIdxMW
        5,  //ssIdxAM
       10,  //ssIdxFM  
        0,  //currentBFO
        0,  //currentBFOmanu
        2,  //currentAGCAtt
       45,  //currentVOL
       25,  //currentBFOStep
        1,  //RDS
  3276800,  //FreqSI5351
       20,  //currentBrightness
        1,  //currentAGCgain
        0   //calibratvalSI5351 
};

uint8_t rssi = 0;
uint8_t stereo = 1;
uint8_t volume = DEFAULT_VOLUME;

// Devices class declarations
Rotary encoder = Rotary(ENCODER_PIN_A, ENCODER_PIN_B);

TFT_eSPI tft    = TFT_eSPI();
TFT_eSprite spr = TFT_eSprite(&tft);
SI4735 si4735;

Si5351wire si5351wire;

//=======================================================================================
void IRAM_ATTR RotaryEncFreq() {
//=======================================================================================
  // rotary encoder events
  if (!writingEeprom) {
    encoderStatus = encoder.process();

    if (encoderStatus)
    {
      if (encoderStatus == DIR_CW)// Direction clockwise
      {
        encoderCount = 1;
      }
      else
      {
        encoderCount = -1;
      }
    }
  }
}

//=======================================================================================
void setup() {
  //=======================================================================================
  Serial.begin(115200);
  pinMode(Display_Led, OUTPUT);
  pinMode(BEEPER, OUTPUT);
  digitalWrite(Display_Led, displayon);
  DISplay = true;
  //Wire.begin(ESP32_I2C_SDA, ESP32_I2C_SCL); //I2C for SI4735
  
  ledcSetup(LedChannelforTFT, LedFreq, LedResol);
  ledcAttachPin(Display_Led, LedChannelforTFT);
  
  int16_t si4735Addr = si4735.getDeviceI2CAddress(RESET_PIN);
  Beep(1, 200);
  tft.init();
  //tft.setRotation(0); // Rotate 0
  //tft.setRotation(1); // Rotate 90
  //tft.setRotation(2); // Rotate 180
  //tft.setRotation(3); // Rotate 270
#ifdef IhaveVertTFT
  // Calibration code for touchscreen : for 2.8 inch & Rotation = 2
  tft.setRotation(2);
  uint16_t calData[5] = { 258, 3566, 413, 3512, 2 };
  tft.setTouch(calData);
#endif

#ifdef IhaveHoriTFT
  // Calibration code for touchscreen : Rotation = 1
  tft.setRotation(1);
  uint16_t calData[5] = { 387, 3530, 246, 3555, 7 };
  tft.setTouch(calData);
#endif



#ifdef IhaveSI5351
  si5351wire.output_enable(CLK_Xtal, 1);
  if (si5351wire.init(SI5351wire_CRYSTAL_LOAD_8PF, CLK_Xtal, 0) == false)
  {
    Serial.println ( "SI5351 not found" );
  }
  //si5351wire.set_freq(1000000000UL, CLK_Xtal); // used for calibrating 10MHz
  si5351wire.set_correction(0, SI5351wire_PLL_INPUT_XO);
  //si5351wire.set_correction(26613UL, SI5351wire_PLL_INPUT_XO);  // Calibration example with 10 MHz replace 26613UL with your figure.
  si5351wire.set_freq(FreqSI5351, CLK_Xtal);
#endif

  if (!EEPROM.begin(EEPROM_SIZE))
  {
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(0, 0);
    tft.println(F("failed to initialise EEPROM"));
    Serial.println(F("failed to initialise EEPROM"));
    while (1);
  }



  if (EEPROM.read(offsetEEPROM) != storage.chkDigit) {
    Serial.println(F("Writing defaults...."));
    saveConfig();
  }
  loadConfig();
  printConfig();

  //Wire.begin(ESP32_I2C_SDA, ESP32_I2C_SCL); //I2C for SI4735

  // Encoder pins
  pinMode(ENCODER_PIN_A , INPUT_PULLUP); //Rotary encoder Freqency/bfo/preset
  pinMode(ENCODER_PIN_B , INPUT_PULLUP);
  // Encoder interrupt
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), RotaryEncFreq, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), RotaryEncFreq, CHANGE);
  si4735.setAudioMuteMcuPin(AUDIO_MUTE);

  tft.fillScreen(TFT_BLACK);
  delay(500);
  tft.setCursor(7, 50);
  tft.setTextSize(2);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);

  Serial.println("     SI4735/32 Radio");
  Serial.println("Version 3.3 02-11-2021");
  tft.println("SI4735/32  Radio");
  tft.setCursor(7, 70);
  tft.println(" Version 3.3");
  tft.setCursor(7, 95);
  tft.println(" 02-11-2021");
  tft.setTextColor(TFT_BLUE, TFT_BLACK);
  tft.setCursor(7, 170);
  delay(1000);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  if ( si4735Addr == 0 ) {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.print("Si4735 not detected");
    Serial.println("Si4735 not detected");
    while (1);
  } else {
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.print("Si473X addr :  ");
    tft.println(si4735Addr, HEX);
  }

  delay(1500);

  if (si4735Addr == 17)
  {
    si4735.setDeviceI2CAddress(0);
  }
  else
  {
    si4735.setDeviceI2CAddress(1);
  }

  // Setup the radio from last setup in EEPROM
  bandIdx                   = storage.bandIdx;
  band[bandIdx].currentFreq = storage.Freq;
  currentMode               = storage.currentMode;
  bwIdxSSB                  = storage.bwIdxSSB; // band width
  bwIdxAM                   = storage.bwIdxAM;
  bwIdxFM                   = storage.bwIdxFM;
  ssIdxMW                   = storage.ssIdxMW; // step size
  ssIdxAM                   = storage.ssIdxAM;
  ssIdxFM                   = storage.ssIdxFM;
  currentBFO                = storage.currentBFO;
  currentBFOmanu            = storage.currentBFOmanu;
  currentAGCAtt             = storage.currentAGCAtt;
  currentVOL                = storage.currentVOL;
  currentBFOStep            = storage.currentBFOStep;
  RDS                       = storage.RDS;
  FreqSI5351                = storage.FreqSI5351;
  currentBrightness         = storage.currentBrightness;
  currentAGCgain            = storage.currentAGCgain;
  calibratvalSI5351         = storage.calibratvalSI5351;
#ifdef IhaveCrystal
 if (bandIdx == 0) si4735.setup(RESET_PIN, FM_BAND_TYPE); //Start in FM
 else si4735.setup(RESET_PIN, 1);
 if (bandIdx != 0) si4735.setAM(); // Start in AM
#endif

#ifdef IhaveSI5351
  si5351wire.set_freq(FreqSI5351, CLK_Xtal);
  si4735.setRefClock(32768);
  si4735.setRefClockPrescaler(1);   // will work with 32768 Hz
  if (bandIdx == 0)  si4735.setup(RESET_PIN, -1, POWER_UP_FM, SI473X_ANALOG_AUDIO, XOSCEN_RCLK); // Start in FM
  else si4735.setup(RESET_PIN, -1, POWER_UP_AM, SI473X_ANALOG_AUDIO, XOSCEN_RCLK); // Start in AM
  if (bandIdx != 0) si4735.setAM();
#endif

  ledcWrite(LedChannelforTFT, currentBrightness);
  freqstep = 1000;//hz
  previousBFO = -1;
  band[bandIdx].lastBFO  = currentBFO;
  freqDec = currentBFO;
  band[bandIdx].prefmod = currentMode;
  si4735.setVolume(currentVOL);
  previousVOL = currentVOL;
  previousAGCgain = currentAGCgain;
  BandSet();
  currentFrequency = previousFrequency = band[bandIdx].currentFreq;
  Beep(2, 200);
  encBut = 600;
  x = y = 0;
  DrawFila();
  si4735.setSeekFmSpacing(10);
  si4735.setSeekFmLimits(8750, 10800);
  si4735.setSeekAmRssiThreshold(50);
  si4735.setSeekAmSrnThreshold(20);
  si4735.setSeekFmRssiThreshold(5);
  si4735.setSeekFmSrnThreshold(5);
  xTaskCreate(SaveInEeprom, "SaveInEeprom", 2048, NULL, 1, NULL);
  delay(10);
}// end setup
//=======================================================================================
//=======================================================================================


//=======================================================================================
void SaveInEeprom (void* arg)  {
//=======================================================================================
  while (1) {
    storage.bandIdx           = bandIdx;
    storage.Freq              = band[bandIdx].currentFreq;
    storage.currentMode       = currentMode;
    storage.bwIdxSSB          = bwIdxSSB;
    storage.bwIdxAM           = bwIdxAM;
    storage.bwIdxFM           = bwIdxFM;
    storage.ssIdxMW           = ssIdxMW;
    storage.ssIdxAM           = ssIdxAM;
    storage.ssIdxFM           = ssIdxFM;
    storage.currentBFO        = currentBFO;
    storage.currentBFOmanu    = currentBFOmanu;
    storage.currentAGCAtt     = currentAGCAtt;
    storage.currentVOL        = currentVOL;
    storage.currentBFOStep    = currentBFOStep;
    storage.RDS               = RDS;
    storage.FreqSI5351           = FreqSI5351;
    storage.currentBrightness = currentBrightness;
    storage.currentAGCgain    = currentAGCgain;
    storage.calibratvalSI5351 = calibratvalSI5351;
    
    for (unsigned int t = 0; t < sizeof(storage); t++) {
      delay(1);
      if (EEPROM.read(offsetEEPROM + t) != *((char*)&storage + t)) {
        delay(1);
        EEPROM.write(offsetEEPROM + t, *((char*)&storage + t));
      }
    }

    writingEeprom = true;
    EEPROM.commit();
    writingEeprom = false;
    vTaskDelay(5000 / portTICK_RATE_MS);
  }

}

//=======================================================================================
void saveConfig() {
//=======================================================================================
  delay(10);
  for (unsigned int t = 0; t < sizeof(storage); t++) {
    if (EEPROM.read(offsetEEPROM + t) != *((char*)&storage + t)) {
      EEPROM.write(offsetEEPROM + t, *((char*)&storage + t));
    }
  }
  EEPROM.commit();
}

//=======================================================================================
void loadConfig() {
//=======================================================================================
  if (EEPROM.read(offsetEEPROM + 0) == storage.chkDigit) {
    for (unsigned int t = 0; t < sizeof(storage); t++)
      *((char*)&storage + t) = EEPROM.read(offsetEEPROM + t);
    Serial.println("Load config done");
  }
}

//=======================================================================================
void printConfig() {
//=======================================================================================
  Serial.print("Storage = ");
  Serial.println(sizeof(storage));
  if (EEPROM.read(offsetEEPROM) == storage.chkDigit) {
    for (unsigned int t = 0; t < sizeof(storage); t++)
      Serial.write(EEPROM.read(offsetEEPROM + t));
      Serial.println();
  }
}

//=======================================================================================
void BandSet()  {
//=======================================================================================
  if (bandIdx == 0) currentMode = FM;// only mod FM in FM band
  if ((currentMode == AM) or (currentMode == FM)) {
    ssbLoaded = false;
  }
  if ((currentMode == LSB) or  (currentMode == USB)) 
  {
    if (ssbLoaded == false) {
      loadSSB();
    } 
  }
  useBand();
  setBandWidth();
}

//=======================================================================================
void useBand()  {
//=======================================================================================
  if ((band[bandIdx].bandType == MW_BAND_TYPE) || (band[bandIdx].bandType == LW_BAND_TYPE)) {
    band[bandIdx].currentStep = ssIdxMW;
  }    
  if (band[bandIdx].bandType == SW_BAND_TYPE) {   
    band[bandIdx].currentStep = ssIdxAM;
  }
  if (band[bandIdx].bandType == FM_BAND_TYPE)
  {
    bfoOn = false;
    si4735.setTuneFrequencyAntennaCapacitor(0);
    delay(100);
    band[bandIdx].currentStep = ssIdxFM;
    si4735.setFM(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq, band[bandIdx].currentFreq, band[bandIdx].currentStep);
    si4735.setFMDeEmphasis(1);
    ssbLoaded = false;
    si4735.RdsInit();
    si4735.setRdsConfig(1, 2, 2, 2, 2);
  }
  else
  {
    if (band[bandIdx].bandType == MW_BAND_TYPE || band[bandIdx].bandType == LW_BAND_TYPE) {
      si4735.setTuneFrequencyAntennaCapacitor(0);
    }else{
      si4735.setTuneFrequencyAntennaCapacitor(1);
    }
    if (ssbLoaded)
    {
      si4735.setSSB(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq, band[bandIdx].currentFreq, band[bandIdx].currentStep, currentMode);
      si4735.setSSBAutomaticVolumeControl(1);
      //si4735.setSsbSoftMuteMaxAttenuation(0); // Disable Soft Mute for SSB
      //si4735.setSSBDspAfc(0);
      //si4735.setSSBAvcDivider(3);
      //si4735.setSsbSoftMuteMaxAttenuation(8); // Disable Soft Mute for SSB
      //si4735.setSBBSidebandCutoffFilter(0);
      si4735.setSSBBfo(currentBFO);
      int temp = 1; // SSB ONLY 1KHz stepsize
      si4735.setFrequencyStep(temp);
      band[bandIdx].currentStep = temp;
    }
    else
    {
      si4735.setAM(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq, band[bandIdx].currentFreq, band[bandIdx].currentStep);
      //si4735.setAutomaticGainControl(1, 0);
      //si4735.setAmSoftMuteMaxAttenuation(0); // // Disable Soft Mute for AM
      bfoOn = false;
    }
  }
  delay(100);

}// end useband

//=======================================================================================
void setBandWidth()  {
//=======================================================================================
  if (currentMode == LSB || currentMode == USB)
  {
    si4735.setSSBAudioBandwidth(bwIdxSSB);
    // If audio bandwidth selected is about 2 kHz or below, it is recommended to set Sideband Cutoff Filter to 0.
    if (bwIdxSSB == 0 || bwIdxSSB == 4 || bwIdxSSB == 5)
      si4735.setSBBSidebandCutoffFilter(0);
    else
      si4735.setSBBSidebandCutoffFilter(1);
  }
  if (currentMode == AM)
  {
    si4735.setBandwidth(bwIdxAM, 0);
  }
  if (currentMode == FM)
  {
    si4735.setFmBandwidth(bwIdxFM);
  }
}

//=======================================================================================
void loadSSB()  {
//=======================================================================================
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
  si4735.setSSBConfig(bwIdxSSB, 1, 0, 1, 0, 1);
  delay(25);
  ssbLoaded = true;
}


//=======================================================================================
void Freqcalq(int keyval)  {
//=======================================================================================
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


//=======================================================================================
void Smeter() {
//=======================================================================================
  int spoint;
  if (currentMode != FM) {
    //dBuV to S point conversion HF
    if ((rssi >= 0) and (rssi <=  1)) spoint =  12;                    // S0
    if ((rssi >  1) and (rssi <=  1)) spoint =  24;                    // S1
    if ((rssi >  2) and (rssi <=  3)) spoint =  36;                    // S2
    if ((rssi >  3) and (rssi <=  4)) spoint =  48;                    // S3
    if ((rssi >  4) and (rssi <= 10)) spoint =  48 + (rssi - 4) * 2;   // S4
    if ((rssi > 10) and (rssi <= 16)) spoint =  60 + (rssi - 10) * 2;  // S5
    if ((rssi > 16) and (rssi <= 22)) spoint =  72 + (rssi - 16) * 2;  // S6
    if ((rssi > 22) and (rssi <= 28)) spoint =  84 + (rssi - 22) * 2;  // S7
    if ((rssi > 28) and (rssi <= 34)) spoint =  96 + (rssi - 28) * 2;  // S8
    if ((rssi > 34) and (rssi <= 44)) spoint = 108 + (rssi - 34) * 2;  // S9
    if ((rssi > 44) and (rssi <= 54)) spoint = 124 + (rssi - 44) * 2;  // S9 +10
    if ((rssi > 54) and (rssi <= 64)) spoint = 140 + (rssi - 54) * 2;  // S9 +20
    if ((rssi > 64) and (rssi <= 74)) spoint = 156 + (rssi - 64) * 2;  // S9 +30
    if ((rssi > 74) and (rssi <= 84)) spoint = 172 + (rssi - 74) * 2;  // S9 +40
    if ((rssi > 84) and (rssi <= 94)) spoint = 188 + (rssi - 84) * 2;  // S9 +50
    if  (rssi > 94)                   spoint = 204;                    // S9 +60
    if  (rssi > 95)                   spoint = 208;                    //>S9 +60
  }
  else
  {
    //dBuV to S point conversion FM
    if  (rssi <  1) spoint = 36;
    if ((rssi >  1) and (rssi <=  2)) spoint =  60;                    // S6
    if ((rssi >  2) and (rssi <=  8)) spoint =  84 + (rssi - 2) * 2;   // S7
    if ((rssi >  8) and (rssi <= 14)) spoint =  96 + (rssi - 8) * 2;   // S8
    if ((rssi > 14) and (rssi <= 24)) spoint = 108 + (rssi - 14) * 2;  // S9
    if ((rssi > 24) and (rssi <= 34)) spoint = 124 + (rssi - 24) * 2;  // S9 +10
    if ((rssi > 34) and (rssi <= 44)) spoint = 140 + (rssi - 34) * 2;  // S9 +20
    if ((rssi > 44) and (rssi <= 54)) spoint = 156 + (rssi - 44) * 2;  // S9 +30
    if ((rssi > 54) and (rssi <= 64)) spoint = 172 + (rssi - 54) * 2;  // S9 +40
    if ((rssi > 64) and (rssi <= 74)) spoint = 188 + (rssi - 64) * 2;  // S9 +50
    if  (rssi > 74)                   spoint = 204;                    // S9 +60
    if  (rssi > 76)                   spoint = 208;                    //>S9 +60
  }

  tft.fillRect(Xsmtr + 15, Ysmtr + 38 , (2 + spoint), 6, TFT_RED);
  tft.fillRect(Xsmtr + 17 + spoint, Ysmtr + 38 , 212 - (2 + spoint), 6, TFT_GREEN);
}

//=======================================================================================
void VolumeIndicator(int vol) {
//=======================================================================================
  vol = map(vol, 20, 63, 0, 212);
  tft.fillRect(XVolInd + 15, YVolInd + 16 , (2 + vol), 6, TFT_RED);
  tft.fillRect(XVolInd + 17 + vol, YVolInd + 16 , 212 - (2 + vol), 6, TFT_GREEN);
}

//=======================================================================================
void brightnessIndicator(int vol) {
//=======================================================================================
  vol = map(vol, 250, 20, 0, 212);
  tft.fillRect(XVolInd + 15, YVolInd + 16 , (2 + vol), 6, TFT_RED);
  tft.fillRect(XVolInd + 17 + vol, YVolInd + 16 , 212 - (2 + vol), 6, TFT_GREEN);
}

//=======================================================================================
void loop() {
//=======================================================================================
  unsigned long now = millis();
  if (((FirstLayer == true) or (ThirdLayer == true)) and (bright == false )) VolumeIndicator(si4735.getVolume());
  if ((ThirdLayer == true) and (bright)) brightnessIndicator(currentBrightness);

  
  // Pressed will be set true is there is a valid touch on the screen
  
  while (((pressed == false) and (encoderCount == 0) and (encoderButton == 0) and (analogRead(ENCODER_SWITCH) > 500)) or (writingEeprom)) {  // wait loop
    pressed = tft.getTouch(&x, &y);
    showtimeRSSI();
    DisplayRDS();
    Dispoff();
    MuteAud();
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
      //digitalWrite(Display_Led, displayon);
      ledcWrite(LedChannelforTFT, currentBrightness);
      DISplay = true;
      x = y = 0; // no valid touch only for switch on display led
    }
    if (FirstLayer) { //==================================================
      
      if (( x > 139) and (x < 219) and (y > 25) and (y < 60)) {    // digit selection
        for (int n = 0 ; n <=lastdignum ; n++) { 
          if ((x > (dn[n].Xdignumos) + (dn[n].Xdignumnr)) and (x < ((dn[n].Xdignumos) + (dn[n].Xdignumsr) + (dn[n].Xdignumnr))) and (y > (dn[n].Ydignumos) and (y < ((dn[n].Ydignumos) + (dn[n].Ydignumsr) )))) {
            Beep(1, 0);  
            delay(400);
            if ((currentMode == LSB || currentMode == USB || currentMode == CW) and (bfoOn == false)){
              freqstepnr = n;
              if (freqstepnr == 0)  freqstep = 1000;
              if (freqstepnr == 1)  freqstep = 100;
              if (freqstepnr == 2)  freqstep = 10;    
              FreqDispl();
            }
            if ((currentMode == LSB || currentMode == USB || currentMode == CW) and (bfoOn == true)){
              if (n ==1) stepsizesynth = 10;
              if (n ==2) stepsizesynth = 1;
              FreqDispl();
            }
          }
        }
      }
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

          if ((bfoOn == true) and ((n == VOL) or (bright == true))) {
            bfoOn = false;
            drawBFO();
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

            if (currentMode == LSB || currentMode == USB || currentMode == CW)  {
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
            AGCgain = 0;
            if  (si4735.isAgcEnabled()) {
              si4735.setAutomaticGainControl(1, 0);     //    disabled
            } else {
              AGCgainbut = false;
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
              BandWidth = true;
              BWList();
              FirstLayer = false;
              SecondLayer = true;
          }


          if (n == STEP) {            //========================= STEPS for tune and bfo
            delay(400);
            
              if (bfoOn) setStep();
              if (currentMode == AM || currentMode == FM){
                STEPbut = true;
                Steplist();
                FirstLayer = false;
                SecondLayer = true;
              } 
              if (currentMode == LSB || currentMode == USB || currentMode == CW){
                Beep(4, 100);
              }          
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
            currentMode = n;
            if ((CWShift == true) and (previousMode == USB)  ) {  
              currentBFO = currentBFO - 700;
              band[bandIdx].lastBFO = currentBFO;
              freqDec = currentBFO;
              CWShift = false;
            }
             if ((currentMode !=  previousMode) and (currentMode == CW) and (CWShift == false)) {
              currentMode = USB;
              CWShift = true;
              currentBFO = currentBFO + 700;
              band[bandIdx].lastBFO = currentBFO;
              freqDec = currentBFO;
            }
            Modebut = false;
            previousMode = currentMode;
            band[bandIdx].prefmod = currentMode;
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
              DrawFila();
            }
          }
        }
         if ((currentMode == LSB) or  (currentMode == USB) or (currentMode == CW)) {
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
        
        if (currentMode == FM) {
          for (int n = 0 ; n < 5; n++) {
            if ((x > bw[n].Xos) and (x < ((bw[n].Xos) + (bw[n].Xsr))) and (y > ((bw[n].Yos) + (bw[n].Ynr))) and (y < ((bw[n].Yos) + (bw[n].Ysr) + (bw[n].Ynr)))) {
              Beep(1, 0);
              delay(400);
              bwIdxFM = bw[n].BandWidthFM;
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
            if (((band[bandIdx].bandType == MW_BAND_TYPE) or (band[bandIdx].bandType == LW_BAND_TYPE)) and ( currentMode == AM)) {           
              ssIdxMW = sp[n].stepFreq;
              si4735.setFrequencyStep(ssIdxMW);
              band[bandIdx].currentStep = ssIdxMW;
            }
            
            if ((band[bandIdx].bandType == SW_BAND_TYPE) and( currentMode == AM)) {           
              ssIdxAM = sp[n].stepFreq;
              si4735.setFrequencyStep(ssIdxAM);
              band[bandIdx].currentStep = ssIdxAM;
            }
      
            if (currentMode == FM){
              ssIdxFM = sp[n].stepFreqFM;
              si4735.setFrequencyStep(ssIdxFM);
              band[bandIdx].currentStep = ssIdxFM;
            }
            setStep();
            DrawFila();
          }
        }
      }

      
      if (BroadBand == true) {
          if (CWShift == true)  {    // CW reset
              currentBFO = currentBFO - 700;
              band[bandIdx].lastBFO = currentBFO;
              CWShift = false;
            }
        for (int n = 0 ; n <= lastBroad; n++) {
          if ((x > ((bb[n].Xbbandos) + (bb[n].Xbbandnr))) and (x < ((bb[n].Xbbandos) + (bb[n].Xbbandsr) + (bb[n].Xbbandnr))) and (y > ((bb[n].Ybbandos) + (bb[n].Ybbandnr))) and (y < ((bb[n].Ybbandos) + (bb[n].Ybbandsr) + (bb[n].Ybbandnr)))) {
            Beep(1, 0);
            delay(400);
            BroadBand = false;
            bandIdx = bb[n].BbandNum;
            if ((bandIdx == 0) and (currentAGCgain >= 28)) currentAGCgain = previousAGCgain = 26; // currentAGCgain in FM max. 26
            si4735.setAM();

            delay(50);
            currentBFO = band[bandIdx].lastBFO;
            freqDec = currentBFO; 
            currentMode = band[bandIdx].prefmod;
            ssbLoaded = false;
            //bwIdxAM =  3;
            BandSet();
            DrawFila(); //Draw first layer
          }
        }
      }

      if (HamBand == true) {
         if (CWShift == true)  {     // CW reset
              currentBFO = currentBFO - 700;
              band[bandIdx].lastBFO = currentBFO;
              CWShift = false;
            }
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
            currentBFO = band[bandIdx].lastBFO;
            freqDec = currentBFO; 
            currentMode = band[bandIdx].prefmod;
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
        if (CWShift == true)  {    // CW reset
          currentBFO = currentBFO - 700;
          band[bandIdx].lastBFO = currentBFO;
          CWShift = false;
        }
        spr.createSprite(But_Key_Width-6, But_Key_Height-5);
        spr.fillScreen(COLOR_BACKGROUND);
        for (int n = 0 ; n < 12; n++) { // which keys are pressed?
          if ((x > ((kp[n].Xkeypos) + (kp[n].Xkeypnr))) and (x < ((kp[n].Xkeypos) + (kp[n].Xkeypsr) + (kp[n].Xkeypnr))) and (y > ((kp[n].Ykeypos) + (kp[n].Ykeypnr))) and (y < ((kp[n].Ykeypos) + (kp[n].Ykeypsr) + (kp[n].Ykeypnr)))) {
            if ( n == 11) { // Send button is red
                spr.pushImage(0, 0, But_Key_Width, But_Key_Height, (uint16_t *)But_Key_Green);
            } else {
                spr.pushImage(0, 0, But_Key_Width, But_Key_Height, (uint16_t *)But_Key_Red);
            }
            spr.setTextColor(COLOR_BUTTON_TEXT);
            spr.setTextSize(2);
            spr.setTextDatum(BC_DATUM);
            spr.setTextPadding(0);
            spr.drawString((Keypathtext[kp[n].KeypNum]), (But_Key_Width/2)-2, (But_Key_Height/2)+9);
            spr.pushSprite((kp[n].Xkeypos + kp[n].Xkeypnr + 3), (kp[n].Ykeypos + kp[n].Ykeypnr  + 3));

            Beep(1, 0);
            delay(100);

            if ( n == 11) { // Send button is red
                spr.pushImage(0, 0, But_Key_Width, But_Key_Height, (uint16_t *)But_Key_Red);
            } else {
                spr.pushImage(0, 0, But_Key_Width, But_Key_Height, (uint16_t *)But_Key_Green);
            }
            spr.setTextColor(COLOR_BUTTON_TEXT);
            spr.setTextSize(2);
            spr.setTextDatum(BC_DATUM);
            spr.setTextPadding(0);
            spr.drawString((Keypathtext[kp[n].KeypNum]), (But_Key_Width/2)-2, (But_Key_Height/2)+9);
            spr.pushSprite((kp[n].Xkeypos + kp[n].Xkeypnr + 3), (kp[n].Ykeypos + kp[n].Ykeypnr  + 3));
            spr.deleteSprite();
          
            if ((n >= 0) and (n <= 8)) Freqcalq(n + 1);
            if (n == 10) Freqcalq(0);
            if (n == 9) {
              Decipoint = true;
              fact = 10;
            }          
            if (n == 11) {// SET button
              FREQbut = false;
              Displayfreq = (Displayfreq / 10) + dpfrq;
              if(Displayfreq < 1) {
                tft.setCursor(7, 25);
                tft.print("Freqency < 1 has no function");
                ErrorBeep();
              }else{
                if ((Displayfreq > 30) and (Displayfreq < 87.5 )) {
                  tft.setCursor(7, 25);
                  tft.print("Freqency > 30Mhz and < 87.5 MHz");
                  ErrorBeep();
                }else{
                  if ((Displayfreq >= 108) and (Displayfreq < 153 )) {
                    tft.setCursor(7, 25);
                    tft.print("Freqency >= 108 and < 153 ");
                    ErrorBeep();
                  }else{
                    if (Displayfreq > 30000) Displayfreq = Displayfreq / 1000000;
                    if ((Displayfreq <= 30000) and (Displayfreq >= 153) and (Decipoint == false )) Displayfreq = Displayfreq / 1000;
                    if ((Displayfreq >= 87.5) and (Displayfreq <= 108)) {
                      currentFrequency = Displayfreq * 100;
                      bandIdx = 0;
                      band[bandIdx].currentFreq = currentFrequency;
                    }else{
                      currentFrequency = Displayfreq * 1000;
                      for (int q = 1 ; q <= lastBand; q++) {
                        if (((currentFrequency) >= band[q].minimumFreq) and ((currentFrequency) <= band[q].maximumFreq)) {
                          bandIdx = q; 
                          currentMode = band[q].prefmod;
                          if (((band[bandIdx].bandType == MW_BAND_TYPE) or (band[bandIdx].bandType == LW_BAND_TYPE))and( currentMode == AM)) {           
                            ssIdxMW = band[bandIdx].currentStep;         
                          }
             
                          if ((band[bandIdx].bandType == SW_BAND_TYPE) and( currentMode == AM)) {             
                            ssIdxAM = band[bandIdx].currentStep;
                          }
      
                          if (currentMode == FM){
                            ssIdxFM = band[bandIdx].currentStep;
                          }
                          break;
                        }
                      }
                      delay(100);
                      band[bandIdx].currentFreq = currentFrequency;
                      //currentMode = band[bandIdx].prefmod;
                      freqDec = currentBFO = band[bandIdx].lastBFO = 0;
                    }
                  }
                }
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

          if ((bright == true) and (n != Displbut)) {
            bright = false;
            drawDispbut();
          }

          if (n == SEEKUP) {
            delay(200);
            x = 0;
            y = 0;
            SEEK = true;
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
              si4735.seekStationProgress(SeekFreq, checkStopSeeking,  SEEK_UP); // 1 is up
              delay(300);
              currentFrequency = si4735.getFrequency();
              band[bandIdx].currentFreq = currentFrequency ;
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

              si4735.seekStationProgress(SeekFreq,checkStopSeeking,  SEEK_DOWN);
              delay(300);
              currentFrequency = si4735.getFrequency();
              band[bandIdx].currentFreq = currentFrequency ;
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

          if (n == Displbut) {
            delay(200);
            x = 0;
            y = 0;
            if (bright == false)  {
              bright = true;
              if (bfoOn == true) {
                bfoOn = false;
                drawBFO();
                DrawThla();
              }
              drawDispbut();
              previousBrightness = currentBrightness;
            }
            else {
              bright = false;
              drawDispbut();
            }
          }
         
          if (n == PREV) {
            delay(200);
            x = 0;
            y = 0;
            AGCgainbut = false;
            FirstLayer  = true;
            SecondLayer = false;
            ThirdLayer  = false;
            ForthLayer  = false;
            DrawFila();
          }

          if (n == RDSbut) {
            delay(200);
            x = 0;
            y = 0;
            if (RDS) RDS = false;
            else RDS = true;
             DrawRDSbut();
            //DrawThla();
            //DrawButThla();
          }

         
          if (n == AGCset) {
            delay(200);
            x = 0;
            y = 0;
            if (AGCgainbut) AGCgainbut = false;
            else {
              bfoOn = false; // only AGC function at the rotory encoder
              AGCgainbut = true;
              si4735.getAutomaticGainControl();
              previousAGCgain = 37; // force to setup AGC gain
            }
            FreqDispl();
            DrawThla();
          }

          if (n == ChipType) {
            delay(200);
            x = 0;
            y = 0;
            showFirmwareInformation();
            DrawThla();
          }
        }
      }
    } // end ThirdLayer

    if (ForthLayer) { //===============================================================

    }
  }// end pressed
   
  if (currentMode == LSB || currentMode == USB || currentMode == CW) // set BFO value in si4735
  {
    if ((currentBFO != previousBFO) or (currentBFOmanu != previousBFOmanu))
    {
      previousBFO = currentBFO;
      previousBFOmanu = currentBFOmanu;
      si4735.setSSBBfo(currentBFO+currentBFOmanu);
      if (bfoOn) FreqDispl();
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
    tft.fillRect(XFreqDispl + 6, YFreqDispl + 19 , 228, 48, TFT_BLACK);// Black freq. field
    AGCfreqdisp();
    tft.setTextColor(TFT_YELLOW, TFT_BLACK );
    tft.setTextSize(2);
    tft.setTextDatum(BC_DATUM);

    tft.drawString(String(currentPRES) + ") " + String(((preset[currentPRES].presetIdx) / 100), 1), 60, 51);
    tft.setTextColor(TFT_WHITE, TFT_BLACK );
    tft.drawString(String(preset[currentPRES].PresetName), 175, 51);
    bandIdx = 0;
    si4735.setFrequency((preset[currentPRES].presetIdx));
    band[bandIdx].currentFreq = si4735.getFrequency();
  }


  if (currentVOL != previousVOL)
  {
    currentVOL = currentVOL + (currentVOL - previousVOL);
    tft.setCursor(0, 20);
    if (currentVOL > MaxVOL) currentVOL = MaxVOL;
    if (currentVOL < MinVOL) currentVOL = MinVOL;
    previousVOL = currentVOL;
    si4735.setVolume(currentVOL);
    FreqDispl();
  }

if (currentBrightness != previousBrightness)
  {
    if (currentBrightness < MaxBrightness) currentBrightness = MaxBrightness;
    if (currentBrightness > MinBrightness) currentBrightness = MinBrightness;
    previousBrightness = currentBrightness;
    ledcWrite(LedChannelforTFT, currentBrightness);
  }

  if (currentAGCgain != previousAGCgain)
  {
    AGCgain = 1;
    tft.setCursor(0, 20);
    if (si4735.isCurrentTuneFM())  MaxAGCgain = MaxAGCgainFM;
    else MaxAGCgain = MaxAGCgainAM;

    if (currentAGCgain > MaxAGCgain) currentAGCgain = MaxAGCgain;
    if (currentAGCgain < MinAGCgain) currentAGCgain = MinAGCgain;

    previousAGCgain = currentAGCgain;
    si4735.setAutomaticGainControl(1, currentAGCgain);
    DrawDispl();
    DrawAGCgainbut();
  }


//=======================================================================================
}// end loop
//=======================================================================================




//=======================================================================================
void Dispoff()  {
//=======================================================================================
  if (((millis() - DisplayOnTime) > MIN_ELAPSED_DISPL_TIME * 300) and (DISplay == true)) {
    DISplay = false;
    //digitalWrite(Display_Led, displayoff);
    ledcWrite(LedChannelforTFT, 256);
    PRESbut = false;
    DrawDispl();
    DisplayOnTime = millis();
  }
}

//=======================================================================================
void VOLbutoff()  {
//=======================================================================================
  if (((millis() - VOLbutOnTime) > MIN_ELAPSED_VOLbut_TIME * 30) and (VOLbut == true)) {
    VOLbut = false;
    drawVOL();
    FreqDispl();
  }
  if (VOLbut == false) VOLbutOnTime = millis();
}



//=======================================================================================
void DisplayRDS()  {
//=======================================================================================
  if (( currentMode == FM) and ((FirstLayer) or (ThirdLayer))) {
    if ( currentFrequency != previousFrequency ) {
      previousFrequency = currentFrequency;
      tft.fillRect(XFreqDispl + 60, YFreqDispl + 54, 130, 20, TFT_BLACK);  // clear RDS text
    }
    if ((RDS) and  (NewSNR >= 12)) checkRDS();
    else  tft.fillRect(XFreqDispl + 60, YFreqDispl + 54, 130, 20, TFT_BLACK); // clear RDS text
  }
}

//=======================================================================================
void showtimeRSSI() {
//=======================================================================================
  // Show RSSI status only if this condition has changed
  if ((millis() - elapsedRSSI) > MIN_ELAPSED_RSSI_TIME * RSSIfact) // 150 * 1  = 150 msec refresh time RSSI
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

//=======================================================================================
void showRSSI() {
//=======================================================================================
  if ((  currentMode == FM ) and ((FirstLayer) or (ThirdLayer))) {
    sprintf(buffer, "%s", (si4735.getCurrentPilot()) ? "STEREO" : "MONO");
    tft.setTextColor(TFT_YELLOW, TFT_RED);
    tft.setTextSize(1);
    tft.setTextDatum(BC_DATUM);
    tft.setTextPadding(0);
    tft.fillRect(XFreqDispl + 191, YFreqDispl + 71 , 38, 11, TFT_RED); // STEREO MONO
    tft.drawString(buffer, XFreqDispl + 210, YFreqDispl + 81); 
  }
  rssi = NewRSSI;
  if ((FirstLayer) or (ThirdLayer)) Smeter();
  tft.setTextSize(1);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  if ((FirstLayer) or (ThirdLayer)) {  // dBuV and dB at freq. display
    tft.fillRect(XFreqDispl + 7, YFreqDispl + 75 , 173, 10, TFT_BLACK);
    tft.setTextDatum(TL_DATUM);
    tft.drawString("RSSI = " + String(NewRSSI) + " dBuV" , XFreqDispl + 8, YFreqDispl + 75);
    tft.setTextDatum(TR_DATUM);
    tft.drawString("SNR = " + String(NewSNR) + " dB", XFreqDispl + 180, YFreqDispl + 75);
  }
  VOLbutoff();
}

//=======================================================================================
void encoderCheck()  {
//=======================================================================================
  if (encoderCount != 0)
  {
    if (DISplay == false) { //  Wake-up  Display
      DisplayOnTime = millis();
      //digitalWrite(Display_Led, displayon);
      ledcWrite(LedChannelforTFT, currentBrightness);
      DISplay = true;
    }
    int mainpurp = 1;
     
    if (bfoOn)  {
      #ifdef IhaveSI5351
      currentBFOmanu = 0;
      if (encoderCount == 1) {
        FreqSI5351 = FreqSI5351 + stepsizesynth;
      } else {
        FreqSI5351 = FreqSI5351 - stepsizesynth;
      }
      calibratSI5351 = false; 
      si5351wire.set_freq(FreqSI5351, CLK_Xtal);
      mainpurp = 0;
      #endif
      
      #ifdef IhaveCrystal
      currentBFOmanu = (encoderCount == 1) ? (currentBFOmanu + currentBFOStep) : (currentBFOmanu - currentBFOStep);
      mainpurp = 0;
      #endif
      }
    
    if (PRESbut) {     // FM preset
      currentPRES = (encoderCount == 1) ? (currentPRES + currentPRESStep) : (currentPRES - currentPRESStep);
      mainpurp = 0;
    }

    if (VOLbut) {     // Volume control
      currentVOL = (encoderCount == 1) ? (currentVOL + currentVOLStep) : (currentVOL - currentVOLStep);
      mainpurp = 0;
    }

    if (AGCgainbut) {     // AGC gain control
      currentAGCgain = (encoderCount == 1) ? (currentAGCgain + currentAGCgainStep) : (currentAGCgain - currentAGCgainStep);
      mainpurp = 0;
    }

    if (bright) {     // Brightness control
      currentBrightness = (encoderCount == 1) ? (currentBrightness - 20) : (currentBrightness + 20);
      mainpurp = 0;
    }

    if (mainpurp == 1)
    {
   
      if (currentMode == LSB || currentMode == USB || currentMode == CW) {
        if (encoderCount == 1) {              
          freqDec = freqDec - freqstep;
          int freqTot = (si4735.getFrequency() * 1000) + (freqDec * -1);
          if ( freqTot > (band[bandIdx].maximumFreq * 1000)) {
            si4735.setFrequency(band[bandIdx].maximumFreq);
            freqDec = 0;
          }  
          if (freqDec <= -16000)  {
            freqDec = freqDec + 16000;
            int freqPlus16 = currentFrequency + 16; //trunc(freqDec/1000);
            si4735.setHardwareAudioMute(1);
            AudioMut = true;
            elapsedAudMut = millis();
            si4735.setFrequency(freqPlus16);
          } 
          currentBFO = freqDec;
        } else {   
          freqDec = freqDec + freqstep;
          int freqTot = (si4735.getFrequency() * 1000) - freqDec;
          if ( freqTot < (band[bandIdx].minimumFreq * 1000)) {
            si4735.setFrequency(band[bandIdx].minimumFreq);
            freqDec = 0;
          }
          if (freqDec >= 16000)  {
            freqDec = freqDec - 16000;
            int freqMin16 = currentFrequency - 16;
            si4735.setHardwareAudioMute(1);
            AudioMut = true;
            elapsedAudMut = millis();
            si4735.setFrequency(freqMin16);
          }
          currentBFO = freqDec;
        }
      band[bandIdx].lastBFO = currentBFO; 
      } else  {
        if (encoderCount == 1) {
          si4735.frequencyUp();
        } else {
          si4735.frequencyDown();
        }
      }
      band[bandIdx].currentFreq = si4735.getFrequency();
      //band[bandIdx].lastBFO = currentBFO;
    }
    FreqDispl();
    encoderCount = 0;
  }
}

//=======================================================================================
void encoderButtonCheck()  {
//=======================================================================================
  //Encoder button
  if (analogRead(ENCODER_SWITCH) < 500) {
    Beep(1, 0);
    //delay(300);
    int mainpurp = 1;
    if (DISplay == false) { //  Wake-up  Display
      DisplayOnTime = millis();
      //digitalWrite(Display_Led, displayon);
      ledcWrite(LedChannelforTFT, currentBrightness);
      DISplay = true;
      mainpurp = 0;
    }
    if (PRESbut == true) {// FM preset selection
      PRESbut = false;
      DrawDispl();
      mainpurp = 0;
    }

    if (bright) {
     bright = false;
     drawDispbut();
     mainpurp = 0;
    }
    
    #ifdef IhaveSI5351
    if (FirstLayer) {
      if ((bfoOn) and (calibratSI5351 == false)) {
        calibratSI5351 = true;
        calibratvalSI5351 = 3276800 - FreqSI5351;
        FreqDispl(); 
        mainpurp = 0; 
      } 
    }   
      #endif

    if (mainpurp == 1) {     
      if (VOLbut)  {
        VOLbut = false;
        drawVOL();
        FreqDispl();
        while (analogRead(ENCODER_SWITCH) < 500) { 
        }
      }
      else {
        VOLbut = true;
        if (bfoOn == true) {
          bfoOn = false;
          drawBFO();
        }
        drawVOL();
        FreqDispl();
      }
    }
    while (analogRead(ENCODER_SWITCH) < 500) { 
    } 
  }  
}
//=======================================================================================
void setStep()  {
//=======================================================================================
  // This command should work only for SSB mode
  if (bfoOn && (currentMode == LSB || currentMode == USB || currentMode == CW))
  {
    currentBFOStep = (currentBFOStep == 25) ? 10 : 25;
  }
  useBand();
  DrawDispl();
}

//=======================================================================================
void Beep(int cnt, int tlb) {
//=======================================================================================
  int tla = 100;
  for (int i = 0; i < cnt; i++) {
    digitalWrite(BEEPER, beepOn);
    delay(tla);
    digitalWrite(BEEPER, beepOff);
    delay(tlb);
  }
}

//=======================================================================================
void DrawFila()   {// Draw of first layer
//=======================================================================================
  FirstLayer = true;
  SecondLayer = false;
  tft.fillScreen(TFT_BLACK);
  DrawButFila();
  DrawDispl();
  DrawSmeter();
  DrawVolumeIndicator();
}

//=======================================================================================
void DrawThla()  {  // Draw of Third layer
//=======================================================================================
  ThirdLayer = true;
  ForthLayer = false;
  tft.fillScreen(TFT_BLACK);
  DrawButThla();
  DrawDispl();
  DrawSmeter();
  DrawVolumeIndicator();
  DrawRDSbut();
  DrawAGCgainbut();
}

//=======================================================================================
void DrawButFila() { // Buttons first layer
//=======================================================================================
  spr.createSprite(Xbutsiz,Ybutsiz);
  for (int n = 0 ; n <= lastButton; n++) {
  
  spr.fillScreen(TFT_BLACK);
     spr.pushImage(0, 0, But_Width, But_Height, (uint16_t *)But_Blue);
     spr.setTextColor(COLOR_BUTTON_TEXT);
     spr.setTextSize(2);
     spr.setTextDatum(BC_DATUM);
     spr.setTextPadding(0);
     spr.drawString((bt[n].ButtonNam),(Xbutsiz/2)+2, (Ybutsiz/2)+9);
     spr.pushSprite(bt[bt[n].ButtonNum].XButos + Xbutst, bt[bt[n].ButtonNum].YButos + Ybutst);
  }
     spr.deleteSprite();
  drawBFO();
  drawAGC();
  
}

//=======================================================================================
void DrawButThla() { // Buttons Third layer
//=======================================================================================
  spr.createSprite(Xbutsiz,Ybutsiz);
  for (int n = 0 ; n <= lastButton; n++) {
     spr.fillScreen(COLOR_BACKGROUND);
     spr.pushImage(0, 0, But_Width, But_Height, (uint16_t *)But_Blue);
     spr.setTextColor(COLOR_BUTTON_TEXT);
     spr.setTextSize(2);
     spr.setTextDatum(BC_DATUM);
     spr.setTextPadding(0);
     spr.drawString((bt[n].ButtonNam1),(Xbutsiz/2)+2, (Ybutsiz/2)+9);
     spr.pushSprite(bt[bt[n].ButtonNum1].XButos + Xbutst, bt[bt[n].ButtonNum1].YButos + Ybutst);
  }
     spr.deleteSprite();
  DrawRDSbut();
}


//=======================================================================================
void DrawVolumeIndicator()  {
//=======================================================================================
  tft.setTextSize(1);
  //tft.fillRect(XVolInd, YVolInd, 240, 30, TFT_GREY);
  tft.fillRect(XVolInd + 5, YVolInd + 5, 230, 20, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(XVolInd +  11, YVolInd + 7);
  tft.print("0%");
  tft.setCursor(XVolInd + 116, YVolInd + 7);
  tft.print("50%");
  tft.setCursor(XVolInd + 210, YVolInd + 7);
  tft.print("100%");
}

//=======================================================================================
void DrawSmeter()  {
//=======================================================================================
  String IStr;
  tft.setTextSize(1);
  //tft.fillRect(Xsmtr, Ysmtr, 240, 55, TFT_GREY);
  tft.fillRect(Xsmtr + 5, Ysmtr + 5, 230, 45, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(BC_DATUM);
  for (int i = 0; i < 10; i++) {
    tft.fillRect(Xsmtr + 15 + (i * 12), Ysmtr + 24, 4, 8, TFT_YELLOW);
    IStr = String(i);
    tft.setCursor((Xsmtr + 14 + (i * 12)), Ysmtr + 13);
    tft.print(i);
  }
  for (int i = 1; i < 7; i++) {
    tft.fillRect((Xsmtr + 123 + (i * 16)), Ysmtr + 24, 4, 8, TFT_RED);
    IStr = String(i * 10);
    tft.setCursor((Xsmtr + 117 + (i * 16)), Ysmtr + 13);
    if ((i == 2) or (i == 4) or (i == 6))  {
      tft.print("+");
      tft.print(i * 10);
    }
  }
  tft.fillRect(Xsmtr + 15, Ysmtr + 32 , 112, 4, TFT_YELLOW);
  tft.fillRect(Xsmtr + 127, Ysmtr + 32 , 100, 4, TFT_RED);
  // end Smeter
}

//=======================================================================================
void drawDispbut()   {
//=======================================================================================
  spr.createSprite(Xbutsiz,Ybutsiz);
  spr.fillScreen(COLOR_BACKGROUND);
  if (bright) {
    spr.pushImage(0, 0, But_Width, But_Height, (uint16_t *)But_Amber);
  } else {
    spr.pushImage(0, 0, But_Width, But_Height, (uint16_t *)But_Blue);
  }
  spr.setTextColor(COLOR_BUTTON_TEXT);
  spr.setTextSize(2);
  spr.setTextDatum(BC_DATUM);
  spr.setTextPadding(0);
  spr.drawString((bt[6].ButtonNam1), (Xbutsiz/2)+2, (Ybutsiz/2)+9);
  spr.pushSprite(bt[bt[6].ButtonNum1].XButos + Xbutst, bt[bt[6].ButtonNum1].YButos + Ybutst);
  spr.deleteSprite();
}

//=======================================================================================
void drawVOL()   {
//=======================================================================================
  spr.createSprite(Xbutsiz,Ybutsiz);
  spr.fillScreen(COLOR_BACKGROUND);
  if (VOLbut) {
    spr.pushImage(0, 0, But_Width, But_Height, (uint16_t *)But_Amber);
  } else {
    spr.pushImage(0, 0, But_Width, But_Height, (uint16_t *)But_Blue);
  }
  spr.setTextColor(COLOR_BUTTON_TEXT);
  spr.setTextSize(2);
  spr.setTextDatum(BC_DATUM);
  spr.setTextPadding(0);
  spr.drawString((bt[5].ButtonNam), (Xbutsiz/2)+2, (Ybutsiz/2)+9);
  spr.pushSprite(bt[bt[5].ButtonNum].XButos + Xbutst, bt[bt[5].ButtonNum].YButos + Ybutst);
  spr.deleteSprite();
}

//=======================================================================================
void DrawAGCgainbut()  {
//=======================================================================================
  if (ThirdLayer)  {  
  spr.createSprite(Xbutsiz,Ybutsiz);
  spr.fillScreen(COLOR_BACKGROUND);
  if (AGCgainbut) {
    AGCgainbuttext = currentAGCgain;
    spr.pushImage(0, 0, But_Width, But_Height, (uint16_t *)But_Red);
  }else { 
    AGCgainbuttext = currentAGCgain;
    spr.pushImage(0, 0, But_Width, But_Height, (uint16_t *)But_Blue);
  }
  spr.setTextColor(COLOR_BUTTON_TEXT);
  spr.setTextSize(2);
  spr.setTextDatum(BC_DATUM);
  spr.setTextPadding(0);
  spr.drawString("RF-att", (Xbutsiz/2)+2, (Ybutsiz/2)+1);
  spr.drawString(AGCgainbuttext, (Xbutsiz/2)+2, (Ybutsiz/2)+16);
  spr.pushSprite(bt[bt[4].ButtonNum1].XButos + Xbutst, bt[bt[4].ButtonNum1].YButos + Ybutst);
  spr.deleteSprite();
 }
}


//=======================================================================================
void DrawRDSbut()  {
//=======================================================================================
  spr.createSprite(Xbutsiz,Ybutsiz);
  spr.fillScreen(COLOR_BACKGROUND);
  if (RDS) {
    RDSbuttext = "ON";
    spr.pushImage(0, 0, But_Width, But_Height, (uint16_t *)But_Red);
  } else {
    RDSbuttext = "OFF";
    spr.pushImage(0, 0, But_Width, But_Height, (uint16_t *)But_Blue);
  }
  spr.setTextColor(COLOR_BUTTON_TEXT);
  spr.setTextSize(2);
  spr.setTextDatum(BC_DATUM);
  spr.setTextPadding(0);
  spr.drawString((bt[3].ButtonNam1), (Xbutsiz/2)+2, (Ybutsiz/2)+12);
  spr.pushSprite(bt[bt[3].ButtonNum1].XButos + Xbutst, bt[bt[3].ButtonNum1].YButos + Ybutst);
  spr.deleteSprite();
}
//=======================================================================================
void drawMUTE()  {
//=======================================================================================
  spr.createSprite(Xbutsiz,Ybutsiz);
  spr.fillScreen(COLOR_BACKGROUND);
  if (Mutestat) {
    si4735.setAudioMute(audioMuteOn);
    spr.pushImage(0, 0, But_Width, But_Height, (uint16_t *)But_Amber);
  } else {
    si4735.setAudioMute(audioMuteOff);
     spr.pushImage(0, 0, But_Width, But_Height, (uint16_t *)But_Blue);
  }
  spr.setTextColor(COLOR_BUTTON_TEXT);
  spr.setTextSize(2);
  spr.setTextDatum(BC_DATUM);
  spr.setTextPadding(0);
  spr.drawString((bt[4].ButtonNam), (Xbutsiz/2)+2, (Ybutsiz/2)+9);
  spr.pushSprite(bt[bt[4].ButtonNum].XButos + Xbutst, bt[bt[4].ButtonNum].YButos + Ybutst);
  spr.deleteSprite();
}
//=======================================================================================
void drawAGC()  {
//=======================================================================================
  si4735.getAutomaticGainControl();
  spr.createSprite(Xbutsiz,Ybutsiz);
  spr.fillScreen(COLOR_BACKGROUND);
  if (si4735.isAgcEnabled()) {
     spr.pushImage(0, 0, But_Width, But_Height, (uint16_t *)But_Red);
  } else {
     spr.pushImage(0, 0, But_Width, But_Height, (uint16_t *)But_Blue);
  }
  spr.setTextColor(COLOR_BUTTON_TEXT);
  spr.setTextSize(2);
  spr.setTextDatum(BC_DATUM);
  spr.setTextPadding(0);
  spr.drawString((bt[3].ButtonNam), (Xbutsiz/2)+2, (Ybutsiz/2)+9);
  spr.pushSprite(bt[bt[3].ButtonNum].XButos + Xbutst, bt[bt[3].ButtonNum].YButos + Ybutst);
  spr.deleteSprite();
}

//=======================================================================================
void drawBFO ()  {
//=======================================================================================
  spr.createSprite(Xbutsiz,Ybutsiz);
  spr.fillScreen(COLOR_BACKGROUND);
  if (bfoOn) {
     spr.pushImage(0, 0, But_Width, But_Height, (uint16_t *)But_Red);
  } else {
     spr.pushImage(0, 0, But_Width, But_Height, (uint16_t *)But_Blue);
  }
  spr.setTextColor(COLOR_BUTTON_TEXT);
  spr.setTextSize(2);
  spr.setTextDatum(BC_DATUM);
  spr.setTextPadding(0);
  spr.drawString((bt[1].ButtonNam), (Xbutsiz/2)+2, (Ybutsiz/2)+9);
  spr.pushSprite(bt[bt[1].ButtonNum].XButos + Xbutst, bt[bt[1].ButtonNum].YButos + Ybutst);
  spr.deleteSprite();
}

//=======================================================================================
void drawKeyPath() {
//=======================================================================================
  tft.fillScreen(TFT_BLACK);
  spr.createSprite(But_Key_Width-6, But_Key_Height-5);
  spr.fillScreen(COLOR_BACKGROUND);

  for (int n = 0 ; n <= lastKPath; n++) {
    if ( n == 11) { // Send button is red
      spr.pushImage(0, 0, But_Key_Width, But_Key_Height, (uint16_t *)But_Key_Red);
    } else {
      spr.pushImage(0, 0, But_Key_Width, But_Key_Height, (uint16_t *)But_Key_Green);
    }

    spr.setTextColor(COLOR_BUTTON_TEXT);
    spr.setTextSize(2);
    spr.setTextDatum(BC_DATUM);
    spr.setTextPadding(0);
    spr.drawString((Keypathtext[kp[n].KeypNum]), (But_Key_Width/2)-2, (But_Key_Height/2)+9);
    spr.pushSprite((kp[n].Xkeypos + kp[n].Xkeypnr + 3), (kp[n].Ykeypos + kp[n].Ykeypnr  + 3));
  }
  spr.deleteSprite();
}
//=======================================================================================
void HamBandlist() {
//=======================================================================================
 
  tft.fillScreen(COLOR_BACKGROUND);  ////
  spr.createSprite(But_Mode_Width, But_Mode_Height);
  for (int n = 0 ; n <= lastHam; n++) {
     spr.fillScreen(COLOR_BACKGROUND);
     spr.pushImage(0, 0, But_Mode_Width, But_Mode_Height, (uint16_t *)But_Mode_Green);
     spr.setTextColor(COLOR_PANEL_TEXT);
     spr.setTextSize(2);
     spr.setTextDatum(BC_DATUM);
     spr.setTextPadding(0);
     spr.drawString(band[bn[n].BandNum].bandName, But_Mode_Width/2, But_Mode_Height-2);
     spr.pushSprite((bn[n].Xbandos + bn[n].Xbandnr + 3) , (bn[n].Ybandos) + (bn[n].Ybandnr + 3));
  }
     spr.deleteSprite();
}


//=======================================================================================
void BroadBandlist() {
//=======================================================================================
  tft.fillScreen(COLOR_BACKGROUND); ////
  spr.createSprite(But_Band_Width, But_Band_Height);
  for (int n = 0 ; n <= lastBroad; n++) {
     spr.fillScreen(COLOR_BACKGROUND);
     if (n==0)  spr.pushImage(0, 0, But_Band_Width, But_Band_Height, (uint16_t *)But_Band_Red);
     else if (n==1)  spr.pushImage(0, 0, But_Band_Width, But_Band_Height, (uint16_t *)But_Band_Amber);
     else if (n==2)  spr.pushImage(0, 0, But_Band_Width, But_Band_Height, (uint16_t *)But_Band_Blue);
     else  spr.pushImage(0, 0, But_Band_Width, But_Band_Height, (uint16_t *)But_Band_Green);
     spr.setTextColor(COLOR_PANEL_TEXT);
     spr.setTextSize(2);
     spr.setTextDatum(BC_DATUM);
     spr.setTextPadding(0);
     spr.drawString(band[bb[n].BbandNum].bandName, But_Band_Width/2, But_Band_Height-2);
     spr.pushSprite((bb[n].Xbbandos + bb[n].Xbbandnr + 3) , (bb[n].Ybbandos) + (bb[n].Ybbandnr + 3));
  }
     spr.deleteSprite();
}

//=======================================================================================
void Steplist() {
//=======================================================================================
  tft.fillScreen(COLOR_BACKGROUND);
  spr.createSprite(But_Mode_Width, But_Mode_Height);
  if ( currentMode != FM) {
    for (int n = 0 ; n <= lastStep; n++) {
     spr.fillScreen(COLOR_BACKGROUND);
     spr.pushImage(0, 0, But_Mode_Width, But_Mode_Height, (uint16_t *)But_Mode_Green);
     spr.setTextColor(COLOR_PANEL_TEXT);
     spr.setTextSize(2);
     spr.setTextDatum(BC_DATUM);
     spr.setTextPadding(0);
     spr.drawString(String(sp[n].stepFreq) + "kHz", But_Mode_Width/2, But_Mode_Height-2);
     spr.pushSprite((sp[n].Xstepos) + 5, sp[n].Ystepos + 5 + sp[n].Ystepnr);
    }
  }else {
      for (int n = 0 ; n <= 1; n++) {
        spr.fillScreen(COLOR_BACKGROUND);
        spr.pushImage(0, 0, But_Mode_Width, But_Mode_Height, (uint16_t *)But_Mode_Green);
        spr.setTextColor(COLOR_PANEL_TEXT);
        spr.setTextSize(2);
        spr.setTextDatum(BC_DATUM);
        spr.setTextPadding(0);
        spr.drawString(String((sp[n].stepFreqFM)*10,0) + "kHz", But_Mode_Width/2, But_Mode_Height-2);
        spr.pushSprite((sp[n].Xstepos) + 5, sp[n].Ystepos + 5 + sp[n].Ystepnr);
      }
    }  
     spr.deleteSprite();
}

//=======================================================================================
void Modelist() {
//=======================================================================================
  
  tft.fillScreen(COLOR_BACKGROUND);
  spr.createSprite(But_Mode_Width, But_Mode_Height);
  for (int n = 1 ; n <= lastMod; n++) {
     spr.fillScreen(COLOR_BACKGROUND);
     spr.pushImage(0, 0, But_Mode_Width, But_Mode_Height, (uint16_t *)But_Mode_Green);
     spr.setTextColor(COLOR_PANEL_TEXT);
     spr.setTextSize(2);
     spr.setTextDatum(BC_DATUM);
     spr.setTextPadding(0);
     spr.drawString(bandModeDesc[md[n].Modenum], But_Mode_Width/2, But_Mode_Height-2);
     spr.pushSprite((md[n].Xmodos + 5), md[n].Ymodos + 5 + (md[n].Ymodnr));
  }
     spr.deleteSprite();
}

//=======================================================================================
void BWList()  {
//=======================================================================================
  tft.fillScreen(COLOR_BACKGROUND);  ////
  if ( currentMode == AM) nrbox = 7;
  if ( currentMode == FM) nrbox = 5;
  if ((currentMode == LSB) or ( currentMode == USB) or ( currentMode == CW)) nrbox = 6;
  spr.createSprite(But_Mode_Width, But_Mode_Height);
  for (int n = 0 ; n < nrbox; n++) {
     spr.fillScreen(COLOR_BACKGROUND);
     spr.pushImage(0, 0, But_Mode_Width, But_Mode_Height, (uint16_t *)But_Mode_Green);
     spr.setTextColor(COLOR_PANEL_TEXT);
     spr.setTextSize(2);
     spr.setTextDatum(BC_DATUM);
     spr.setTextPadding(0);
     if ( currentMode == AM) spr.drawString(bandwidthAM[bw[n].BandWidthAM], But_Mode_Width/2, But_Mode_Height-2);
     if ( currentMode == FM) spr.drawString(bandwidthFM[bw[n].BandWidthFM], But_Mode_Width/2, But_Mode_Height-2);
     if ((currentMode == LSB) or ( currentMode == USB) or ( CWShift == true)) spr.drawString(bandwidthSSB[bw[n].BandWidthSSB], But_Mode_Width/2, But_Mode_Height-2);
     spr.pushSprite((bw[n].Xos + 3), bw[n].Yos + 3 + (bw[n].Ynr));
  }
     spr.deleteSprite();

  tft.setTextColor(COLOR_BUTTON_TEXT, COLOR_BACKGROUND);
  tft.setTextSize(2);
  tft.setTextDatum(BC_DATUM);
  tft.setTextPadding(0);
  if ( currentMode == AM)  tft.drawString("AM Filter in kHz"  , XfBW + 50, YfBW );
  if (( currentMode == USB) and (CWShift == false)) tft.drawString("USB Filter in kHz" , XfBW + 50, YfBW );  
  if ( currentMode == LSB) tft.drawString("LSB Filter in kHz" , XfBW + 50, YfBW ); 
  if ( currentMode == FM)  tft.drawString("FM Filter in kHz"  , XfBW + 50, YfBW );
  if (( currentMode == USB) and (CWShift ==  true)) tft.drawString("CW Filter in kHz"  , XfBW + 50, YfBW ); 
}


//=======================================================================================
void subrstatus() {
//=======================================================================================
  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(TL_DATUM);
  tft.setTextSize(1);
  tft.setCursor(0, 0);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.drawString("Software version V3.3",5,10);
  tft.drawString("Mod.     : " + String(bandModeDesc[band[bandIdx].prefmod]), 5, 20);
  if ( currentMode != FM)  tft.drawString("Freq.    : " + String(currentFrequency, 0) + " KHz", 5, 30);
  else tft.drawString("Freq.    : " + String(currentFrequency / 100, 1) + " MHz", 5, 30);
  si4735.getCurrentReceivedSignalQuality();
  tft.drawString("RSSI     : " + String(si4735.getCurrentRSSI()) + "dBuV", 5, 40); // si4735.getCurrentSNR()
  tft.drawString("SNR      : " + String(si4735.getCurrentSNR()) + "uV", 5, 50);
  if (  currentMode == FM ) {
    sprintf(buffer, "%s", (si4735.getCurrentPilot()) ? "STEREO" : "MONO");
    tft.drawString("         : " + String(buffer), 5, 60);
  }
  si4735.getAutomaticGainControl();
  si4735.getCurrentReceivedSignalQuality();
  tft.drawString("LNA GAIN index: " + String(si4735.getAgcGainIndex()) + "/" + String(currentAGCAtt), 5, 70);
  tft.drawString("Volume   : )" + String(si4735.getVolume()), 5, 80);
  sprintf(buffer, "%s", (si4735.isAgcEnabled()) ? "AGC ON " : "AGC OFF");
  tft.drawString(buffer, 5, 90);
  if (bfoOn) tft.drawString("BFO ON  ", 5,100);
  else tft.drawString("BFO OFF ", 5,100);
  tft.drawString("AVC max GAIN  : " + String(si4735.getCurrentAvcAmMaxGain()), 5, 110);
  tft.drawString("Ant. Cap = " + String(si4735.getAntennaTuningCapacitor()) , 5, 120);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("Band          :  " + String(bandIdx) + "  " + String(band[bandIdx].bandName) , 5, 140);
  tft.drawString("Bandwidth SSB : " + String(bandwidthSSB[bwIdxSSB]) + " KHz", 5, 150);
  tft.drawString("Bandwidth AM  : " + String(bandwidthAM[bwIdxAM]) + " KHz", 5, 160);
  tft.drawString("Bandwidth FM  : " + String(bandwidthFM[bwIdxFM]) + " KHz", 5, 170);
  tft.drawString("Stepsize  MW  :   " + String(ssIdxMW) + " KHz", 5, 180);
  tft.drawString("Stepsize  AM  :   " + String(ssIdxAM) + " KHz", 5, 190);
  tft.drawString("Stepsize SSB  :   " "1 KHz fixed", 5, 200);
  tft.drawString("Stepsize  FM  : " + String(ssIdxFM * 10) + " KHz", 5, 210);
  int vsupply;
  vsupply = analogRead(ENCODER_SWITCH);
  tft.drawString("Power Supply  : " + String(((1.66 / 1850)*vsupply) * 2) + " V.", 5, 220);
  while (x == 0) {
    presStat = tft.getTouch(&x, &y);
  }
  x = y = 0;
  Beep(1, 0);
  delay(400);
}

//=======================================================================================
void showRDSStation() {
//=======================================================================================
  if ((FirstLayer) or (ThirdLayer)) {
    tft.setCursor(XFreqDispl + 75, YFreqDispl + 57);
    tft.print(stationName);
  }
  delay(250);
}



//=======================================================================================
void checkRDS() {
//=======================================================================================
  si4735.getRdsStatus();
  if (si4735.getRdsReceived()) {
    if (si4735.getRdsSync() && si4735.getRdsSyncFound() ) {
      stationName = si4735.getRdsText0A();
      tft.setTextSize(2);
      tft.setTextColor(TFT_VIOLET, TFT_BLACK);
      tft.setTextDatum(BC_DATUM);
      if ( stationName != NULL )   showRDSStation();
    }
  }
}

//=======================================================================================
void FreqDispl() {
//=======================================================================================
  if ((FirstLayer) or (ThirdLayer)) {
    currentFrequency = si4735.getFrequency();
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.fillRect( XFreqDispl + 6, YFreqDispl + 19 , 228, 48, TFT_BLACK); // Black freq. field
    AGCfreqdisp();
    BFOStepdisp();
    tft.setTextSize(4);
    tft.setTextDatum(BC_DATUM);
    if ((VOLbut) or (AGCgainbut)) {
      if (VOLbut) {
        tft.setTextSize(3);
        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
        tft.drawString(String(map(currentVOL, 20, 63, 0, 100)), XFreqDispl + 60, YFreqDispl + 53);
        tft.setTextSize(2);
        tft.drawString( "Volume", XFreqDispl + 160, YFreqDispl + 53);
      }
      if (AGCgainbut) {
        tft.setTextSize(2);
        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
        tft.drawString(String(currentAGCgain), XFreqDispl + 40, YFreqDispl + 53);
        tft.setTextSize(2);
        tft.drawString("RF Attenuation", XFreqDispl + 150, YFreqDispl + 53);
      }

    } else {
      if ((band[bandIdx].bandType == MW_BAND_TYPE || band[bandIdx].bandType == LW_BAND_TYPE) and (bfoOn == false)) {
          Displayfreq =  currentFrequency;
          tft.setTextSize(1);
          tft.setTextColor(COLOR_INDICATOR_FREQ, COLOR_BACKGROUND);
          tft.setFreeFont(&DSEG7_Classic_Mini_Regular_34);
          tft.drawString(String(Displayfreq, 0), XFreqDispl + 120, YFreqDispl + 61);
          tft.setTextSize(2);
          tft.setTextColor(TFT_YELLOW, TFT_BLACK);
          tft.setFreeFont(NULL);      
          tft.drawString("KHz", XFreqDispl + 215, YFreqDispl + 61);
      /*  } */
      }
      if (band[bandIdx].bandType == FM_BAND_TYPE) {
        Displayfreq =  currentFrequency / 100;
        tft.setTextSize(1);
        tft.setTextColor(COLOR_INDICATOR_FREQ, COLOR_BACKGROUND);
        tft.setFreeFont(&DSEG7_Classic_Mini_Regular_34);
        tft.drawString(String(Displayfreq, 2), XFreqDispl + 120, YFreqDispl + 55);
        tft.setTextSize(2);
        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
        tft.setFreeFont(NULL); 
        tft.drawString("MHz", XFreqDispl + 215, YFreqDispl + 54);
      }
      if ((currentMode == AM) and (band[bandIdx].bandType != MW_BAND_TYPE) and (band[bandIdx].bandType != LW_BAND_TYPE)) {
        Displayfreq =  currentFrequency / 1000;
        tft.setTextSize(1);
        tft.setTextColor(COLOR_INDICATOR_FREQ, COLOR_BACKGROUND);
        tft.setTextDatum(BC_DATUM);
        tft.setFreeFont(&DSEG7_Classic_Mini_Regular_34);
        tft.drawString(String(Displayfreq, 3), XFreqDispl + 120, YFreqDispl + 61);
        tft.setTextSize(2);
        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
        tft.setFreeFont(NULL); 
        tft.drawString("MHz", XFreqDispl + 215, YFreqDispl + 61);
      }
      if (currentMode == LSB || currentMode == USB  || currentMode == CW ) {
        if (bfoOn) {
          tft.setTextSize(1);
          tft.setTextColor(COLOR_INDICATOR_FREQ, COLOR_BACKGROUND);
          tft.setTextDatum(BR_DATUM);
          tft.setFreeFont(&DSEG7_Classic_Mini_Regular_34);
          #ifdef IhaveCrystal
            tft.drawString(String(currentBFOmanu), XFreqDispl + 180, YFreqDispl + 61);
          #endif
          #ifdef IhaveSI5351
            float temp = FreqSI5351 + calibratvalSI5351;
            tft.drawString(String((temp/100),2), XFreqDispl + 220, YFreqDispl + 61);
          #endif
          tft.setFreeFont(NULL);
          tft.setTextSize(2);
          tft.setTextColor(TFT_YELLOW, TFT_BLACK);
          tft.drawString("Hz", XFreqDispl + 229, YFreqDispl + 84);
          #ifdef IhaveSI5351
          if (stepsizesynth == 10)  tft.fillRect(XFreqDispl + 170, YFreqDispl + 62, 20, 5, TFT_ORANGE);
          if (stepsizesynth ==  1)  tft.fillRect(XFreqDispl + 198, YFreqDispl + 62, 20, 5, TFT_ORANGE);
          #endif
          tft.setTextDatum(BC_DATUM);   
        }
        else {
          tft.fillRect( XFreqDispl + 6, YFreqDispl + 26 , 228, 45, TFT_BLACK); // Black freq. field
          Displayfreq = (currentFrequency*1000) - (band[bandIdx].lastBFO);
          if (CWShift) Displayfreq = Displayfreq + 700;
          int mhz = trunc(Displayfreq/1000000);
          int khz = Displayfreq-(mhz * 1000000);
          khz = trunc(khz/1000);
          int hz = Displayfreq-(mhz * 1000000)- (khz * 1000);
          char s[12] = {'\0'};
          sprintf(s,"%i.%03i.%02i",mhz,khz,hz/10);
          //tft.setTextSize(2);
          tft.setTextColor(TFT_YELLOW, TFT_BLACK);
          tft.setFreeFont(NULL); 
          tft.setTextSize(1);
          tft.setTextColor(COLOR_INDICATOR_FREQ, COLOR_BACKGROUND);
          tft.setTextDatum(BR_DATUM);
          tft.setFreeFont(&DSEG7_Classic_Mini_Regular_34);
          tft.drawString(String(s), XFreqDispl + 220, YFreqDispl + 61);
          tft.setTextSize(2);
          tft.setTextColor(TFT_YELLOW, TFT_BLACK);
          tft.setFreeFont(NULL); 
          tft.drawString("MHz", XFreqDispl + 229, YFreqDispl + 84);
          if (freqstepnr == 0)  tft.fillRect(XFreqDispl + 140, YFreqDispl + 62, 20, 5, TFT_ORANGE);
          if (freqstepnr == 1)  tft.fillRect(XFreqDispl + 170, YFreqDispl + 62, 20, 5, TFT_ORANGE);
          if (freqstepnr == 2)  tft.fillRect(XFreqDispl + 198, YFreqDispl + 62, 20, 5, TFT_ORANGE);
          tft.setTextDatum(BC_DATUM);
          tft.setTextColor(TFT_YELLOW, TFT_BLACK);
        }
      }
    }
  }
}

/*
 * Checks the stop seeking criterias.  
 * Returns true if the user press the touch or rotates the encoder. 
 */
bool checkStopSeeking() {
  // Checks the touch and encoder
  return (bool) encoderCount || tft.getTouch(&x, &y);   // returns true if the user rotates the encoder or touches on screen
} 

//=======================================================================================
void SeekFreq (uint16_t freq)  {
//=======================================================================================
  if ((FirstLayer) or (ThirdLayer))  {
    currentFrequency = freq;
    tft.setTextSize(4);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.setTextDatum(BC_DATUM);
    tft.setTextPadding(0);
    tft.fillRect( XFreqDispl + 6, YFreqDispl + 28 , 228, 32, TFT_BLACK); // Black freq. field
    if (band[bandIdx].bandType == MW_BAND_TYPE || band[bandIdx].bandType == LW_BAND_TYPE) {
      Displayfreq =  currentFrequency;
      tft.setTextSize(1);
      tft.setTextColor(COLOR_INDICATOR_FREQ, COLOR_BACKGROUND);
      tft.setFreeFont(&DSEG7_Classic_Mini_Regular_34);
      tft.drawString(String(Displayfreq, 0), XFreqDispl + 120, YFreqDispl + 61);
      tft.setTextSize(2);
      tft.setTextColor(TFT_YELLOW, TFT_BLACK);
      tft.setFreeFont(NULL);      
      tft.drawString("KHz", XFreqDispl + 215, YFreqDispl + 61);
    }
    if (band[bandIdx].bandType == FM_BAND_TYPE) {
      Displayfreq =  currentFrequency / 100;
      tft.setTextSize(1);
      tft.setTextColor(COLOR_INDICATOR_FREQ, COLOR_BACKGROUND);
      tft.setFreeFont(&DSEG7_Classic_Mini_Regular_34);
      tft.drawString(String(Displayfreq, 2), XFreqDispl + 120, YFreqDispl + 54);
      tft.setTextSize(2);
      tft.setTextColor(TFT_YELLOW, TFT_BLACK);
      tft.setFreeFont(NULL); 
      tft.drawString("MHz", XFreqDispl + 215, YFreqDispl + 54);
    }
    if (band[bandIdx].bandType == SW_BAND_TYPE)  {   
      Displayfreq =  currentFrequency / 1000;
      tft.setTextSize(1);
      tft.setTextColor(COLOR_INDICATOR_FREQ, COLOR_BACKGROUND);
      tft.setTextDatum(BC_DATUM);
      tft.setFreeFont(&DSEG7_Classic_Mini_Regular_34);
      tft.drawString(String(Displayfreq, 3), XFreqDispl + 120, YFreqDispl + 61);
      tft.setTextSize(2);
      tft.setTextColor(TFT_YELLOW, TFT_BLACK);
      tft.setFreeFont(NULL); 
      tft.drawString("MHz", XFreqDispl + 215, YFreqDispl + 61);
    }
  }
}

//=======================================================================================
void DrawDispl() {
//=======================================================================================
  //tft.fillRect(XFreqDispl, YFreqDispl, 240, 90, TFT_GREY);
  tft.fillRect(XFreqDispl + 5, YFreqDispl + 5, 230, 80, TFT_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setTextDatum(BC_DATUM);
  tft.drawString(band[bandIdx].bandName, XFreqDispl + 160, YFreqDispl + 18);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString("Band", XFreqDispl + 160, YFreqDispl + 8);

  FreqDispl();
  //if (band[bandIdx].bandType != FM_BAND_TYPE) {
    tft.setTextSize(1);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.drawString("Mod.", XFreqDispl + 80, YFreqDispl + 8);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    Modtext = bandModeDesc[currentMode];
    if ((Modtext == "USB") and (CWShift == true)){
      Modtext = "CW";
    }else {
      Modtext = bandModeDesc[currentMode];
    }
    tft.drawString(Modtext, XFreqDispl + 80, YFreqDispl + 18);
    tft.setTextPadding(tft.textWidth("2.2kHz"));
    if (currentMode == AM) BWtext = bandwidthAM[bwIdxAM];
    if (currentMode == LSB || currentMode == USB || currentMode == CW) BWtext = bandwidthSSB[bwIdxSSB];
    if (currentMode == FM){
      //tft.setTextPadding(tft.textWidth("100 kHz")); 
      BWtext = (bandwidthFM[bwIdxFM]);
    }
    if (BWtext == "AUT") {
      tft.drawString(BWtext, XFreqDispl + 120, YFreqDispl + 18);
    }else tft.drawString(BWtext + " KHz", XFreqDispl + 120, YFreqDispl + 18);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.drawString("Width", XFreqDispl + 120, YFreqDispl + 8);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    if (currentMode == FM){
      tft.drawString(String((band[bandIdx].currentStep)*10) + " KHz", XFreqDispl + 200, YFreqDispl + 18);
    } else  tft.drawString(String(band[bandIdx].currentStep) + " KHz", XFreqDispl + 200, YFreqDispl + 18);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.drawString("Step", XFreqDispl + 200, YFreqDispl + 8);
}

//=======================================================================================
void AGCfreqdisp() {
//=======================================================================================

  tft.setTextSize(1);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setTextDatum(BC_DATUM);
  tft.drawString("AGC", XFreqDispl + 50, YFreqDispl + 8);//16
  si4735.getAutomaticGainControl();
  if (si4735.isAgcEnabled()) {
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.drawString("On", XFreqDispl + 50, YFreqDispl + 18);
    tft.setTextColor(TFT_ORANGE, TFT_BLACK);
  } else {
    if (AGCgain == 0)   {
      tft.drawString("Off", XFreqDispl + 50, YFreqDispl + 18);
    } else {
      tft.drawString(String(currentAGCgain), XFreqDispl + 50, YFreqDispl + 18);
    }
  }
}


//=======================================================================================
void BFOStepdisp() {
//=======================================================================================
  if (band[bandIdx].bandType != FM_BAND_TYPE) {
    tft.setTextSize(1);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setTextDatum(BC_DATUM);
    tft.setTextPadding(tft.textWidth("XXX"));
    tft.drawString("BFO", XFreqDispl + 20, YFreqDispl + 8);
    tft.setTextPadding(tft.textWidth("88"));

    if (bfoOn) {
      #ifdef IhaveCrystal
      tft.setTextColor(TFT_YELLOW, TFT_BLACK);
      tft.drawString(String(currentBFOStep), XFreqDispl + 20, YFreqDispl + 18);
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
      #endif
    } else {
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
      tft.drawString("  ", XFreqDispl + 20, YFreqDispl + 26);
    }
  }
}

//=======================================================================================
void ErrorBeep()  {
//=======================================================================================
  Beep(4, 100);
  delay(2000);
}





//=======================================================================================
void MuteAud() {
//=======================================================================================
  // Stop muting only if this condition has changed
  if (((millis() - elapsedAudMut) > MIN_ELAPSED_AudMut_TIME ) and (AudioMut = true))// 150 msec
  {
    AudioMut = false;
    si4735.setHardwareAudioMute(0);
  }
}

//=======================================================================================
void showFirmwareInformation() {
//=======================================================================================
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  int xp;
  int yp;
#ifdef IhaveVertTFT
  tft.setCursor(0, 20);  
  tft.println("Firmware Information");
  tft.setCursor(0, 35); 
  tft.println("====================");
  xp = 35;
  yp = 50; 
#endif

#ifdef IhaveHoriTFT
  tft.setCursor(40, 20);  
  tft.println("Firmware Information");
  tft.setCursor(40, 35); 
  tft.println("====================");
  xp = 75;
  yp = 50; 
#endif
  tft.setTextSize(1);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setCursor(xp, yp + 20);
  tft.print("Part Number (HEX)........: ");
  tft.println(si4735.getFirmwarePN(), HEX);
  tft.setCursor(xp, yp + 40);
  tft.print("Firmware Major Revision..: ");
  tft.println(si4735.getFirmwareFWMAJOR());
  tft.setCursor(xp, yp + 60);
  tft.print("Firmware Minor Revision..: ");
  tft.println(si4735.getFirmwareFWMINOR());
  tft.setCursor(xp, yp + 80);
  tft.print("Patch ID ................: ");
  tft.print(si4735.getFirmwarePATCHH(), HEX);
  tft.println(si4735.getFirmwarePATCHL(), HEX);
  tft.setCursor(xp, yp + 100);
  tft.print("Component Major Revision.: ");
  tft.println(si4735.getFirmwareCMPMAJOR());
  tft.setCursor(xp, yp + 120);
  tft.print("Component Minor Revision.: ");
  tft.println(si4735.getFirmwareCMPMINOR());
  tft.setCursor(xp, yp + 140);
  tft.print("Chip Revision............: ");
  tft.println(si4735.getFirmwareCHIPREV());

  while (x == 0) {
    presStat = tft.getTouch(&x, &y);
  }
  x = y = 0;
  Beep(1, 0);
  delay(400);
}
