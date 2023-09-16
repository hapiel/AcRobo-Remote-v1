/*
Title: Acrobot Remote v1
Author: Daniel Simu
Date: January 2023
Description: This remote serves as a slave to the Acrobot. It reads data from sliders, joysticks, keypad matrix, encoder, and forwards this to the Acrobot over ESP-NOW. It also checks its own battery voltage, and gives an alarm if this goes below 10%.
Status light: Blue = connected wireless, Red = disconnected wireless, Yellow = low power mode.
TODO: The LCD is updated by the Acrobot
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <ESP32Encoder.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad_I2C.h>    // https://github.com/joeyoung/arduino_keypads/tree/master/Keypad_I2C
#include <Keypad.h>        // https://playground.arduino.cc/Code/Keypad/
#include <RunningMedian.h> // https://github.com/RobTillaart/RunningMedian
#include <WiFi.h>
#include <esp_now.h>

#define BATTERY_V 35
#define LOW_POWER_SW 18

#define BUZZER 13

#define ENCODER_A 26
#define ENCODER_B 25
#define ENCODER_SW 33

#define LED_R 12
#define LED_G 14
#define LED_B 27

#define JOYSTICK_L_Y 34
#define JOYSTICK_L_X 32
#define JOYSTICK_R_X 36
#define JOYSTICK_R_Y 39

#define I2CMATRIX 0x38



// ---------------
// MARK: - FORWARD DECLARATIONS in order of document

// ADC
Adafruit_ADS1115 ads1115; // adc converter

uint32_t adsTimer;
uint8_t currentAds = 0;
int16_t sliderLL;
int16_t sliderLA;
int16_t sliderRL;
int16_t sliderRA;

void readADS();

// BATTERY
int8_t batteryPercent;
RunningMedian batterySamples = RunningMedian(64);
uint32_t batteryAlarmTimer = 0;
bool chargingState = false;

void setModemSleep();
void wakeModemSleep();
void updateBattery();

// BUZZER
uint32_t buzzerTimer = 0;

void setBuzzer(uint16_t time = 100);
void updateBuzzer();

// DATA ESPNOW

uint32_t dataTimer = 0;
uint8_t robotAddress[] = {0x94, 0xE6, 0x86, 0x00, 0xE0, 0xD0}; // mac address of robot
typedef struct struct_data_in
{
  double kP;
  double kI;
  double kD;

  double rInput;
  double lInput;

} struct_data_in;

double kP = 0.2;
double kI = 0;
double kD = 0;

struct_data_in dataIn;

// data out:

typedef struct struct_data_out
{
  int16_t joystickLX;
  int16_t joystickLY;
  int16_t joystickRX;
  int16_t joystickRY;

  int16_t sliderLL;
  int16_t sliderLA;
  int16_t sliderRL;
  int16_t sliderRA;

  int16_t encoderPos;
  bool encoderSwDown;

  char key;

  int8_t batteryPercent;

  double kP;
  double kI;
  double kD;

  double lP;
  double lI;
  double lD;

  uint16_t rTargetPositionDegrees;
  uint16_t lTargetPositionDegrees;

} struct_data_out;

uint16_t rTargetPositionDegrees = 180;
uint16_t lTargetPositionDegrees = 180;

struct_data_out dataOut;

esp_now_peer_info_t peerInfo;
bool lastPackageSuccess = false;

void prepareData();
void sendData();
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);

// ENCODER
ESP32Encoder encoder;
int16_t encoderPos = 0;
bool encoderUp = false;
bool encoderDown = false;
bool encoderSwDown = false;
bool encoderSwPressed = false;

void updateEncoder();
void encoderPID();


// KEYPAD MATRIX
const byte ROWS = 4;
const byte COLS = 4;
char hexaKeys[ROWS][COLS] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}};
byte rowPins[ROWS] = {0, 1, 2, 3}; // connect to the row pinouts of the keypad
byte colPins[COLS] = {4, 5, 6, 7}; // connect to the column pinouts of the keypad
Keypad_I2C keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS, I2CMATRIX);

void checkButtons();



// LCD
LiquidCrystal_I2C lcd(0x27, 20, 4);
uint32_t lcdTimer = 0;

void setLCD();
void updateLCD();
void lcdAllModesOff();

bool lcdJoystick = false;
void lcdSetJoystick();
void lcdUpdateJoystick();

bool lcdSlider = false;
void lcdUpdateSlider();

bool lcdPID = true;
void lcdSetPID();
void lcdUpdatePID();

bool lcdTargetPosition = true;
void lcdSetTargetPosition();
void lcdUpdateTargetPosition();

bool lcdModeName = true;
void lcdSetModeName();


enum remoteModes {
  poseMode,
  sliderMode,
  moveMode
};

remoteModes remoteMode = poseMode;

// LED
void ledRed(uint8_t);
void ledGreen(uint8_t);
void ledBlue(uint8_t);
void ledYellow(uint8_t);
void ledWhite(uint8_t);
void updateLED();

// MOVES

uint32_t moveTimer = 0;

enum moveList{
  stop,
  relax,
  stand,
  walk,
  walkLarge,
  pirouette,
  acroyogaSequence,
  jump,
  flip,
  sit,
  backflip,
  pitch180,
  headBalanceSplit,
  musicSequence0,
  musicSequence1,
  musicSequence2,
  musicSequence3,
  musicSequence4,
  musicSequence5,
  musicSequence6,
  musicSequence7,
  musicSequence8,
  musicSequence9,
  textSequence0,
  textSequence1,
};

moveList move = stop;

void startMove(moveList);
void updateMoves();
bool moveTimePassed(u32_t time);

// POSITIONS

uint16_t forwardLimit = 80;
uint16_t backwardLimit = 280;

uint16_t withinLimits(uint16_t position);
void pBow(int16_t upperBodyDegrees = 45);
void pStand();
void pStep(int8_t degrees = 20, bool rightFront = true);
void pStepRight(int8_t degrees = 20);
void pStepLeft(int8_t degrees = 20);
void pKick(int8_t degrees = 90, bool rightFront = true);
void pKickRight(int8_t degrees = 90);
void pKickLeft(int8_t degrees = 90);
void pSpecial(int16_t degreesL, int16_t degreesR);



// PRINT
uint32_t printTimer = 0;

void printAll();

// END FORWARD DECLARATIONS
// **********************************

// MARK: - Poses new

void pStand(){
  pBow(2);
}

void pStep(int8_t degrees, bool rightFront){
  // for left, can also do negative value in degrees

  int8_t sign = rightFront * 2 - 1;

  rTargetPositionDegrees = withinLimits(180 + degrees * -sign);
  lTargetPositionDegrees = withinLimits(180 + degrees * sign);
}

void pStepRight(int8_t degrees){
  pStep(degrees, 1);
}

void pStepLeft(int8_t degrees){
  pStep(degrees, 0);
}

void pKick(int8_t degrees, bool rightFront){
  
  if (rightFront){
    lTargetPositionDegrees = 180;
    rTargetPositionDegrees = withinLimits(180 - degrees);
  } else {
    rTargetPositionDegrees = 180;
    lTargetPositionDegrees = withinLimits(180 - degrees);
  }

}

void pSpecial(int16_t degreesL, int16_t degreesR){
  lTargetPositionDegrees = withinLimits(180 - degreesL);
  rTargetPositionDegrees = withinLimits(180 - degreesR);
}

void pKickRight(int8_t degrees){
  
  pKick(degrees, 1);

}

void pKickLeft(int8_t degrees){
  
  pKick(degrees, 0);

}


void pBow(int16_t upperBodyDegrees){
  // -90 to 90
  lTargetPositionDegrees = withinLimits(180 - upperBodyDegrees);
  rTargetPositionDegrees = withinLimits(180 - upperBodyDegrees);
}

// MARK: - Movement new

uint32_t choreoTimer;
uint16_t choreoStepCounter = 0;

// declarations:
void myMove();
void moveWalk();
void moveWalkCont();
void moveStand();
void moveBreathe();
void moveWalkSlow();
void moveStepRight();
void moveStepLeft();
void moveWaveFront();
void moveWaveBack();
void moveBreatheSit();

bool choreoTimePassed(uint32_t time){
  return (choreoTimer + time) <= millis();
}


struct ChoreographyStep{
  uint32_t time;
  void (*move)();
};

ChoreographyStep* currentChoreo;

ChoreographyStep myChoreo[] = {
  {0, [](){ kP = 1.2; pStand();}},
  {1000, [](){ kP = 0.6; pStep(90,1);}},
  {2000, myMove },
  {4000, moveWalk },
  {UINT32_MAX, nullptr}  // Sentinel value indicating the end of the array
};

ChoreographyStep mySecondChoreo[] = {
  {0, [](){ kP = 1.2; pStand();}},
  {1000, [](){ kP = 0.6; pStep(90,1);}},
  {4000, moveWalk },
  {6000, moveWalk },
  {8000, moveWalk },
  {10000, moveWalk },
  {UINT32_MAX, nullptr}  // Sentinel value indicating the end of the array
};

ChoreographyStep testChoreo[] = {
  {0, [](){ kP = 0.8; pStand();}},
  {2000, moveWalkCont},
  {10000, [](){ kP = 0.8; pStand();} },
  {UINT32_MAX, nullptr}  // Sentinel value indicating the end of the array
};

ChoreographyStep standingChoreo[] = {
  {0, moveStand},
  {UINT32_MAX, nullptr}  // Sentinel value indicating the end of the array
};

ChoreographyStep breathingChoreo[] = {
  {0, moveBreathe},
  {UINT32_MAX, nullptr}  // Sentinel value indicating the end of the array
};

ChoreographyStep walkingChoreo[] = {
  {0, moveWalkCont},
  {UINT32_MAX, nullptr}  // Sentinel value indicating the end of the array
};

ChoreographyStep nodChoreo[] = {
  {0, [](){ kP = 1.4; pBow(90);}},
  {1000, [](){ kP = 1.4; pBow(75);}},
  {2000, [](){ kP = 1.4; pBow(90);}},
  {8000, moveStand},

  {UINT32_MAX, nullptr}  // Sentinel value indicating the end of the array
};

ChoreographyStep mirrorChoreo[] = {
  {0, moveBreathe},
  {30000, [](){ kP = 0.8; pBow(15);}},
  {31000, [](){ kP = 0.8; pBow(20);}},
  {32000, [](){ kP = 0.8; pBow(25);}},
  {33000, [](){ kP = 0.8; pBow(30);}},
  {34000, [](){ kP = 0.8; pBow(38);}}, // fall into me
  {37800, [](){ kP = 0.8; pBow(15);}},
  {40000, [](){ kP = 1; pStand();}}, // back standing
  {41000, moveBreathe}, 
  {47000, [](){ kP = 1.4; pKickRight(50);}}, // step 1
  {49500, [](){ kP = 0.8; pStepRight(20);}},
  {50000, [](){ kP = 1.2; pStepRight(20);}},
  {55000, [](){ kP = 0.9; pBow(-5);}}, // step 2
  {56200, [](){ kP = 0.8; pStepLeft(20);}},
  {57000, [](){ kP = 1.4; pStepLeft(20);}},
  {60000, moveWalkSlow}, 
  {80000, [](){ kP = 0.8; pStepRight(15);}},
  {80500, [](){ kP = 1.4; pStepRight(15);}},
  {84000, [](){ kP = 1.4; pStepLeft(20);}}, // sudden turn
  {86000, [](){ kP = 0.8; pStepRight(20);}},
  {86200, [](){ kP = 1.4; pStepRight(20);}},
  {87000, [](){ kP = 0.8; pStepLeft(20);}},
  {87200, [](){ kP = 1.4; pStepLeft(20);}},
  {88300, moveWalkCont}, 

  {120000, moveStand}, 
  {125000, [](){ kP = 1.4; pKickRight(20);}}, // raise leg
  {128000, [](){ kP = 1.4; pKickRight(30);}},
  {131000, [](){ kP = 1.4; pKickRight(40);}},
  {134000, [](){ kP = 1.4; pKickRight(50);}},
  {137000, [](){ kP = 1.5; pKickRight(60);}},
  {140000, [](){ kP = 1.6; pKickRight(70);}},
  {143000, [](){ kP = 1.7; pSpecial(-7, 80);}},
  {145000, [](){ kP = 1.8; pSpecial(-14, 90);}},
  {150000, [](){ kP = 2; pSpecial(-21, 100);}},

  {154000, moveStand}, 

  {157000, [](){ kP = 0.8; pBow(45);}}, // situp challenge
  {158000, [](){ kP = 1; pBow(90);}},

  {168000, [](){ kP = 1.4; pBow(75);}},
  {172000, [](){ kP = 1.4; pBow(65);}},
  {176000, [](){ kP = 1.4; pBow(50);}},

  {190000, moveStand}, 

  // random sequence

  {200000, [](){ kP = 1.1; pKickRight(80);}}, 
  {201400, [](){ kP = 1.1; pStepRight(70);}}, 
  {202200, [](){ kP = 1.1; pKickLeft(80);}}, 
  {203100, [](){ kP = 1.1; pStepLeft(70);}}, 
  {204000, [](){ kP = 1.1; pBow(45);}}, 
  {206600, [](){ kP = 1.3; pBow(90);}}, 
  {207000, [](){ kP = 1.1; pStand();}}, 
  {209100, [](){ kP = 0.8; pKickRight(80);}}, 
  {210500, [](){ kP = 1.1; pStepRight(70);}}, 
  {212900, [](){ kP = 1.3; pBow(90);}}, 
  {213200, [](){ kP = 1.1; pStand();}}, 
  {214100, [](){ kP = 1.1; pStepLeft(70);}}, 
  {216000, [](){ kP = 0.8; pKickLeft(80);}}, 
  {217400, [](){ kP = 1.1; pStand();}}, 
  {219000, [](){ kP = 1.1; pStepRight(70);}}, 
  {220400, [](){ kP = 1.1; pBow(45);}}, 
  {222100, [](){ kP = 0.8; pStand();}}, 
  {222800, [](){ kP = 0.8; pBow(90);}}, 
  {223100, [](){ kP = 1.1; pStepRight(70);}}, 
  {223600, [](){ kP = 0.8; pKickLeft(80);}}, 
  {224000, [](){ kP = 1.3; pBow(30);}}, 
  {225000, [](){ kP = 1.1; pStand();}}, 
  {224500, [](){ kP = 1.3; pBow(90);}}, 

  {225000, moveBreatheSit},  // sitting lying on the floor

  // add longer

  {250000 + 6000, [](){ kP = 1.3; pBow(90);}}, 
  {252000 + 6000, [](){ kP = 1.1; pBow(75);}}, 
  {254000 + 6000, [](){ kP = 1.3; pBow(90);}}, 
  {256000 + 6000, [](){ kP = 1.1; pBow(65);}}, 
  {258000 + 6000, [](){ kP = 1.3; pBow(90);}}, 
  {260000 + 6000, [](){ kP = 1.1; pBow(55);}}, 
  {261500 + 6000, [](){ kP = 1.3; pBow(90);}}, 
  {263000 + 6000, [](){ kP = 1.1; pBow(45);}}, 
  {264500 + 6000, [](){ kP = 1.3; pBow(85);}}, 
  {266000 + 6000, [](){ kP = 1.1; pBow(35);}}, 
  {267500 + 6000, [](){ kP = 1.3; pBow(80);}}, 
  {269000 + 6000, [](){ kP = 1.1; pBow(15);}}, 
  {271500 + 6000, [](){ kP = 1.3; pBow(80);}}, 
  {273000 + 6000, [](){ kP = 1.1; pBow(0);}}, 
  {274500 + 6000, [](){ kP = 1.3; pBow(80);}}, 
  {276000 + 6000, [](){ kP = 1.1; pBow(-15);}}, 
  {277500 + 6000, [](){ kP = 1.3; pBow(80);}}, 
  {278800 + 6000, [](){ kP = 1.1; pBow(-35);}}, 

  {279000 + 7000, [](){ kP = 1.3; pBow(80);}}, 
  {281200 + 7000, [](){ kP = 1.1; pBow(-65);}}, 
  {282400 + 7000, [](){ kP = 1.3; pBow(80);}}, 
  {283650 + 7000, [](){ kP = 1.1; pBow(-65);}}, 
  {284800 + 7000, [](){ kP = 1.3; pBow(80);}}, 
  {285900 + 7000, [](){ kP = 1.1; pBow(-65);}}, 
  {287000 + 7000, [](){ kP = 1.3; pBow(80);}}, 
  {288000 + 7000, [](){ kP = 1.1; pBow(-65);}}, 
  {289000 + 7000, [](){ kP = 1.3; pBow(80);}}, 
  {230000 + 7000, [](){ kP = 1.1; pBow(-65);}}, 
  {231000 + 7000, [](){ kP = 1.3; pBow(80);}}, 
  {232000 + 7000, [](){ kP = 1.1; pBow(-65);}}, 
  {233000 + 7000, [](){ kP = 1.3; pBow(80);}}, 
  {234000 + 7000, [](){ kP = 1.1; pBow(-65);}}, 
  {235000 + 7000, [](){ kP = 1.3; pBow(80);}}, 
  {236000 + 7000, [](){ kP = 1.1; pBow(-65);}}, 
  {237000 + 7000, [](){ kP = 1.3; pBow(80);}}, 
  {238000 + 7000, [](){ kP = 1.1; pBow(-65);}}, 
  {239000 + 7000, [](){ kP = 1.3; pBow(80);}}, 
  {240000 + 7000, [](){ kP = 1.1; pBow(-65);}}, 
  {241000 + 7000, [](){ kP = 1.3; pBow(80);}}, 
  {242000 + 7000, [](){ kP = 1.1; pBow(-65);}}, 
  {243000 + 7000, [](){ kP = 1.3; pBow(80);}}, 

// add more




  {251500, [](){ kP = 1.0; pBow(-65);}}, 
  {253000 + 7000, [](){ kP = 1.2; pBow(80);}}, 
  {235000 + 7000, [](){ kP = 1.1; pBow(-65);}}, 
  {238000 + 7000, [](){ kP = 1.2; pBow(75);}}, 


  {UINT32_MAX, nullptr}  // end of the array
};

ChoreographyStep acroChoreo[] = {
  {0, moveStand},

  {8780, moveStepRight},
  {9830, moveStepLeft},
  {10865, moveStepRight},
  {11870, moveStepLeft},

  {12905, moveStand},
  // bird
  {16846, [](){ kP = 1.6; pBow(25);}},
  {17846, [](){ kP = 1.2; pBow(15);}},
  {18846, [](){ kP = 1.8; pBow(0);}},
  // swimming
  {20789, [](){ kP = 1.8; pStepRight(10);}},
  {21310, [](){ kP = 1.8; pStepLeft(10);}},
  {21805, [](){ kP = 1.8; pStepRight(10);}},
  {22295, [](){ kP = 1.8; pStepLeft(10);}},
  {22765, [](){ kP = 1.8; pStepRight(10);}},
  {23245, [](){ kP = 1.8; pStepLeft(10);}},
  {23720, [](){ kP = 1.8; pStepRight(10);}},
  {24161, [](){ kP = 1.8; pBow(0);}},
  // cloth hanger
  {24640, [](){ kP = 0.3; pBow(75);}},

  {25780, [](){ kP = 1.2; pKickRight(90);}},
  {26743, [](){ kP = 0.8; pStepRight(70);}},
  {28780, [](){ kP = 1.2; pStepRight(45);}},
  {29866, [](){ kP = 1.2; pStepRight(20);}},
  {30742, [](){ kP = 1.2; pStepRight(0);}},

  {35663, [](){ kP = 0.5; pStepRight(70);}},

  {38577, [](){ kP = 0.6; pBow(92);}},

  {40898, [](){ kP = 1; pBow(75);}},
  {41722, [](){ kP = 1.2; pBow(45);}},
  {42715, [](){ kP = 1.2; pBow(5);}}, // staan in handen

  {49945, [](){ kP = 1.4; pBow(15);}}, // richting vloer vanuit snoek
  {54015, [](){ kP = 1.4; pStand();}},

  {56965, moveStepLeft},
  {58044, moveStepRight},
  {59032, moveStepLeft},
  {60054, moveStepRight},
  {61173, moveStepLeft},

  {62996, [](){ kP = 3; pSpecial(-10, 90);}}, // backflip
  {63200, [](){ kP = 2; pStepRight(90);}},
  {63900, [](){ kP = 1.5; pBow(20);}},
  {64400, [](){ kP = 1.1; pStand();}},
  {67135, [](){ kP = 0.5; pBow(45);}},
  {69011, [](){ kP = 1.1; pStand();}},


  {72702, [](){ kP = 0.8; pBow(15);}}, // schouder zit
  {73247, [](){ kP = 0.8; pBow(30);}},
  {74064, [](){ kP = 1; pBow(70);}},

  {76422, [](){ kP = 0.8; pBow(50);}},
  {77422, [](){ kP = 1; pBow(-20);}}, // uitbouw

  {81301, [](){ kP = 1.2; pStand();}},

  {83388, [](){ kP = 1.8; pKickRight(-14);}}, // opzwaai snoek
  {84000, [](){ kP = 1.6; pKickRight(80);}},
  {84300, [](){ kP = 1.4; pStepRight(90);}},
  {85000, [](){ kP = 1.0; pBow(30);}},
  {85500, [](){ kP = 1.0; pBow(-10);}},

  {87301, [](){ kP = 1.0; pBow(50);}}, // kopstand
  {88000, [](){ kP = 0.5; pSpecial(70,90);}},
  {89300, [](){ kP = 1.0; pSpecial(30,90);}},
  {90100, [](){ kP = 1.0; pSpecial(-10,90);}},
  {91300, [](){ kP = 0.8; pSpecial(-50,90);}},
  {91500, [](){ kP = 1.2; pSpecial(-80,90);}},

  {95648, [](){ kP = 1.2; pStepRight(50);}},
  {96339, [](){ kP = 0.8; pBow(60);}},
  {96839, [](){ kP = 1.2; pBow(95);}},

  {99535, [](){ kP = 1.2; pBow(70);}},
  {100000, [](){ kP = 1.2; pBow(50);}},
  {100800, [](){ kP = 1.2; pBow(20);}},

  {101618, [](){ kP = 1.5; pStand();}}, // schouder staan

  {104200, moveWaveFront}, // wave
  {105880, moveWaveBack},
  {107229, moveWaveFront},
  {108743, moveWaveBack},
  {110181, moveWaveFront},
  {111998, moveWaveBack},
  {113448, moveWaveFront},
  {114975, moveWaveBack},
  {116581, [](){ kP = 1.6; pStand();}},

  {130280, [](){ kP = 1.8; pKickRight(90);}}, // finale pose



  {135000, [](){ kP = 1.1; pStand();}},


  {UINT32_MAX, nullptr}  // Sentinel value indicating the end of the array
};




// MOVES

void moveWaveFront(){
  kP = 0.8;
  pBow(50);

  if (choreoTimePassed(currentChoreo[choreoStepCounter-1].time + 200)){
    kP = 1.2;
    pBow(55);
  }
    if (choreoTimePassed(currentChoreo[choreoStepCounter-1].time + 500)){
    kP = 1.4;
    pBow(65);
  }
}

void moveWaveBack(){
  kP = 0.8;
  pBow(-30);

  if (choreoTimePassed(currentChoreo[choreoStepCounter-1].time + 200)){
    kP = 1.2;
    pBow(-45);
  }
    if (choreoTimePassed(currentChoreo[choreoStepCounter-1].time + 500)){
    kP = 1.4;
    pBow(-55);
  }
}


void myMove(){
  kP = 0.6;
  pKick(90,1);
}

void moveWalk (){
  kP = 1.4;
  pStep(20, 1);

  // wait 800
  if (choreoTimePassed(currentChoreo[choreoStepCounter-1].time + 800)){
    pStep(20, 0);
  }
}

void moveWalkCont (){
  int timeStep = 1800;
  int miniStep = 200;

  for(int i = 0; i < 100; i++){
      // wait 800
    if (choreoTimePassed(currentChoreo[choreoStepCounter-1].time + i * timeStep)){ // 2000 + 1 * (800 * 0.4)
      kP = 0.8;
      pStep(20, 1);
    }
    if (choreoTimePassed(currentChoreo[choreoStepCounter-1].time + i * timeStep + miniStep)){ // 2000 + 1 * (800 * 0.5)
      kP = 1.4;
    }

    if (choreoTimePassed(currentChoreo[choreoStepCounter-1].time + i * timeStep + (timeStep/2)  )){
      kP = 0.8;
      pStep(20, 0);
    }

    if (choreoTimePassed(currentChoreo[choreoStepCounter-1].time + i * timeStep + (timeStep/2) + miniStep)){
      kP = 1.4;
    }
  }
}

void moveWalkSlow (){
  int timeStep = 3400;
  int miniStep = 400;

  for(int i = 0; i < 100; i++){
      // wait 800
    if (choreoTimePassed(currentChoreo[choreoStepCounter-1].time + i * timeStep)){ // 2000 + 1 * (800 * 0.4)
      kP = 0.8;
      pStep(20, 1);
    }
    if (choreoTimePassed(currentChoreo[choreoStepCounter-1].time + i * timeStep + miniStep)){ // 2000 + 1 * (800 * 0.5)
      kP = 1.4;
    }

    if (choreoTimePassed(currentChoreo[choreoStepCounter-1].time + i * timeStep + (timeStep/2)  )){
      kP = 0.8;
      pStep(20, 0);
    }

    if (choreoTimePassed(currentChoreo[choreoStepCounter-1].time + i * timeStep + (timeStep/2) + miniStep)){
      kP = 1.4;
    }
  }
}

void moveStand(){
  kP = 0.8;
  pStand();

  if (choreoTimePassed(currentChoreo[choreoStepCounter-1].time + 200)){
    kP = 1.2;
  }

  if (choreoTimePassed(currentChoreo[choreoStepCounter-1].time + 450)){
    kP = 1.6;
  }
}

void moveBreathe (){
  int timeStep = 4000;
  int miniStep = 400;

  for(int i = 0; i < 100; i++){
      // wait 800
    if (choreoTimePassed(currentChoreo[choreoStepCounter-1].time + i * timeStep)){ // 2000 + 1 * (800 * 0.4)
      kP = 0.7;
      pBow(10);
    }
    if (choreoTimePassed(currentChoreo[choreoStepCounter-1].time + i * timeStep + miniStep)){ // 2000 + 1 * (800 * 0.5)
      kP = 1.0;
    }

    if (choreoTimePassed(currentChoreo[choreoStepCounter-1].time + i * timeStep + (timeStep * 0.4)  )){
      kP = 1.4;
      pStand();
    }

    if (choreoTimePassed(currentChoreo[choreoStepCounter-1].time + i * timeStep + (timeStep * 0.4) + miniStep)){
      kP = 1.8;
    }
  }

}

void moveBreatheSit (){
  int timeStep = 4000;
  int miniStep = 400;

  for(int i = 0; i < 100; i++){
      // wait 800
    if (choreoTimePassed(currentChoreo[choreoStepCounter-1].time + i * timeStep)){ // 2000 + 1 * (800 * 0.4)
      kP = 1;
      pBow(90);
    }
    if (choreoTimePassed(currentChoreo[choreoStepCounter-1].time + i * timeStep + miniStep)){ // 2000 + 1 * (800 * 0.5)
      kP = 1.6;
    }

    if (choreoTimePassed(currentChoreo[choreoStepCounter-1].time + i * timeStep + (timeStep * 0.4)  )){
      kP = 1;
      pBow(80);
    }

    if (choreoTimePassed(currentChoreo[choreoStepCounter-1].time + i * timeStep + (timeStep * 0.4) + miniStep)){
      kP = 1.6;
    }
  }

}

void moveStepRight (){
  kP = 0.8;
  pStepRight(20);
  
  if (choreoTimePassed(currentChoreo[choreoStepCounter-1].time + 200)){
    kP = 1.4;
  }
}

void moveStepLeft (){
  kP = 0.8;
  pStepLeft(20);
  
  if (choreoTimePassed(currentChoreo[choreoStepCounter-1].time + 200)){
    kP = 1.4;
  }
}



// call once to set timer
void startChoreo(ChoreographyStep choreo[]){
  currentChoreo = choreo; 
  choreoTimer = millis();
  choreoStepCounter = 0;
}

// run this in the loop
void playCurrentChoreo(){
  if (choreoTimePassed(currentChoreo[choreoStepCounter].time)){
    if (currentChoreo[choreoStepCounter].time != UINT32_MAX){
      choreoStepCounter ++;
    }
    
  }
  currentChoreo[choreoStepCounter - 1].move();
}



// MARK: -Setup

void setup()
{
  currentChoreo = standingChoreo; 

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  pinMode(BUZZER, OUTPUT);
  pinMode(ENCODER_SW, INPUT_PULLUP);

  pinMode(LOW_POWER_SW, INPUT_PULLDOWN);

  Serial.begin(115200);
  Serial.println("remote is connected to serial");

  Wire.begin();
  keypad.begin();

  Serial.println("keypad added");

  lcd.init();
  lcd.clear();
  lcd.backlight();

  setLCD();

  // ESP32encoder library setup
  ESP32Encoder::useInternalWeakPullResistors = UP;
  encoder.attachSingleEdge(ENCODER_A, ENCODER_B);

  ads1115.begin();
  currentAds = 0;
  ads1115.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0, /*continuous=*/false);

  WiFi.mode(WIFI_MODE_STA);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // data sent&receive callback
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // register peer
  memcpy(peerInfo.peer_addr, robotAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }


  Serial.println("setup done");
}

//**********************************
// MARK: -Loop

void loop()
{
  // TODO test duration of loop
  updateBattery();
  updateEncoder(); // update encoder pos, and set encoder up/down bools for one loop
  updateBuzzer();  // play buzzer if buzzertimer is high
  updateLCD();
  updateLED();
  readADS();

  
  

  // in slider mode, send continuously.
  // if (lcdSlider){
    
  //   sendData();
  // }

  checkButtons();

  if (encoderUp)
  {
    setBuzzer(4);
  }
  if (encoderDown)
  {
    setBuzzer(4);
  }
  if (encoderSwPressed)
  {
    setBuzzer(50);
  }

  // printAll();
}

//**********************************
// MARK: -FUNCTIONS

//--------------------
// MARK: - Ads sliders

void readADS()
{
  // based on https://github.com/adafruit/Adafruit_ADS1X15/blob/master/examples/nonblocking/nonblocking.ino

  if (!ads1115.conversionComplete())
  {
    return;
  }

  if (currentAds == 0)
  {
    sliderRA = ads1115.getLastConversionResults();
    ads1115.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_1, /*continuous=*/false);
  }
  if (currentAds == 1)
  {
    sliderRL = ads1115.getLastConversionResults();
    ads1115.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_2, /*continuous=*/false);
  }
  if (currentAds == 2)
  {
    sliderLL = ads1115.getLastConversionResults();
    ads1115.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_3, /*continuous=*/false);
  }
  if (currentAds == 3)
  {
    sliderLA = ads1115.getLastConversionResults();
    ads1115.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0, /*continuous=*/false);
  }

  currentAds += 1;
  currentAds %= 4;
}

// ----------------------------
// MARK: -Battery

void setModemSleep()
{
  WiFi.setSleep(true);
  setCpuFrequencyMhz(80);
  Serial.println("sleep mode enabled");
}

void wakeModemSleep()
{
  WiFi.setSleep(false);
  setCpuFrequencyMhz(240);
  Serial.println("sleep mode disabled");
}

void updateBattery()
{
  batterySamples.add(analogRead(BATTERY_V));

  batteryPercent = map(batterySamples.getAverage(), 2060, 2370, 0, 100); // 2060 =~ 3.65v, 2370 =~ 4.2v

  // go to modem sleep
  if (digitalRead(LOW_POWER_SW) && !chargingState)
  {
    chargingState = true;
    setModemSleep();
  }
  if (!digitalRead(LOW_POWER_SW) && chargingState)
  {
    chargingState = false;
    wakeModemSleep();
    lcd.init();
    lcd.clear();
    setLCD();
  }

  // low battery alarm
  if (batteryPercent < 10 && batteryAlarmTimer < millis() && !chargingState)
  {
    setBuzzer(300);
    batteryAlarmTimer = millis() + 600;
  }
}

// ----------------------------
// MARK: -Buzzer

void setBuzzer(uint16_t time)
{
  buzzerTimer = millis() + time;
}
void updateBuzzer()
{
  if (buzzerTimer > millis()) {
    digitalWrite(BUZZER, HIGH);
  } else {
    digitalWrite(BUZZER, LOW);
  }
}

// ----------------------
// MARK: -Data espnow

void prepareData()
{

  // todo: correct for middle position

  uint16_t joyCorrectedLX = 4095 - analogRead(JOYSTICK_L_X); // invert scale
  uint16_t joyCorrectedLY = analogRead(JOYSTICK_L_Y);
  uint16_t joyCorrectedRX = analogRead(JOYSTICK_R_X);
  uint16_t joyCorrectedRY = 4095 - analogRead(JOYSTICK_R_Y); // invert scale

  uint16_t joyCenterLX = 2225; // unflipped: 1870
  uint16_t joyCenterLY = 1860;
  uint16_t joyCenterRX = 1840;
  uint16_t joyCenterRY = 2175; // unflipped: 1920

  if (joyCorrectedLX < joyCenterLX)
  {
    joyCorrectedLX = map(joyCorrectedLX, 0, joyCenterLX, 0, 2047);
  }
  else
  {
    joyCorrectedLX = map(joyCorrectedLX, joyCenterLX, 4095, 2048, 4095);
  }

  if (joyCorrectedLY < joyCenterLY)
  {
    joyCorrectedLY = map(joyCorrectedLY, 0, joyCenterLY, 0, 2047);
  }
  else
  {
    joyCorrectedLY = map(joyCorrectedLY, joyCenterLY, 4095, 2048, 4095);
  }

  if (joyCorrectedRX < joyCenterRX)
  {
    joyCorrectedRX = map(joyCorrectedRX, 0, joyCenterRX, 0, 2047);
  }
  else
  {
    joyCorrectedRX = map(joyCorrectedRX, joyCenterRX, 4095, 2048, 4095);
  }

  if (joyCorrectedRY < joyCenterRY)
  {
    joyCorrectedRY = map(joyCorrectedRY, 0, joyCenterRY, 0, 2047);
  }
  else
  {
    joyCorrectedRY = map(joyCorrectedRY, joyCenterRY, 4095, 2048, 4095);
  }

  dataOut.joystickLX = joyCorrectedLX;
  dataOut.joystickLY = joyCorrectedLY;
  dataOut.joystickRX = joyCorrectedRX;
  dataOut.joystickRY = joyCorrectedRY;

  // ads1115 is very slow, only process every 100ms...
  // if ( adsTimer < millis()){
  dataOut.sliderLL = sliderLL;
  dataOut.sliderLA = sliderLA;
  dataOut.sliderRL = sliderRL;
  dataOut.sliderRA = sliderRA;
  //   adsTimer = millis() + 5000;
  // }

  dataOut.encoderPos = encoderPos;
  dataOut.encoderSwDown = encoderSwDown;

  dataOut.batteryPercent = batteryPercent;

  dataOut.kP = kP;
  dataOut.kI = kI;
  dataOut.kD = kD;

  dataOut.rTargetPositionDegrees = rTargetPositionDegrees;
  dataOut.lTargetPositionDegrees = lTargetPositionDegrees;

}

void sendData()
{

  if (dataTimer < millis())
  {
    esp_now_send(robotAddress, (uint8_t *)&dataOut, sizeof(dataOut));

    dataTimer = millis() + 2;
  }
  // send once every 2 ms.
};

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  // Serial.print("\r\nLast Packet Send Status:\t");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  lastPackageSuccess = status == ESP_NOW_SEND_SUCCESS;

  // Note that too short interval between sending two ESP-NOW data may lead to disorder of sending callback function. So, it is recommended that sending the next ESP-NOW data after the sending callback function of the previous sending has returned.
  //  https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  memcpy(&dataIn, incomingData, sizeof(dataIn));
  Serial.print("Bytes received: ");
  Serial.println(len);

  // only after boot
  if (millis() < 1000){
    kP = dataIn.kP;
    kI = dataIn.kI;
    kD = dataIn.kD;
  }

}

// -----------------------
// MARK: - Encoder

void updateEncoder()
{
  int16_t newPos = encoder.getCount();
  bool newSw = !digitalRead(ENCODER_SW);

  if (newPos > encoderPos)
  {
    encoderUp = true;
  }
  else
  {
    encoderUp = false;
  }
  // encoderUp = newPos > encoderPos;

  if (newPos < encoderPos)
  {
    encoderDown = true;
  }
  else
  {
    encoderDown = false;
  }

  // switch
  if (newSw == true && encoderSwDown == false)
  {
    encoderSwPressed = true;
  }
  else
  {
    encoderSwPressed = false;
  }

  encoderPos = newPos;
  encoderSwDown = newSw;

  encoderPID();
}

uint8_t encoderPIDSelection = 0;

void encoderPID(){

  if (!lcdPID){
    return;
  }

  if (encoderSwPressed){
    encoderPIDSelection++;
    encoderPIDSelection %= 3;
    lcdSetPID();
  }

  if (encoderUp){
    if (encoderPIDSelection == 0){
      kP += 0.2;
    }
    if (encoderPIDSelection == 1){
      kI += 0.2;
    }
    if (encoderPIDSelection == 2){
      kD += 0.2;
    }
  }

  if (encoderDown){
    if (encoderPIDSelection == 0){
      kP -= 0.2;
      kP = max(kP, 0.);
    }
    if (encoderPIDSelection == 1){
      kI -= 0.2;
      kI = max(kI, 0.);
    }
    if (encoderPIDSelection == 2){
      kD -= 0.2;
      kD = max(kD, 0.);
    }
  }

}

//  -------------------------------
// MARK: - Keypad matrix

void checkButtons(){

  char keyInput = keypad.getKey();
  dataOut.key = keyInput;


  if (keyInput == '1'){
    remoteMode = poseMode;
    setLCD();
  }

  if (keyInput == '2'){
    remoteMode = sliderMode;
    kP = 0.2;
    setLCD();
  }

  if (keyInput == '3'){
    remoteMode = moveMode;
    startMove(relax);
    setLCD();
  }

  if (keyInput == 'A'){
    remoteMode = moveMode;
    startMove(textSequence0);
  }


  if (remoteMode == poseMode){
    if (keyInput == '0'){
      pStand();
    }

      // babysteps
    if (keyInput == '*'){
      pStep(20, false);
    }
    if (keyInput == '#'){
      pStep(20, true);
    }

    if (keyInput == '7'){
      pKick(90, false);
    }
    if (keyInput == '9'){
      pKick(90, true);
    }

    if (keyInput == '8'){
      pBow(45);
    }

    

    // send data only on button press
    if (lcdTargetPosition){
      if (keyInput != NO_KEY){
        prepareData();
        sendData();
      }
    }
  }

  if (remoteMode == sliderMode){
    lTargetPositionDegrees = map(sliderLL, 0, 17620, backwardLimit, forwardLimit);
    rTargetPositionDegrees = map(sliderRL, 0, 17620, backwardLimit, forwardLimit);
    prepareData();
    sendData();
  }

  if (remoteMode == moveMode){

    if (keyInput == '4'){
      startChoreo(standingChoreo);
    }

    if (keyInput == '5'){
      startChoreo(breathingChoreo);
    }

    if (keyInput == '6'){
      startChoreo(standingChoreo); // replaceme
    }

    if (keyInput == '7'){
      startChoreo(walkingChoreo);
    }
    
    if (keyInput == '8'){
      startChoreo(testChoreo);
    }

    if (keyInput == '9'){
      startMove(jump);
    }

    if (keyInput == 'B'){
      startChoreo(mirrorChoreo);
    }

    if (keyInput == 'C'){
      startChoreo(nodChoreo);
      // move = musicSequence6;
      // moveTimer = millis() - 51000;
    }

    if (keyInput == '*'){
      startMove(musicSequence0);
    }

    if (keyInput == '0'){
      startMove(pitch180);
    }

    if (keyInput == '#'){
      startMove(headBalanceSplit);
    }

    if (keyInput == 'D'){
      startChoreo(acroChoreo);
    }

    // updateMoves(); // OLD SYSTEM
    playCurrentChoreo();
    prepareData();
    sendData();
  }

}

//  -------------------------------
// MARK: - Lcd

void setLCD()
{
  lcd.clear();

  if (lcdJoystick){
    lcdSetJoystick();
  }

  if (lcdPID){
    lcdSetPID();
  }

  if (lcdTargetPosition){
    lcdSetTargetPosition();
  }

  if (lcdModeName){
    lcdSetModeName();
  }

}


void updateLCD()
{

  if (lcdTimer > millis())
  {
    return;
  }
  lcdTimer = millis() + 200; // update every 200 milliseconds

  if (lcdJoystick){
    lcdUpdateJoystick();
  }

  if (lcdSlider){
    lcdUpdateSlider();
  }

  if (lcdPID){
    lcdUpdatePID();
  }

  if (lcdTargetPosition){
    lcdUpdateTargetPosition();
  }

  // old style, battery:

  lcd.setCursor(18, 0);
  char batPerc[3];
  sprintf(batPerc, "%02d", batteryPercent);
  lcd.print(batPerc);
}

void lcdSetJoystick(){
  lcd.setCursor(0, 0);
  lcd.print("JLX: ");
  lcd.setCursor(0, 1);
  lcd.print("JLY: ");
  lcd.setCursor(0, 2);
  lcd.print("JRX: ");
  lcd.setCursor(0, 3);
  lcd.print("JRY: ");
}

void lcdUpdateJoystick(){
  char joyLX[5]; // amount of characters in string + 1
  char joyLY[5];
  char joyRX[5];
  char joyRY[5];
  sprintf(joyLX, "%04d", dataOut.joystickLX);
  sprintf(joyLY, "%04d", dataOut.joystickLY); // 4 digit string with leading zeros
  sprintf(joyRX, "%04d", dataOut.joystickRX);
  sprintf(joyRY, "%04d", dataOut.joystickRY);

  lcd.setCursor(4, 0);
  lcd.print(joyLX);
  lcd.setCursor(4, 1);
  lcd.print(joyLY);
  lcd.setCursor(4, 2);
  lcd.print(joyRX);
  lcd.setCursor(4, 3);
  lcd.print(joyRY);
}

void lcdUpdateSlider(){
  char slideLLeg[6]; // amount of characters in string + 1
  char slideLArm[6];
  char slideRLeg[6];
  char slideRArm[6];
  sprintf(slideLLeg, "%05d", dataOut.sliderLL);
  sprintf(slideLArm, "%05d", dataOut.sliderLA); // 4 digit string with leading zeros
  sprintf(slideRLeg, "%05d", dataOut.sliderRL);
  sprintf(slideRArm, "%05d", dataOut.sliderRA);

  // print ads
  lcd.setCursor(9, 0);
  lcd.print(slideLLeg);
  lcd.setCursor(9, 1);
  lcd.print(slideLArm);
  lcd.setCursor(9, 2);
  lcd.print(slideRLeg);
  lcd.setCursor(9, 3);
  lcd.print(slideRArm);
}

void lcdSetPID(){
  lcd.setCursor(0, 3);

  if (encoderPIDSelection == 0){
    lcd.print("P=     i:     d:");
  }
  if (encoderPIDSelection == 1){
    lcd.print("p:     I=     d:");
  }
  if (encoderPIDSelection == 2){
    lcd.print("p:     i:     D=");
  }
}

void lcdUpdatePID(){
  lcd.setCursor(2,3);
  lcd.print(kP, 1);
  lcd.setCursor(9,3);
  lcd.print(kI, 1);
  lcd.setCursor(16,3);
  lcd.print(kD, 1);
}

void lcdSetTargetPosition(){
  lcd.setCursor(0,0);
  lcd.setCursor(0,1);
  lcd.print("tR:     tL:");
  lcd.setCursor(0,2);
  lcd.print("pR:     pL:");
}

void lcdUpdateTargetPosition(){
  lcd.setCursor(3,1);
  lcd.print(rTargetPositionDegrees);
  lcd.print(" ");
  lcd.setCursor(3,2);
  lcd.print(dataIn.rInput, 0);
  lcd.print(" ");

  lcd.setCursor(11,1);
  lcd.print(lTargetPositionDegrees);
  lcd.print(" ");
  lcd.setCursor(11,2);
  lcd.print(dataIn.lInput, 0);
  lcd.print(" ");
}

void lcdSetModeName(){
  lcd.setCursor(0,0);
  if (remoteMode == poseMode){
    lcd.print("Pose mode");
  }
  if (remoteMode == sliderMode){
    lcd.print("Slider mode");
  }
  if (remoteMode == moveMode){
    lcd.print("Move mode");
  }
}

void lcdAllModesOff(){
  lcdJoystick = false;
  lcdSlider = false;
  lcdPID = false;
  lcdModeName = false;
}


// --------------------------------
// MARK: - Led

void ledRed(uint8_t brightness = 10)
{
  analogWrite(LED_R, brightness);
  analogWrite(LED_G, 0);
  analogWrite(LED_B, 0);
}

void ledGreen(uint8_t brightness = 10)
{
  analogWrite(LED_R, 0);
  analogWrite(LED_G, brightness);
  analogWrite(LED_B, 0);
}

void ledBlue(uint8_t brightness = 10)
{
  analogWrite(LED_R, 0);
  analogWrite(LED_G, 0);
  analogWrite(LED_B, brightness);
}

void ledYellow(uint8_t brightness = 10)
{
  analogWrite(LED_R, brightness);
  analogWrite(LED_G, brightness * 0.4);
  analogWrite(LED_B, 0);
}

void ledWhite(uint8_t brightness = 10)
{
  analogWrite(LED_R, brightness);
  analogWrite(LED_G, brightness);
  analogWrite(LED_B, brightness);
}

void ledOff()
{
  analogWrite(LED_R, 0);
  analogWrite(LED_G, 0);
  analogWrite(LED_B, 0);
}

void updateLED()
{
  if (chargingState)
  {
    ledYellow(5);
    return;
  }
  if (lastPackageSuccess)
  {
    ledBlue();
  }
  else
  {
    ledRed();
  }
}

// --------------
// MARK: - Moves



void startMove(moveList theMove){
  move = theMove;
  moveTimer = millis();
}

void updateMoves(){
  if (move == relax){
    kP = 0;
  }

  if (move == sit){
    kP = 0.8;
    pBow(90);
  }

  if (move == stop){
    kP = 2;
    lTargetPositionDegrees = dataIn.lInput;
    rTargetPositionDegrees = dataIn.rInput;

    // round to even
    lTargetPositionDegrees = (lTargetPositionDegrees/2) * 2 ;
    rTargetPositionDegrees = (rTargetPositionDegrees/2) * 2 ;
  }

  if (move == stand){
    pStand();
    kP = 0.6;
    if (moveTimePassed(300)){
      kP = 1;
    }
    if (moveTimePassed(600)){
      kP = 1.5;
    }
    if (moveTimePassed(1000)){
      kP = 2;
    }
  }

  if (move == walk){
    kP = 1.4;
    pStep(20, 1);
    if (moveTimePassed(800)){
      pStep(20, 0);
    }
    if (moveTimePassed(1600)){
      startMove(walk);
    }
  }

  if (move == jump){
    kP = 1.4;
    pStand();

    if (moveTimePassed(2000)){
      kP = 0.6;
      pBow(45);
    }
    if (moveTimePassed(3000)){
      kP = 4;
      pBow(-10);
    }
    if (moveTimePassed(3800)){
      kP = 2;
      pBow(10);
    }
    if (moveTimePassed(6000)){
      kP = 0.8;
      pStand();
    }
  }

  if (move == flip){
    kP = 1.4;
    pStand();

    if (moveTimePassed(2000)){
      kP = 1;
      pBow(15);
    }
    if (moveTimePassed(3000)){
      kP = 2;
      pStand();
    }
    if (moveTimePassed(3300)){
      kP = 3;
      pKick(90, 1);
    }
    if (moveTimePassed(3500)){
      kP = 2;
      pStep(90, 1);
    }
    if (moveTimePassed(4300)){
      kP = 1.5;
      pBow(20);
    }
    if (moveTimePassed(5500)){
      kP = 1.5;
      pStand();
    }
  }

  if (move == pirouette){
    kP = 1.4;
    pStand();

    if (moveTimePassed(2000)){
      kP = 3;
      rTargetPositionDegrees = 200;
      lTargetPositionDegrees = 170;
    }
    if (moveTimePassed(3000)){
      kP = 2;
      pKick(90);
    }
    if (moveTimePassed(3450)){
      kP = 2;
      pBow(10);
    }
    if (moveTimePassed(3800)){
      kP = 1.8;
      pStand();
    }
  }

  if (move == backflip){

    kP = 1.6;
    pStand();

    uint32_t moveTime = 0;

    // prep position
    if (moveTimePassed(moveTime += 7000)){
      kP = 1.8;
      pBow(15);
    }

    // tempo

    if (moveTimePassed(moveTime += 3000)){
      kP = 1.8;
      pBow(5);
    }
    if (moveTimePassed(moveTime += 800)){
      kP = 1.8;
      pBow(20);
    }
    if (moveTimePassed(moveTime += 800)){
      kP = 2.6;
      pBow(-20);
      // further bow back!?
    }

    // tuck

    if (moveTimePassed(moveTime += 400)){
      kP = 2.2;
      pBow(85);
    }

    if (moveTimePassed(moveTime += 650)){
      kP = 1.8;
      pStand();
    }

    // BOW ON LANDING???! to fix backwards motion

  }

  if (move == pitch180){

    kP = 1.6;
    pStand();

    uint32_t moveTime = 0;

    // prep position
    if (moveTimePassed(moveTime += 5000)){
      kP = 1.6;
      pKick(80, 1);
    }

    // tempo

    if (moveTimePassed(moveTime += 2000)){
      kP = 2.5;
      pKick(100, 1);
    }
    if (moveTimePassed(moveTime += 800)){
      kP = 1.8;
      pKick(90, 1);
    }
    if (moveTimePassed(moveTime += 800)){
      kP = 2;
      pKick(10, 1);
    }

    if (moveTimePassed(moveTime += 300)){
      kP = 2;
      pStand();
    }

    if (moveTimePassed(moveTime += 900)){
      kP = 0.8;
      pKick(90, 0);
    }

    if (moveTimePassed(moveTime += 400)){
      kP = 1.6;
      pKick(90, 0);
    }

    if (moveTimePassed(moveTime += 3000)){
      kP = 0.4;
      pStand();
    }

    if (moveTimePassed(moveTime += 1000)){
      kP = 2;
      pBow(8);
    }

  }

  if (move == headBalanceSplit){

    
    kP = 1.6;
    pStand();

    uint32_t moveTime = 0;

    // start move
    if (moveTimePassed(moveTime += 9000)){
      kP = 1;
      pStep(5, 1);
    }

    if (moveTimePassed(moveTime += 400)){
      kP = 1;
      pStep(11, 1);
    }

    if (moveTimePassed(moveTime += 400)){
      kP = 1;
      pStep(18, 1);
    }

    if (moveTimePassed(moveTime += 400)){
      kP = 1;
      pStep(26, 1);
    }

    if (moveTimePassed(moveTime += 400)){
      kP = 1;
      pStep(37, 1);
    }

    if (moveTimePassed(moveTime += 400)){
      kP = 1;
      pStep(47, 1);
    }

    if (moveTimePassed(moveTime += 400)){
      kP = 1;
      pStep(58, 1);
    }

    if (moveTimePassed(moveTime += 400)){
      kP = 1.2;
      pStep(67, 1);
    }

    if (moveTimePassed(moveTime += 400)){
      kP = 1.3;
      pStep(75, 1);
    }

    if (moveTimePassed(moveTime += 400)){
      kP = 1.4;
      pStep(82, 1);
    }

    if (moveTimePassed(moveTime += 400)){
      kP = 1.5;
      pStep(88, 1);
    }

    if (moveTimePassed(moveTime += 400)){
      kP = 1.5;
      pStep(90, 1);
    }

    if (moveTimePassed(moveTime += 5000)){
      kP = 1.5;
      pStep(80, 1);
    }

    if (moveTimePassed(moveTime += 500)){
      kP = 1.3;
      pStep(70, 1);
    }
    
    if (moveTimePassed(moveTime += 500)){
      kP = 1.1;
      pStep(60, 1);
    }

        if (moveTimePassed(moveTime += 500)){
      kP = 1.1;
      pStep(50, 1);
    }

    if (moveTimePassed(moveTime += 500)){
      kP = 1.1;
      pStep(40, 1);
    }

    if (moveTimePassed(moveTime += 500)){
      kP = 1.1;
      pStep(30, 1);
    }

    if (moveTimePassed(moveTime += 500)){
      kP = 1.1;
      pStep(20, 1);
    }

    if (moveTimePassed(moveTime += 500)){
      kP = 1.1;
      pStep(10, 1);
    }

    if (moveTimePassed(moveTime += 500)){
      kP = 1.8;
      pStand();
    }

  }

  if (move == acroyogaSequence){
    kP = 1.4;
    pStand();

    uint32_t moveTime = 0;

    // fall to bird
    if (moveTimePassed(moveTime += 3000)){
      kP = 1.8;
      pBow(25);
    }

    if (moveTimePassed(moveTime += 1000)){
      kP = 1.2;
      pBow(15);
    }
    if (moveTimePassed(moveTime += 2000)){
      kP = 2;
      pStand();
    }

    // swimming
    if (moveTimePassed(moveTime += 4500)){
      kP = 2;
      pKick(-20, 1);
    }
    if (moveTimePassed(moveTime += 1000)){
      kP = 0.5;
      pStand();
    }
    if (moveTimePassed(moveTime += 1000)){
      kP = 2;
      pKick(-20, 0);
    }
    if (moveTimePassed(moveTime += 1000)){
      kP = 0.5;
      pStand();
    }
    if (moveTimePassed(moveTime += 1000)){
      kP = 2;
      pKick(-20, 1);
    }
    if (moveTimePassed(moveTime += 1000)){
      kP = 0.5;
      pStand();
    }
    if (moveTimePassed(moveTime += 1000)){
      kP = 2;
      pKick(-20, 0);
    }
    if (moveTimePassed(moveTime += 1000)){
      kP = 0.5;
      pStand();
    }
    if (moveTimePassed(moveTime += 1000)){
      kP = 2;
      pKick(-20, 1);
    }
    if (moveTimePassed(moveTime += 1000)){
      kP = 0.5;
      pStand();
    }
    if (moveTimePassed(moveTime += 1000)){
      kP = 2;
      pKick(-20, 0);
    }
    if (moveTimePassed(moveTime += 1000)){
      kP = 0.5;
      pStand();
    }

    // cloth hanger
    if (moveTimePassed(moveTime += 1000)){
      kP = 1.6;
      pStand();
    }
    if (moveTimePassed(moveTime += 3000)){
      kP = 0.3;
      pBow(90);
    }
    if (moveTimePassed(moveTime += 5000)){
      kP = 2;
      pBow(76);
    }

    // kick naar bolkje

    if (moveTimePassed(moveTime += 1000)){
      kP = 1.2;
      pKick(90, 1);
    }
    if (moveTimePassed(moveTime += 1500)){
      kP = 0.7;
      pStep(75, 1);
    }
    if (moveTimePassed(moveTime += 1000)){
      kP = 1;
      pStep(60, 1);
    }
    if (moveTimePassed(moveTime += 500)){
      kP = 1.2;
      pStep(50, 1);
    }
    if (moveTimePassed(moveTime += 500)){
      kP = 1;
      pStep(35, 1);
    }
    if (moveTimePassed(moveTime += 500)){
      kP = 1.8;
      pStep(10, 1);
    }

    // swim in bolk

    if (moveTimePassed(moveTime += 1500)){
      kP = 1;
      pStep(10, 0);
    }
    if (moveTimePassed(moveTime += 1500)){
      kP = 1;
      pStep(10, 1);
    }
    if (moveTimePassed(moveTime += 800)){
      kP = 1.2;
      pStep(10, 0);
    }
    if (moveTimePassed(moveTime += 800)){
      kP = 1.2;
      pStep(10, 1);
    }

    // to knees
    if (moveTimePassed(moveTime += 4000)){
      kP = 1;
      pStep(10, 0);
    }

    if (moveTimePassed(moveTime += 800)){
      kP = 0.5;
      pStep(45, 0);
    }
    if (moveTimePassed(moveTime += 800)){
      kP = 0.5;
      pStep(75, 0);
    }
    if (moveTimePassed(moveTime += 800)){
      kP = 0.7;
      pStep(90, 0);
    }
    if (moveTimePassed(moveTime += 3000)){
      kP = 0.5;
      pKick(90, 1);
    }
    if (moveTimePassed(moveTime += 1500)){
      kP = 0.6;
      pBow(90);
    }

    // back to bird

    if (moveTimePassed(moveTime += 3000)){
      kP = 1.4;
      pKick(45, 1);
    }
    if (moveTimePassed(moveTime += 500)){
      kP = 1.4;
      pKick(90, 1);
    }
    if (moveTimePassed(moveTime += 2500)){
      kP = 1;
      pStand();
    }
    if (moveTimePassed(moveTime += 1000)){
      kP = 2.2;
      pStand();
    }

    // back to standing
    if (moveTimePassed(moveTime += 6000)){
      kP = 2;
      pBow(15);
    }

    if (moveTimePassed(moveTime += 10000)){
      kP = 2;
      pBow(10);
    }
    if (moveTimePassed(moveTime += 1000)){
      kP = 1.8;
      pBow(5);
    }
    if (moveTimePassed(moveTime += 1000)){
      kP = 1.6;
      pStand();
    }


    // current total time: 76s
    // Serial.print("acroyoga sequence total time: ");
    // Serial.println(moveTime);

  }

  // --------------------------------
  // MARK: - TEXT SEQUENCE 0

  if (move == textSequence0){
    kP = 0.6;
    pBow(45);

    if (moveTimePassed(7600)){
      kP = 0.2;
      pStand();
    }

    // raise left
    if (moveTimePassed(9250)){
      kP = 0.4;
      pKick(90, 0);
    }
    if (moveTimePassed(9550)){
      kP = 0.6;
      pKick(90, 0);
    }
    if (moveTimePassed(9950)){
      kP = 1;
      pKick(90, 0);
    }

    // swing excited
    if (moveTimePassed(19750)){
      kP = 1;
      rTargetPositionDegrees = 120;
    }
    if (moveTimePassed(20550)){
      kP = 0.8;
      rTargetPositionDegrees = 220;
    }
    if (moveTimePassed(21190)){
      kP = 0.8;
      rTargetPositionDegrees = 120;
    }

    if (moveTimePassed(29800)){
      kP = 1.2;
      rTargetPositionDegrees = 90;
    }

    // lower left

    if (moveTimePassed(37680)){
      kP = 0.2;
      lTargetPositionDegrees = 180;
    }

    // swing left

    if (moveTimePassed(41760)){
      kP = 0.8;
      lTargetPositionDegrees = 120;
    }

    if (moveTimePassed(42350)){
      kP = 0.8;
      lTargetPositionDegrees = 220;
    }
    if (moveTimePassed(43250)){
      kP = 0.2;
      lTargetPositionDegrees = 180;
    }

    // left back

    if (moveTimePassed(50830)){
      kP = 0.7;
      lTargetPositionDegrees = 235;
    }

    if (moveTimePassed(56360)){
      kP = 0.2;
      pStep(15,1);
    } 

    // hello people bow

    if (moveTimePassed(59920)){
      kP = 0.7;
      pBow(60);
    } 

    if (moveTimePassed(60000)){
      startMove(textSequence1);
    } 

  }

  // --------------------------------
  // MARK: - TEXT SEQUENCE 1 (+10s)

  if (move == textSequence1){
    kP = 0.8;
    pBow(60);

    // hello people bows

    if (moveTimePassed(1500)){
      kP = 1.6;
      pBow(90);
    } 

    if (moveTimePassed(2860)){
      kP = 0.2;
      pBow(70);
    } 

    if (moveTimePassed(3305)){
      kP = 1.8;
      pBow(90);
    } 

    if (moveTimePassed(4500)){
      kP = 0.2;
      pBow(70);
    } 

    if (moveTimePassed(4900)){
      kP = 1.8;
      pBow(90);
    } 

    if (moveTimePassed(6150)){
      kP = 0.2;
      pBow(70);
    } 

    
    if (moveTimePassed(7070)){
      kP = 1.8;
      pBow(90);
    } 

    if (moveTimePassed(8080)){
      kP = 0.2;
      pBow(70);
    } 

    if (moveTimePassed(8700)){
      kP = 1.8;
      pBow(90);
    } 

    if (moveTimePassed(9880)){
      kP = 0.2;
      pBow(70);
    } 
    
    if (moveTimePassed(10600)){
      kP = 1.8;
      pBow(90);
    } 

    if (moveTimePassed(12560)){
      kP = 0.2;
      pBow(70);
    } 

    
    if (moveTimePassed(13640)){
      kP = 1.8;
      pBow(90);
    } 

    if (moveTimePassed(14680)){
      kP = 0.2;
      pBow(70);
    } 

    // wiggle

    if (moveTimePassed(15800)){
      kP = 1;
      rTargetPositionDegrees = 90;
      lTargetPositionDegrees = 60;
    } 

    if (moveTimePassed(16130)){
      kP = 1;
      rTargetPositionDegrees = 60;
      lTargetPositionDegrees = 90;
    } 

    if (moveTimePassed(16430)){
      kP = 1;
      rTargetPositionDegrees = 90;
      lTargetPositionDegrees = 60;
    } 

    if (moveTimePassed(17680)){
      kP = 1;
      rTargetPositionDegrees = 60;
      lTargetPositionDegrees = 90;
    } 

    if (moveTimePassed(17940)){
      kP = 1;
      rTargetPositionDegrees = 90;
      lTargetPositionDegrees = 60;
    } 

    if (moveTimePassed(18230)){
      kP = 1;
      rTargetPositionDegrees = 60;
      lTargetPositionDegrees = 90;
    } 

    if (moveTimePassed(19090)){
      kP = 1.2;
      pBow(90);
    } 

    if (moveTimePassed(21700)){
      kP = 0.4;
      pBow(15);
    } 

    // but wait

    if (moveTimePassed(56770)){
      kP = 1.8;
      pBow(45);
    } 

    if (moveTimePassed(58070)){
      kP = 0.2;
      pStand();
    } 


    if (moveTimePassed(61000)){
      kP = 1;
      pStand();
    } 

    if (moveTimePassed(70000)){
      startMove(musicSequence0);
    } 

  }

  // --------------------------------
  // MARK: - MUSIC SEQUENCE 0 intro

  if (move == musicSequence0){

    // bow
    kP = 1;
    pBow(10);

    if (moveTimePassed(2200)){
      kP = 1;
      pStand();
    }

    // raise leg, walk.

    if (moveTimePassed(6300)){
      kP = 1;
      pKick(45, 1);
    }

    if (moveTimePassed(8670)){
      kP = 1.4;
      pStep(15, 1);
    }

    if (moveTimePassed(9570)){
      kP = 1.4;
      pStep(15, 0);
    }

    if (moveTimePassed(10560)){
      kP = 1.4;
      pStep(15, 1);
    }

    if (moveTimePassed(11470)){
      kP = 1.6;
      pStand();
    }

    // raise leg, walk.

    if (moveTimePassed(14600)){
      kP = 1.2;
      pKick(55, 0);
    }

    if (moveTimePassed(16550)){
      kP = 1.4;
      pStep(20, 0);
    }

    if (moveTimePassed(17600)){
      kP = 1.4;
      pStep(20, 1);
    }

    if (moveTimePassed(18650)){
      kP = 1.4;
      pStep(20, 0);
    }

    if (moveTimePassed(19600)){
      kP = 1.6;
      pStand();
    }
    
    // breathe

    if (moveTimePassed(20700)){
      kP = 0.8;
      pBow(15);
    }
    if (moveTimePassed(22800)){
      kP = 1;
      pBow(4);
    }

    // walk, pirouette

    if (moveTimePassed(24800)){
      kP = 1.4;
      pStep(20, 1);
    }

    if (moveTimePassed(25900)){
      kP = 1.4;
      pStep(20, 0);
    }

    if (moveTimePassed(26900)){
      kP = 1.4;
      pStep(20, 1);
    }

    if (moveTimePassed(27900)){
      kP = 1.4;
      pStep(20, 0);
    }

    if (moveTimePassed(28900)){
      kP = 1.8;
      pStand();
    }
    // pirouette

    if (moveTimePassed(29900)){
      kP = 3;
      rTargetPositionDegrees = 200;
      lTargetPositionDegrees = 170;
    }

    if (moveTimePassed(30900)){
      kP = 2;
      pKick(90);
    }
    if (moveTimePassed(31350)){
      kP = 2;
      pBow(10);
    }
    if (moveTimePassed(31700)){
      kP = 1.8;
      pStand();
    }

    // breathe
    if (moveTimePassed(32870)){
      kP = 0.8;
      pBow(15);
    }
    if (moveTimePassed(34950)){
      kP = 1;
      pBow(4);
    }

    // jump

    if (moveTimePassed(37150)){
      kP = 0.6;
      pBow(45);
    }
    if (moveTimePassed(38280)){
      kP = 4;
      pBow(-10);
    }
    if (moveTimePassed(39000)){
      kP = 2;
      pBow(10);
    }
    if (moveTimePassed(40200)){
      kP = 0.8;
      pStand();
    }


    // step step flip
    if (moveTimePassed(43550)){
      kP = 1.4;
      pStep(20, 1);
    }

    if (moveTimePassed(44600)){
      kP = 1.6;
      pStep(10, 0);
    }

    if (moveTimePassed(45900)){
      kP = 3;
      rTargetPositionDegrees = 90;
      lTargetPositionDegrees = 190;
    }

    if (moveTimePassed(46100)){
      kP = 2;
      pStep(90, 1);
    }

    if (moveTimePassed(46900)){
      kP = 1.5;
      pBow(20);
    }

    if (moveTimePassed(49840)){
      kP = 0.8;
      pStand();
    }

    // again step step flip

    if (moveTimePassed(54090)){
      kP = 1.4;
      pStep(20, 1);
    }

    if (moveTimePassed(55120)){
      kP = 1.6;
      pStep(10, 0);
    }

    if (moveTimePassed(56500)){
      kP = 3;
      rTargetPositionDegrees = 90;
      lTargetPositionDegrees = 190;
    }

    if (moveTimePassed(56700)){
      kP = 2;
      pStep(90, 1);
    }

    if (moveTimePassed(57500)){
      kP = 1.5;
      pBow(20);
    }

    if (moveTimePassed(58500)){
      kP = 0.8;
      pStand();
    }



    if (moveTimePassed(60000)){
      startMove(musicSequence1);
    }

  }

  // --------------------------------
  // MARK: - MUSIC SEQUENCE 1 yoga

  if (move == musicSequence1){

    kP = 0.8;
    pStand();

        // bow

    if (moveTimePassed(600)){
      kP = 0.5;
      pBow(45);
    }

    if (moveTimePassed(3160)){
      kP = 0.8;
      pStand();
    }

    // snoek

    // fall to bird
    if (moveTimePassed(8200)){
      kP = 1.8;
      pBow(25);
    }


    if (moveTimePassed(9200)){
      kP = 1.2;
      pBow(15);
    }
    if (moveTimePassed(11200)){
      kP = 2;
      pStand();
    }

    // swimming
    if (moveTimePassed(12100)){
      kP = 2;
      pKick(-20, 1);
    }
    if (moveTimePassed(13000)){
      kP = 0.5;
      pStand();
    }
    if (moveTimePassed(14000)){
      kP = 2;
      pKick(-20, 0);
    }
    if (moveTimePassed(15000)){
      kP = 0.5;
      pStand();
    }
    if (moveTimePassed(15985)){
      kP = 2;
      pKick(-20, 1);
    }
    if (moveTimePassed(17000)){
      kP = 0.5;
      pStand();
    }
    if (moveTimePassed(17890)){
      kP = 2;
      pKick(-20, 0);
    }
    if (moveTimePassed(18900)){
      kP = 0.5;
      pStand();
    }
    if (moveTimePassed(19750)){
      kP = 2;
      pKick(-20, 1);
    }
    if (moveTimePassed(20800)){
      kP = 0.5;
      pStand();
    }
    if (moveTimePassed(21665)){
      kP = 2;
      pKick(-20, 0);
    }
    if (moveTimePassed(22650)){
      kP = 0.5;
      pStand();
    }

     // cloth hanger
    if (moveTimePassed(25240)){
      kP = 0.3;
      pBow(90);
    }
    if (moveTimePassed(28800)){
      kP = 2;
      pBow(76);
    }

    // kick naar bolkje

    if (moveTimePassed(29860)){
      kP = 1.2;
      pKick(90, 1);
    }
    if (moveTimePassed(33270)){
      kP = 0.7;
      pStep(75, 1);
    }
    if (moveTimePassed(35460)){
      kP = 1;
      pStep(60, 1);
    }
    if (moveTimePassed(36000)){
      kP = 1.2;
      pStep(50, 1);
    }
    if (moveTimePassed(36500)){
      kP = 1;
      pStep(35, 1);
    }
    if (moveTimePassed(37000)){
      kP = 1.8;
      pStep(10, 1);
    }


    // swim in bolk

    if (moveTimePassed(38500)){
      kP = 1;
      pStep(10, 0);
    }
    if (moveTimePassed(40000)){
      kP = 1;
      pStep(10, 1);
    }
    if (moveTimePassed(40800)){
      kP = 1.2;
      pStep(10, 0);
    }
    if (moveTimePassed(41600)){
      kP = 1.2;
      pStep(10, 1);
    }

    // to knees
    if (moveTimePassed(43600)){
      kP = 1;
      pStep(10, 0);
    }

    if (moveTimePassed(44400)){
      kP = 0.5;
      pStep(45, 0);
    }
    if (moveTimePassed(45200)){
      kP = 0.5;
      pStep(75, 0);
    }
    if (moveTimePassed(46000)){
      kP = 0.7;
      pStep(90, 0);
    }
    if (moveTimePassed(48590)){
      kP = 0.5;
      pKick(90, 1);
    }
    if (moveTimePassed(52600)){
      kP = 0.6;
      pBow(90);
    }

    if (moveTimePassed(60000)){
      startMove(musicSequence2);
    }

  }

  // --------------------------------
  // MARK: - MUSIC SEQUENCE 2 floor
  if (move == musicSequence2){
    kP = 0.6;
    pBow(90);

    // back to bird

    if (moveTimePassed(450)){
      kP = 1.4;
      pKick(45, 1);
    }
    if (moveTimePassed(950)){
      kP = 1.4;
      pKick(90, 1);
    }
    if (moveTimePassed(4600)){
      kP = 1;
      pStand();
    }
    if (moveTimePassed(9400)){
      kP = 2.2;
      pStand();
    }

    // back to standing
    if (moveTimePassed(11140)){
      kP = 2;
      pBow(15);
    }

    if (moveTimePassed(19870)){
      kP = 2;
      pBow(10);
    }
    if (moveTimePassed(22000)){
      kP = 1.8;
      pBow(5);
    }
    if (moveTimePassed(24300)){
      kP = 1.6;
      pStand();
    }

    // val naar achteren

    if (moveTimePassed(32470)){
      kP = 1.6;
      pBow(-15);
    }

    if (moveTimePassed(33470)){
      kP = 1;
      pStand();
    }

    if (moveTimePassed(38880)){
      kP = 1.6;
      pKick(90, 1);
    }

    if (moveTimePassed(39400)){
      kP = 2;
      pKick(90, 1);
    }

    if (moveTimePassed(43740)){
      kP = 1.2;
      pKick(80, 1);
    }
    if (moveTimePassed(43840)){
      kP = 1.2;
      pKick(70, 1);
    }
    if (moveTimePassed(43940)){
      kP = 1.0;
      pKick(60, 1);
    }
    if (moveTimePassed(44040)){
      kP = 1.0;
      pKick(40, 1);
    }
    if (moveTimePassed(44140)){
      kP = 1.0;
      pKick(20, 1);
    }

    if (moveTimePassed(44200)){
      kP = 1;
      pStand();
    } 

    // zit

    if (moveTimePassed(46870)){
      kP = 0.2;
      pBow(45);
    } 

    if (moveTimePassed(47250)){
      kP = 0.8;
      pBow(90);
    } 

    // lig

    if (moveTimePassed(56370)){
      kP = 0.4;
      pBow(25);
    } 
    if (moveTimePassed(57370)){
      kP = 0.4;
      pStand();
    } 

    // rechts omhoog

    if (moveTimePassed(59700)){
      kP = 0.6;
      pKick(50);
    } 

    if (moveTimePassed(60000)){
      startMove(musicSequence3);
    } 
  }

  // --------------------------------
  // MARK: - MUSIC SEQUENCE 3 floor pt2

  if (move == musicSequence3){
    kP = 0.6;
    pKick(50);

    if (moveTimePassed(600)){
      kP = 0.85;
      pKick(90);
    } 

    // trap naar split

    if (moveTimePassed(7600)){
      kP = 0.5;
      rTargetPositionDegrees = 90;
      lTargetPositionDegrees = 45;
    } 
    if (moveTimePassed(8200)){
      kP = 0.5;
      pBow(90);
    } 

    if (moveTimePassed(8800)){
      kP = 0.6;
      pStep(80, 0);
    } 
    if (moveTimePassed(9800)){
      kP = 0.8;
      pStep(90, 0);
    } 
    if (moveTimePassed(10100)){
      kP = 1.2;
      pStep(90, 0);
    } 

    // split opduwen

    if (moveTimePassed(17500)){
      kP = 3;
      pStep(45, 0);
    } 

    if (moveTimePassed(18000)){
      kP = 2;
      pStep(45, 0);
    } 

    if (moveTimePassed(20720)){
      kP = 0.6;
      pStep(90, 0);
    } 
    if (moveTimePassed(24430)){
      kP = 0.7;
      pStep(90, 0);
    } 

    // split wissel

    if (moveTimePassed(31580)){
      kP = 0.5;
      pKick(90, 0);
    } 

    if (moveTimePassed(32580)){
      kP = 0.6;
      pStep(80, 1);
    } 

    if (moveTimePassed(33100)){
      kP = 0.6;
      pStep(90, 1);
    } 

    // naar rug

    if (moveTimePassed(35650)){
      kP = 1;
      pBow(30);
    }

    if (moveTimePassed(36200)){
      kP = 0.6;
      rTargetPositionDegrees = 200;
      lTargetPositionDegrees = 150;
    }

    if (moveTimePassed(37200)){
      kP = 1.2;
      rTargetPositionDegrees = 180;
      lTargetPositionDegrees = 170;
    }

    if (moveTimePassed(37500)){
      kP = 1.2;
      pStand();
    }

    // rol naar zij

    if (moveTimePassed(39700)){
      kP = 1.4;
      pStep(25, 1);
    }

    if (moveTimePassed(41550)){
      kP = 1;
      rTargetPositionDegrees = 120;
      lTargetPositionDegrees = 210;
    }

    if (moveTimePassed(43700)){
      kP = 1.2;
      rTargetPositionDegrees = 200;
      lTargetPositionDegrees = 190;
    }

    if (moveTimePassed(44630)){
      kP = 1;
      rTargetPositionDegrees = 130;
      lTargetPositionDegrees = 200;
    }

    // buik
    if (moveTimePassed(44630)){
      kP = 0.8;
      pBow(-10);
    }


    // been omhoog

    if (moveTimePassed(51950)){
      kP = 0.7;
      pKick(-80, 1);
    }
    if (moveTimePassed(54020)){
      kP = 0.8;
      pBow(-80);
    }
    if (moveTimePassed(55020)){
      kP = 1.2;
      pBow(-85);
    }

    // staan

    if (moveTimePassed(58090)){
      kP = 0.5;
      pStand();
    }

    if (moveTimePassed(59090)){
      kP = 1;
      pStand();
    }

    if (moveTimePassed(60000)){
      startMove(musicSequence4);
    }

  }

  // --------------------------------
  // MARK: - MUSIC SEQUENCE 4 standing acro

  if (move == musicSequence4){
    kP = 1.2;
    pStand();

    // walk
    if (moveTimePassed(185)){
      kP = 1.4;
      pStep(20, 1);
    }
    if (moveTimePassed(1250)){
      pStep(20, 0);
    }

    if (moveTimePassed(2215)){
      pStep(20, 1);
    }
    if (moveTimePassed(3265)){
      pStep(20, 0);
    }

    if (moveTimePassed(4305)){
      pStep(20, 1);
    }
    if (moveTimePassed(5350)){
      pStep(20, 0);
    }

    if (moveTimePassed(6435)){
      kP = 1.7;
      pStand();
    }

    // rug rol

    if (moveTimePassed(9790)){
      kP = 0.8;
      pBow(90);
    }

    if (moveTimePassed(11850)){
      kP = 1.3;
      pBow(90);
    }


    if (moveTimePassed(15150)){
      kP = 0.6;
      pStand();
    }

    // shoulder sit

    if (moveTimePassed(21740)){
      kP = 0.8;
      pBow(15);
    }
    if (moveTimePassed(22780)){
      kP = 0.8;
      pBow(30);
    }
    if (moveTimePassed(24380)){
      kP = 1;
      pBow(70);
    }

    // uitbouw

    if (moveTimePassed(28780)){
      kP = 0.8;
      pKick(70, 0);
    }

    if (moveTimePassed(30885)){
      kP = 1;
      pStand();
    }

    if (moveTimePassed(32120)){
      kP = 1.5;
      pBow(-20);
    }

    if (moveTimePassed(35436)){
      kP = 1.2;
      pStand();
    }

    if (moveTimePassed(36975)){
      kP = 1;
      lTargetPositionDegrees = 170;
      rTargetPositionDegrees = 162;
    }

    if (moveTimePassed(37600)){
      kP = 1.8;
      pStand();
    }

    // schouder snoek

    if (moveTimePassed(40500)){
      kP = 1.8;
      pKick(-14, 1);
    }

    if (moveTimePassed(41100)){
      kP = 1.6;
      pKick(80, 1);
    }

    if (moveTimePassed(41400)){
      kP = 1.4;
      pStep(90, 1);
    }

    if (moveTimePassed(42100)){
      kP = 1.0;
      pBow(30);
    }

    if (moveTimePassed(43000)){
      kP = 1.0;
      pBow(-10);
    }

    // kopstand

    if (moveTimePassed(48335)){
      kP = 0.2;
      pBow(50);
    }
    if (moveTimePassed(48800)){
      kP = 0.5;
      lTargetPositionDegrees = 110;
      rTargetPositionDegrees = 90;
    }


    if (moveTimePassed(49999)){
      startMove(musicSequence5);
      // jump in time, 50 second sequence to match with full act sound timing
    }

  }

  // --------------------------------
  // MARK: - MUSIC SEQUENCE 5 fall

  if (move == musicSequence5){

    // starts going into headstand split

    if (moveTimePassed(1925)){
      kP = 1.2;
      lTargetPositionDegrees = 150;
      rTargetPositionDegrees = 90;
    }

    if (moveTimePassed(3600)){
      kP = 0.8;
      lTargetPositionDegrees = 190;
      rTargetPositionDegrees = 90;
    }

    if (moveTimePassed(3900)){
      kP = 0.8;
      lTargetPositionDegrees = 230;
      rTargetPositionDegrees = 90;
    }

    if (moveTimePassed(4400)){
      kP = 1;
      lTargetPositionDegrees = 260;
      rTargetPositionDegrees = 90;
    }

    if (moveTimePassed(4700)){
      kP = 1.2;
      lTargetPositionDegrees = 270;
      rTargetPositionDegrees = 90;
    }

    

    if (moveTimePassed(10335)){
      kP = 1.3;
      lTargetPositionDegrees = 267;
      rTargetPositionDegrees = 120;
    }

    // coming down

    if (moveTimePassed(14425)){
      kP = 0.8;
      pBow(60);
    }

    if (moveTimePassed(15646)){
      kP = 1;
      pBow(85);
    }

    if (moveTimePassed(16432)){
      kP = 1;
      pStand();
    }
    if (moveTimePassed(17255)){
      kP = 1.6;
      pStand();
    }

    // fall

    if (moveTimePassed(19309)){
      kP = 1.6;
      pBow(-10);
    } 
    if (moveTimePassed(21050)){
      kP = 0.6;
      pBow(10);
    } 
    if (moveTimePassed(20709)){
      kP = 1.2;
      pBow(10);
    } 

    // I don't care 2

    if (moveTimePassed(52500)){
      kP = 1.2;

      lTargetPositionDegrees = 150;
      rTargetPositionDegrees = 175;
    } 

    if (moveTimePassed(53700)){
      kP = 1.2;

      lTargetPositionDegrees = 205;
      rTargetPositionDegrees = 170;
    } 
    if (moveTimePassed(54100)){
      kP = 0.4;
      lTargetPositionDegrees = 185;
      rTargetPositionDegrees = 175;
    } 

    if (moveTimePassed(60000)){
      startMove(musicSequence6);
    } 

  }

  // --------------------------------
  // MARK: - MUSIC SEQUENCE 6 floor dialog

  if (move == musicSequence6){

    // I want to see them

    if (moveTimePassed(2450)){
      kP = 0.9;
      pBow(25);
    } 
    if (moveTimePassed(4340)){
      kP = 0.9;
      pBow(5);
    } 

    if (moveTimePassed(8025)){
      kP = 0.9;
      pKick(40);
    } 
    if (moveTimePassed(8800)){
      kP = 0.8;
      pStand();
    } 


    if (moveTimePassed(14500)){
      kP = 1.5;
      pKick(-10);
    } 
    if (moveTimePassed(14930)){
      kP = 1.6;
      pKick(40);
    } 
    if (moveTimePassed(15280)){
      kP = 1.2;
      pStep(12,1);
    } 

    // hello people

    if (moveTimePassed(22222)){
      kP = 1.4;
      pStand();
    } 

    if (moveTimePassed(29635)){
      kP = 1;
      pKick(50);
    } 
    if (moveTimePassed(31735)){
      kP = 1;
      pStand();
    } 

    // I'm ready

    if (moveTimePassed(40575)){
      kP = 1;
      pBow(15);
    } 
    if (moveTimePassed(41600)){
      kP = 1.2;
      pStand();
    } 

    // finale


    if (moveTimePassed(55485)){
      kP = 1.2;
      pKick(50, 1);
    } 

    if (moveTimePassed(57240)){
      kP = 1.2;

      lTargetPositionDegrees = 185;
      rTargetPositionDegrees = 130;
    } 
    if (moveTimePassed(59400)){
      kP = 1.2;

      pBow(20);
    } 

    if (moveTimePassed(60000)){
      startMove(musicSequence7);
    } 


  }

  // --------------------------------
  // MARK: - MUSIC SEQUENCE 7 finale

  if (move == musicSequence7){

    kP = 1.2;

      pBow(20);
    
    // if (moveTimePassed(1765)){
    //   kP = 1.3;

    //   lTargetPositionDegrees = 100;
    //   rTargetPositionDegrees = 160;
    // } 

    // if (moveTimePassed(2360)){
    //   kP = 1.6;

    //   lTargetPositionDegrees = 110;
    //   rTargetPositionDegrees = 170;
    // } 

    if (moveTimePassed(3435)){
      kP = 1.3;

      pStand();
    } 

    if (moveTimePassed(13204)){
      kP = 1;

      pKick(10,1);
    } 

    if (moveTimePassed(13510)){
      kP = 1;

      pKick(20,1);
    } 

    if (moveTimePassed(13800)){
      kP = 1;

      pKick(30,1);
    } 

    if (moveTimePassed(14085)){
      kP = 1;

      pKick(40,1);
    } 

    if (moveTimePassed(14390)){
      kP = 1.2;

      pKick(50,1);
    } 

    if (moveTimePassed(14650)){
      kP = 1.4;

      pKick(75,1);
    } 

    if (moveTimePassed(14860)){
      kP = 2.2;

      pKick(90,1);
    } 

    // stand

    if (moveTimePassed(16426)){
      kP = 0.4;

      pStand();
    } 

    if (moveTimePassed(19460)){
      kP = 1.2;

      pStand();
    } 

    // bows

    if (moveTimePassed(23188)){
      kP = 0.6;

      pBow(80);
    } 

    if (moveTimePassed(24741)){
      kP = 1.0;

      pStand();
    } 

    // walk

    if (moveTimePassed(28740)){
      kP = 1.6;
      pStep(15, 1);
    } 
    if (moveTimePassed(29245)){
      kP = 1.5;
      pStep(15, 0);
    } 
    if (moveTimePassed(29995)){
      kP = 1.5;
      pStep(15, 1);
    } 
    if (moveTimePassed(30680)){
      kP = 1.7;
      pStand();
    } 

    if (moveTimePassed(32190)){
      kP = 0.6;

      pBow(80);
    } 

    if (moveTimePassed(33765)){
      kP = 1.5;

      pStand();
    } 

    // mini bow

    if (moveTimePassed(36975)){
      kP = 0.8;

      pBow(10);
    } 

    if (moveTimePassed(38170)){
      kP = 1.2;

      pBow(1);
    } 

    // hug

    if (moveTimePassed(41885)){
      kP = 2;

      pBow(20);
    } 

    if (moveTimePassed(44625)){
      kP = 1;

      pStand();
    } 

    // walk

    if (moveTimePassed(49526)){
      kP = 1.5;
      pStep(17, 1);
    } 
    if (moveTimePassed(50250)){
      kP = 1.5;
      pStep(17, 0);
    } 
    if (moveTimePassed(51025)){
      kP = 1.5;
      pStep(17, 1);
    } 
    if (moveTimePassed(51740)){
      kP = 1.5;
      pStep(17, 0);
    } 
    if (moveTimePassed(52515)){
      kP = 1.5;
      pStep(17, 1);
    } 
    if (moveTimePassed(53245)){
      kP = 1.5;
      pStep(17, 0);
    } 
    if (moveTimePassed(53960)){
      kP = 1.5;
      pStep(17, 1);
    } 
    if (moveTimePassed(54690)){
      kP = 1.5;
      pStand();
    } 


    if (moveTimePassed(60000)){
        startMove(musicSequence8);
    }

  }

  // --------------------------------
  // MARK: - MUSIC SEQUENCE 8 toilet

  if (move == musicSequence8){
    
    kP = 1.2;
    pStand();

    if (moveTimePassed(8090)){
      kP = 1.5;
      pKick(80,1);
    } 
    if (moveTimePassed(8790)){
      kP = 1.5;
      pKick(80,0);
    } 
    if (moveTimePassed(9480)){
      kP = 1.5;
      pBow(80);
    } 

    if (moveTimePassed(10320)){
      kP = 0.2;
      pStand();
    } 


    // splits

    if (moveTimePassed(17500)){
      kP = 1.2;
      pStep(80,1);
    } 
    if (moveTimePassed(18400)){
      kP = 1.2;
      pStep(80,0);
    } 
    if (moveTimePassed(19300)){
      kP = 1.2;
      pStep(90,1);
    } 
    if (moveTimePassed(20200)){
      kP = 1.2;
      pStep(90,0);
    } 

    // forward backward

    if (moveTimePassed(21475)){
      kP = 1.0;
      pBow(80);
    } 
    if (moveTimePassed(22200)){
      kP = 1.0;
      pBow(-80);
    } 

    if (moveTimePassed(22600)){
      kP = 1.0;
      pBow(90);
    } 
    if (moveTimePassed(23455)){
      kP = 1.0;
      pBow(-80);
    } 
    if (moveTimePassed(23725)){
      kP = 1.0;
      pBow(90);
    } 

    if (moveTimePassed(24900)){
      kP = 0.3;
      pStand();
    } 

  }

}




bool moveTimePassed(uint32_t time){
  if ((moveTimer + time) > millis()){
    return false;
  }
  return true;
}


// --------------
// MARK: - Positions


uint16_t withinLimits(uint16_t position){
  return min(max(position, forwardLimit),backwardLimit);
}









// --------------
// MARK: - Print

void printAll()
{
  if (printTimer < millis())
  {
    // Serial.print(analogRead(SLIDER_L_ARM));
    // Serial.print(",");
    // Serial.print(analogRead(JOYSTICK_L_X));
    // Serial.print(",");
    // Serial.print(analogRead(JOYSTICK_R_X));
    // Serial.print(",");
    // Serial.print(analogRead(JOYSTICK_R_Y));
    // Serial.print(",");
    // Serial.print(encoderPos);
    // Serial.print(",");

    // Serial.println(analogRead(BATTERY_V));

    Serial.print(ads1115.readADC_SingleEnded(0));
    Serial.print(",");
    Serial.print(ads1115.readADC_SingleEnded(1));
    Serial.print(",");
    Serial.print(ads1115.readADC_SingleEnded(2));
    Serial.print(",");
    Serial.println(ads1115.readADC_SingleEnded(3));

    printTimer = millis() + 10; // print every 10 ms
  }
}
