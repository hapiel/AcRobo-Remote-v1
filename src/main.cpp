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
  musicSequence0,
  musicSequence1,
  musicSequence2
};

moveList move = stop;

void startMove(moveList);
void updateMoves();
bool moveTimePassed(u32_t time);

// POSITIONS

uint16_t forwardLimit = 90;
uint16_t backwardLimit = 270;

uint16_t withinLimits(uint16_t position);
void pBow(int16_t upperBodyDegrees = 45);
void pStand();
void pStep(int8_t degrees = 20, bool rightFront = true);
void pKick(int8_t degrees = 90, bool rightFront = true);



// PRINT
uint32_t printTimer = 0;

void printAll();

// END FORWARD DECLARATIONS
// **********************************

// MARK: -Setup

void setup()
{

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
      startMove(relax);
    }

    if (keyInput == '5'){
      startMove(stop);
    }

    if (keyInput == '6'){
      startMove(stand);
    }

    if (keyInput == '7'){
      startMove(walk);
    }
    
    if (keyInput == '8'){
      startMove(acroyogaSequence);
    }

    if (keyInput == '9'){
      startMove(jump);
    }

    if (keyInput == 'B'){
      startMove(flip);
    }

    if (keyInput == 'C'){
      startMove(pirouette);
    }

    if (keyInput == '*'){
      startMove(musicSequence0);
    }

    if (keyInput == '0'){
      startMove(musicSequence1);
    }


    updateMoves();
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
  // MARK: - MUSIC SEQUENCE 0 intro

  if (move == musicSequence0){

    // bow
    kP = 1;
    pBow(15);

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

void pBow(int16_t upperBodyDegrees){
  // -90 to 90
  rTargetPositionDegrees = withinLimits(180 - upperBodyDegrees);
  lTargetPositionDegrees = withinLimits(180 - upperBodyDegrees);
}

void pStand(){
  pBow(0);
}

void pStep(int8_t degrees, bool rightFront){
  // for left, can also do negative value in degrees

  int8_t sign = rightFront * 2 - 1;

  rTargetPositionDegrees = withinLimits(180 + degrees * -sign);
  lTargetPositionDegrees = withinLimits(180 + degrees * sign);
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
