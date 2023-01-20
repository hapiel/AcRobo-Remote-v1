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
#include <Keypad_I2C.h> // https://github.com/joeyoung/arduino_keypads/tree/master/Keypad_I2C
#include <Keypad.h> // https://playground.arduino.cc/Code/Keypad/
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

#define JOYSTICK_L_Y 32
#define JOYSTICK_L_X 34
#define JOYSTICK_R_X 36
#define JOYSTICK_R_Y 39

#define I2CMATRIX 0x38

// ---------------
// MARK: - FORWARD DECLARATIONS in order of document 

// ADC
Adafruit_ADS1115 ads1115; // adc converter

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

void setBuzzer(uint16_t time=100);
void updateBuzzer();

// DATA ESPNOW

uint32_t adsTimer;
uint32_t dataTimer = 0;
uint8_t robotAddress[] = {0x94, 0xE6, 0x86, 0x00, 0xE0, 0xD0}; // mac address of robot
typedef struct struct_data_in {
  // test data
  char hi[6];
  uint16_t pot1;
  uint16_t pot2;
} struct_data_in;


// data out:

typedef struct struct_data_out {
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

} struct_data_out;

struct_data_in dataIn;
struct_data_out dataOut;

esp_now_peer_info_t peerInfo;
bool lastPackageSuccess = false;

void collectData();
void sendData();
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);

// ENCODER
ESP32Encoder encoder;
int16_t encoderPos = 0;
bool encoderUp = false;
bool encoderDown = false;
bool encoderSwDown = false;
bool encoderSwPressed = false;

void updateEncoder();

// KEYPAD MATRIX
const byte ROWS = 4; 
const byte COLS = 4; 
char hexaKeys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
byte rowPins[ROWS] = {0, 1, 2, 3}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {4, 5, 6, 7}; //connect to the column pinouts of the keypad
Keypad_I2C keypad( makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS, I2CMATRIX); 

// LCD
LiquidCrystal_I2C lcd(0x27,20,4);
uint32_t lcdTimer = 0;

void setLCD();
void updateLCD();

// LED
void ledRed(uint8_t);
void ledGreen(uint8_t);
void ledBlue(uint8_t);
void ledYellow(uint8_t);
void ledWhite(uint8_t);
void updateLED();

// PRINT
uint32_t printTimer = 0;

void printAll();


// END FORWARD DECLARATIONS
// **********************************

// MARK: -Setup

void setup() {

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
  ESP32Encoder::useInternalWeakPullResistors=UP;
  encoder.attachSingleEdge(ENCODER_A, ENCODER_B);

  ads1115.begin();
  currentAds = 0;
  ads1115.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0, /*continuous=*/false);

  WiFi.mode(WIFI_MODE_STA);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
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
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  Serial.println("setup done");
}

//**********************************
// MARK: -Loop


void loop() {
  updateBattery();
  updateEncoder(); // update encoder pos, and set encoder up/down bools for one loop
  updateBuzzer(); // play buzzer if buzzertimer is high
  updateLCD();
  updateLED();
  readADS();

  collectData();
  // sendData();

  if (encoderUp){
    setBuzzer(20);
    Serial.print("up ");
    Serial.println(millis());
  }
  if (encoderDown){
    setBuzzer(4);
    Serial.print("down ");
    Serial.println(millis());
  }
  if (encoderSwPressed){
    setBuzzer(200);
  }


  // printAll();

}

//**********************************
// MARK: -FUNCTIONS

//--------------------
// MARK: - Ads sliders

void readADS(){
  // based on https://github.com/adafruit/Adafruit_ADS1X15/blob/master/examples/nonblocking/nonblocking.ino


  if (!ads1115.conversionComplete()) {
    return;
  }

  if (currentAds == 0){
    sliderRA = ads1115.getLastConversionResults();
    ads1115.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_1, /*continuous=*/false);
  }
  if (currentAds == 1){
    sliderRL = ads1115.getLastConversionResults();
    ads1115.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_2, /*continuous=*/false);
  }
  if (currentAds == 2){
    sliderLL = ads1115.getLastConversionResults();
    ads1115.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_3, /*continuous=*/false);
  }
  if (currentAds == 3){
    sliderLA = ads1115.getLastConversionResults();
    ads1115.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0, /*continuous=*/false);
  }

  currentAds += 1;
  currentAds %= 4;

}


// ----------------------------
// MARK: -Battery 

void setModemSleep() {
  WiFi.setSleep(true);
  setCpuFrequencyMhz(80);
  Serial.println("sleep mode enabled");
}

void wakeModemSleep() {
  WiFi.setSleep(false);
  setCpuFrequencyMhz(240);
  Serial.println("sleep mode disabled");
}

void updateBattery(){
  batterySamples.add(analogRead(BATTERY_V));

  batteryPercent = map(batterySamples.getAverage(), 2060, 2370, 0, 100); // 2060 =~ 3.65v, 2370 =~ 4.2v

  // go to modem sleep
  if (digitalRead(LOW_POWER_SW) && !chargingState){
    chargingState = true;
    setModemSleep();
  }
  if (!digitalRead(LOW_POWER_SW) && chargingState){
    chargingState = false;
    wakeModemSleep();
    lcd.init();
    lcd.clear(); 
    setLCD();
  }

  // low battery alarm
  if (batteryPercent < 10 && batteryAlarmTimer < millis() && !chargingState){
    setBuzzer(300);
    batteryAlarmTimer = millis() + 600;
  }
}

// ----------------------------
// MARK: -Buzzer 


void setBuzzer(uint16_t time){
  buzzerTimer = millis() + time;
}
void updateBuzzer(){
  if (buzzerTimer > millis()){
    digitalWrite(BUZZER, HIGH);
  } else {
    digitalWrite(BUZZER, LOW);
  }
}

// ----------------------
// MARK: -Data espnow 


void collectData(){

  // todo: correct for middle position

  uint16_t joyCorrectedLX = analogRead(JOYSTICK_L_X) ;
  uint16_t joyCorrectedLY = analogRead(JOYSTICK_L_Y) ;
  uint16_t joyCorrectedRX = analogRead(JOYSTICK_R_X) ;
  uint16_t joyCorrectedRY = analogRead(JOYSTICK_R_Y) ;

  uint16_t joyCenterLX = 1870;
  uint16_t joyCenterLY = 1860;
  uint16_t joyCenterRX = 1840;
  uint16_t joyCenterRY = 1920;

  if (joyCorrectedLX < joyCenterLX){
    joyCorrectedLX = map(joyCorrectedLX, 0, joyCenterLX, 0, 2047);
  } else {
    joyCorrectedLX = map(joyCorrectedLX, joyCenterLX, 4095, 2048, 4095);
  }

  if (joyCorrectedLY < joyCenterLY){
    joyCorrectedLY = map(joyCorrectedLY, 0, joyCenterLY, 0, 2047);
  } else {
    joyCorrectedLY = map(joyCorrectedLY, joyCenterLY, 4095, 2048, 4095);
  }

  if (joyCorrectedRX < joyCenterRX){
    joyCorrectedRX = map(joyCorrectedRX, 0, joyCenterRX, 0, 2047);
  } else {
    joyCorrectedRX = map(joyCorrectedRX, joyCenterRX, 4095, 2048, 4095);
  }

  if (joyCorrectedRY < joyCenterRY){
    joyCorrectedRY = map(joyCorrectedRY, 0, joyCenterRY, 0, 2047);
  } else {
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

  dataOut.key = keypad.getKey();

  dataOut.batteryPercent = batteryPercent;
}

void sendData(){
  
  if (dataTimer < millis()){
    esp_now_send(robotAddress, (uint8_t *) &dataOut, sizeof(dataOut));
  }
  // send once every 2 ms.
  dataTimer = millis() + 2;
};

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.print("\r\nLast Packet Send Status:\t");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  lastPackageSuccess = status == ESP_NOW_SEND_SUCCESS;

  //Note that too short interval between sending two ESP-NOW data may lead to disorder of sending callback function. So, it is recommended that sending the next ESP-NOW data after the sending callback function of the previous sending has returned.
  // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&dataIn, incomingData, sizeof(dataIn));
  Serial.print("Bytes received: ");
  Serial.println(len);
  // pot1 = dataIn.pot1;
  // pot2 = dataIn.pot2;
  // strcpy(hi, dataIn.hi);
}

// -----------------------
// MARK: - Encoder

void updateEncoder(){
  int16_t newPos = encoder.getCount();
  bool newSw = !digitalRead(ENCODER_SW);

  if (newPos > encoderPos){
    encoderUp = true;
  } else {
    encoderUp = false;
  }

  if (newPos < encoderPos){
    encoderDown = true;
  } else {
    encoderDown = false;
  }

  // switch
  if (newSw == true && encoderSwDown == false){
    encoderSwPressed = true;
  } else {
    encoderSwPressed = false;
  }

  encoderPos = newPos;
  encoderSwDown = newSw;
}

//  -------------------------------
// MARK: - Lcd

void setLCD(){
  lcd.clear();

  lcd.setCursor(0,0);   
  lcd.print("JLX: ");
  lcd.setCursor(0,1);   
  lcd.print("JLY: ");
  lcd.setCursor(0,2);   
  lcd.print("JRX: ");
  lcd.setCursor(0,3);   
  lcd.print("JRY: ");
}

//TODO: Create multiple modes, that can be combined (battery + debug, or battery + menu). Switching mode triggers clear.
void updateLCD(){

  if (lcdTimer > millis()){
    return;
  }
  lcdTimer = millis() + 200; // update every 200 milliseconds

  char joyLX[5]; // amount of characters in string + 1
  char joyLY[5]; 
  char joyRX[5];
  char joyRY[5];
  sprintf(joyLX, "%04d", dataOut.joystickLX);
  sprintf(joyLY, "%04d", dataOut.joystickLY); // 4 digit string with leading zeros
  sprintf(joyRX, "%04d", dataOut.joystickRX);
  sprintf(joyRY, "%04d", dataOut.joystickRY);

  lcd.setCursor(4,0);   
  lcd.print(joyLX);
  lcd.setCursor(4,1);   
  lcd.print(joyLY);
  lcd.setCursor(4,2);   
  lcd.print(joyRX);
  lcd.setCursor(4,3);   
  lcd.print(joyRY);

  char slideLLeg[6]; // amount of characters in string + 1
  char slideLArm[6]; 
  char slideRLeg[6];
  char slideRArm[6];
  sprintf(slideLLeg, "%05d",dataOut.sliderLL);
  sprintf(slideLArm, "%05d",dataOut.sliderLA); // 4 digit string with leading zeros
  sprintf(slideRLeg, "%05d",dataOut.sliderRL);
  sprintf(slideRArm, "%05d",dataOut.sliderRA);

  // print ads
  lcd.setCursor(9,0);   
  lcd.print(slideLLeg);
  lcd.setCursor(9,1);   
  lcd.print(slideLArm);
  lcd.setCursor(9,2);   
  lcd.print(slideRLeg);
  lcd.setCursor(9,3);   
  lcd.print(slideRArm);

  if (dataOut.key != NO_KEY){
    lcd.setCursor(15,0);
    lcd.print(dataOut.key);
  } 
  
  lcd.setCursor(18,0);
  char batPerc[3];
  sprintf(batPerc, "%02d", batteryPercent);
  lcd.print(batPerc);

}

// --------------------------------
// MARK: - Led 

void ledRed(uint8_t brightness = 10){
  analogWrite(LED_R, brightness);
  analogWrite(LED_G, 0);
  analogWrite(LED_B, 0);
}

void ledGreen(uint8_t brightness = 10){
  analogWrite(LED_R, 0);
  analogWrite(LED_G, brightness);
  analogWrite(LED_B, 0);
}

void ledBlue(uint8_t brightness = 10){
  analogWrite(LED_R, 0);
  analogWrite(LED_B, brightness);
  analogWrite(LED_G, 0);
}

void ledYellow(uint8_t brightness = 10){
  analogWrite(LED_R, brightness);
  analogWrite(LED_G, brightness * 0.4);
  analogWrite(LED_B, 0);
}

void ledWhite(uint8_t brightness = 10){
  analogWrite(LED_R, brightness);
  analogWrite(LED_G, brightness);
  analogWrite(LED_B, brightness);
}

void ledOff(){
  analogWrite(LED_R, 0);
  analogWrite(LED_G, 0);
  analogWrite(LED_B, 0);
}

void updateLED(){
  if (chargingState){
    ledYellow(5);
    return;
  }
  if (lastPackageSuccess){
    ledBlue();
  } else {
    ledRed();
  }
}

// --------------
// MARK: - Print

void printAll(){
  if (printTimer < millis()){
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

