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

uint8_t robotAddress[] = {0x94, 0xE6, 0x86, 0x00, 0xE0, 0xD0};

#define LED_R 12
#define LED_G 14
#define LED_B 27

#define SLIDER_L_ARM 32
#define SLIDER_L_LEG 34
#define SLIDER_R_ARM 36
#define SLIDER_R_LEG 39

#define ENCODER_A 26
#define ENCODER_B 25
#define ENCODER_SW 33

#define BUZZER 13

#define I2CMATRIX 0x38


#define BATTERY_V 35

typedef struct struct_data_in {
  char hi[6];
  uint16_t pot1;
  uint16_t pot2;
} struct_data_in;

// testing with 2 slide pot values
uint16_t pot1;
uint16_t pot2;
char hi[6];

typedef struct struct_data_out {
  uint16_t pot1;
  uint16_t pot2;
} struct_data_out;

struct_data_in dataIn;
struct_data_out dataOut;

esp_now_peer_info_t peerInfo;

Adafruit_ADS1115 ads1115; // adc converter

// is this really needed?
String success;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }

  //Note that too short interval between sending two ESP-NOW data may lead to disorder of sending callback function. So, it is recommended that sending the next ESP-NOW data after the sending callback function of the previous sending has returned.
  // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html
}


void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&dataIn, incomingData, sizeof(dataIn));
  Serial.print("Bytes received: ");
  Serial.println(len);
  pot1 = dataIn.pot1;
  pot2 = dataIn.pot2;
  strcpy(hi, dataIn.hi);
}

// keypad matrix

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

// Adafruit_ADS1115 ads1115;	

LiquidCrystal_I2C lcd(0x27,20,4);

// buzzer vars
uint32_t buzzerTimer = 0;

void setBuzzer(uint16_t time=100){
  buzzerTimer = millis() + time;
}
void updateBuzzer(){
  if (buzzerTimer > millis()){
    digitalWrite(BUZZER, HIGH);
  } else {
    digitalWrite(BUZZER, LOW);
  }
}

// encoder vars
ESP32Encoder encoder;
int16_t encoderPos = 0;
bool encoderUp = false;
bool encoderDown = false;
bool encoderSwDown = false;
bool encoderSwPressed = false;

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

float batteryVoltage;
int8_t batteryPercent;
RunningMedian batterySamples = RunningMedian(64); 

uint32_t batteryAlarmTimer = 0;

void updateBattery(){
  float vMax = 4.2;
  float vMin = 3.65;

  batterySamples.add(analogRead(BATTERY_V));

  batteryVoltage = (batterySamples.getMedian() / 4096.0) * 3.3 * 2;
  batteryPercent = (batteryVoltage - vMin) / ((vMax - vMin) / 100.0);

  // low battery alarm
  // if (batteryPercent < 10 && batteryAlarmTimer < millis()){
  //   setBuzzer(300);
  //   batteryAlarmTimer = millis() + 600;
  // }

}

void setLCD(){
  lcd.clear();

  lcd.setCursor(0,0);   
  lcd.print("SLA: ");
  lcd.setCursor(0,1);   
  lcd.print("SLL: ");
  lcd.setCursor(0,2);   
  lcd.print("SRA: ");
  lcd.setCursor(0,3);   
  lcd.print("SRL: ");
}

uint32_t lcdTimer = 0;

//TODO: Create multiple modes, that can be combined (battery + debug, or battery + menu). Switching mode triggers clear.
void updateLCD(){

  if (lcdTimer < millis()){

    
    char slideLA[5]; // amount of characters in string + 1
    char slideLL[5];
    char slideRA[5];
    char slideRL[5];
    sprintf(slideLA, "%04d",analogRead(SLIDER_L_ARM)); // 4 digit string with leading zeros
    sprintf(slideLL, "%04d",analogRead(SLIDER_L_LEG));
    sprintf(slideRA, "%04d",analogRead(SLIDER_R_ARM));
    sprintf(slideRL, "%04d",analogRead(SLIDER_R_LEG));


    lcd.setCursor(4,0);   
    lcd.print(slideLA);
    lcd.setCursor(4,1);   
    lcd.print(slideLL);
    lcd.setCursor(4,2);   
    lcd.print(slideRA);
    lcd.setCursor(4,3);   
    lcd.print(slideRL);

    // print ads
    lcd.setCursor(9,0);   
    lcd.print(ads1115.readADC_SingleEnded(0));
    lcd.setCursor(9,1);   
    lcd.print(ads1115.readADC_SingleEnded(1));
    lcd.setCursor(9,2);   
    lcd.print(ads1115.readADC_SingleEnded(2));
    lcd.setCursor(9,3);   
    lcd.print(ads1115.readADC_SingleEnded(3));

    char key = keypad.getKey();
    if (key != NO_KEY){
      lcd.setCursor(15,0);
      lcd.print(key);
    } 
    
    lcd.setCursor(18,0);
    char batPerc[3];
    sprintf(batPerc, "%02d", batteryPercent);
    lcd.print(batPerc);

    lcd.setCursor(15,1);
    lcd.print(hi);
    lcd.setCursor(15,2);
    lcd.print(pot1);
    lcd.setCursor(15,3);
    lcd.print(pot2);

    lcdTimer = millis() + 20; // update every 20 milliseconds
  }

}




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

uint32_t printTimer = 0;

void printAll(){
  if (printTimer < millis()){
    // Serial.print(analogRead(SLIDER_L_ARM));
    // Serial.print(",");
    // Serial.print(analogRead(SLIDER_L_LEG));
    // Serial.print(",");
    // Serial.print(analogRead(SLIDER_R_ARM));
    // Serial.print(",");
    // Serial.print(analogRead(SLIDER_R_LEG));
    // Serial.print(",");
    // Serial.print(encoderPos);
    // Serial.print(",");

    // Serial.println(analogRead(BATTERY_V));

    // Serial.print(ads1115.readADC_SingleEnded(0));
    // Serial.print(",");
    // Serial.print(ads1115.readADC_SingleEnded(1));
    // Serial.print(",");
    // Serial.print(ads1115.readADC_SingleEnded(2));
    // Serial.print(",");
    // Serial.println(ads1115.readADC_SingleEnded(3));

    printTimer = millis() + 10; // print every 10 ms
  }

}

void setup() {

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  pinMode(BUZZER, OUTPUT);
  pinMode(ENCODER_SW, INPUT_PULLUP);

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

  WiFi.mode(WIFI_MODE_STA);

  Serial.println("setup done");
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

}

void loop() {
  updateBattery();
  updateEncoder(); // update encoder pos, and set encoder up/down bools for one loop
  updateBuzzer(); // play buzzer if buzzertimer is high
  updateLCD();

  if (encoderUp){
    setBuzzer();
  }
  if (encoderDown){
    setBuzzer(4);
  }
  if (encoderSwPressed){
    setBuzzer(200);
  }

  if (encoderSwDown){
    ledWhite();
  } else {
    ledBlue();
  }

  dataOut.pot1 = analogRead(SLIDER_L_ARM);
  dataOut.pot2 = analogRead(SLIDER_L_LEG);

  esp_now_send(robotAddress, (uint8_t *) &dataOut, sizeof(dataOut));

  delay(100); // because haven't tested ESP_NOW speed yet..
  // printAll();


}