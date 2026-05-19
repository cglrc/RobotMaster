#include <Wire.h>
#include "LiquidCrystal_I2C.h"
#include <SPI.h>
#include <MFRC522.h>
#include <Servo.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <IRremote.hpp>
#include <SoftwareSerial.h>




//无源蜂鸣器
#define BEEP_PIN 13
//蓝牙引脚
#define Software_TX A4
#define Software_RX A3

#define SS_PIN          10
#define RST_PIN         9
#define SERVO_PIN       5
#define STM_PIN         1

#define LCD_ADDR        0x27
#define LCD_COLS        16
#define LCD_ROWS        2
//温湿度传感器
#define DHT_PIN         4
#define DHT_TYPE        DHT11
//人物感测器
#define PIR_PIN         7
#define RED_LED         3
//红外接收器
#define IR_RECV_PIN     2
#define FAN_PIN         A2
//有源蜂鸣器
#define BUZZER_PIN      8
//声音传感器
#define VOICE_PIN       A0
#define GREEN_LED       6
//复位按钮
#define EMERGENCY_PIN   A1
//红外遥控器按钮设置
#define KEY_1   0x0C
#define KEY_2   0x18
#define KEY_3   0x5E
#define KEY_4   0x8
#define KEY_5   0x1C

SoftwareSerial BLE_JDY_16(Software_RX, Software_TX);//
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);
MFRC522 mfrc522(SS_PIN, RST_PIN);
Servo myservo;
DHT dht(DHT_PIN, DHT_TYPE);

byte authorizedUID1[4] = {0xEF, 0x4F, 0xBE, 0xDE};
byte authorizedUID2[4] = {0x53, 0x04, 0xD4, 0x1A};

#define OPEN_ANGLE      90
#define CLOSE_ANGLE     0
#define OPEN_DELAY      3000
#define DHT_READ_INTERVAL 2000

unsigned long lastDhtReadTime = 0;
float temp = 0.0, humi = 0.0;
bool buzzerState = false;

int ledBrightness = 128;
const int maxBright = 255;
const int minBright = 50;

bool soundTriggered = false;
unsigned long lightEndTime = 0;
const long LIGHT_ON_TIME = 3000;











void setup() {
  SPI.begin();
  mfrc522.PCD_Init();
  myservo.attach(SERVO_PIN);
  myservo.write(CLOSE_ANGLE);
  pinMode(STM_PIN, OUTPUT);
  lcd.init();
  lcd.backlight();
  dht.begin();

  pinMode(PIR_PIN, INPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(VOICE_PIN, INPUT);
  pinMode(GREEN_LED, OUTPUT);

  pinMode(BEEP_PIN, OUTPUT);
  Serial.begin(9600);
  BLE_JDY_16.begin(9600);

  pinMode(EMERGENCY_PIN, INPUT_PULLUP);

  digitalWrite(RED_LED, LOW);
  digitalWrite(FAN_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  analogWrite(GREEN_LED, 0);

  IrReceiver.begin(IR_RECV_PIN, ENABLE_LED_FEEDBACK);

  lcd.clear();
  lcd.print(" System Ready");
  delay(1000);
}

void emergencyStop() {
  digitalWrite(RED_LED, LOW);
  digitalWrite(FAN_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  analogWrite(GREEN_LED, 0);
  myservo.write(CLOSE_ANGLE);

  buzzerState = false;
  soundTriggered = false;

  lcd.clear();
  lcd.print(" EMERGENCY STOP");
  lcd.setCursor(0, 1);
  lcd.print(" All OFF 3s");
  
  delay(3000);
  
  lcd.clear();
}

void readDHTData() {
  unsigned long now = millis();
  if (now - lastDhtReadTime >= DHT_READ_INTERVAL) {
    lastDhtReadTime = now;
    humi = dht.readHumidity();
    temp = dht.readTemperature();
    if (isnan(humi) || isnan(temp)) {
      temp = 0; humi = 0;
    }
  }
}

void showLCD() {
  lcd.setCursor(0, 0);
  lcd.print("T:"); lcd.print(temp, 1); lcd.print("C ");
  lcd.print("H:"); lcd.print(humi, 1); lcd.print("%");

  lcd.setCursor(0, 1);
  lcd.print("Bright:"); lcd.print(ledBrightness);
  lcd.print("   ");
}

bool isUIDAuthorized(byte uid[4]) {
  bool match1 = true, match2 = true;
  for (byte i = 0; i < 4; i++) {
    if (uid[i] != authorizedUID1[i]) match1 = false;
    if (uid[i] != authorizedUID2[i]) match2 = false;
  }
  return match1 || match2;
}

void taskEmergencyStop();
void taskPIRSensor();
void taskVoiceSensor();
void taskIRRemote();
void taskRFID();
void taskBluetooth();

void taskEmergencyStop() {
  if (digitalRead(EMERGENCY_PIN) == LOW) {
    emergencyStop();
  }
}

void taskPIRSensor() {
  digitalWrite(RED_LED, digitalRead(PIR_PIN));
}

void taskVoiceSensor() {
  int voiceVal = analogRead(VOICE_PIN);
  if (voiceVal > 48 && !soundTriggered) {
    soundTriggered = true;
    lightEndTime = millis() + LIGHT_ON_TIME;
  }
  if (soundTriggered && millis() > lightEndTime) {
    soundTriggered = false;
  }
  analogWrite(GREEN_LED, soundTriggered ? ledBrightness : 0);
}

void taskIRRemote() {
  if (IrReceiver.decode()) {
    uint8_t cmd = IrReceiver.decodedIRData.command;
    uint8_t address = IrReceiver.decodedIRData.address;
    
    Serial.print("Address: 0x");
    Serial.print(address, HEX);
    Serial.print(" Command: 0x");
    Serial.println(cmd, HEX);
    
    switch (cmd) {
      case KEY_1:
        digitalWrite(FAN_PIN, HIGH);
        break;
      case KEY_2:
        digitalWrite(FAN_PIN, LOW);
        break;
      case KEY_3:
        buzzerState = !buzzerState;
        digitalWrite(BUZZER_PIN, buzzerState);
        break;
      case KEY_4:
        ledBrightness += 30;
        if (ledBrightness > maxBright) ledBrightness = maxBright;
        break;
      case KEY_5:
        ledBrightness -= 30;
        if (ledBrightness < minBright) ledBrightness = minBright;
        break;
    }
    IrReceiver.resume();
  }
}

void taskRFID() {
  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
    byte uid[4];
    for (byte i = 0; i < 4; i++) uid[i] = mfrc522.uid.uidByte[i];

    if (isUIDAuthorized(uid)) {
      lcd.clear();
      lcd.print(" Access Ok");
      lcd.setCursor(0, 1);
      lcd.print(" Door Opened");
      myservo.write(OPEN_ANGLE);
      delay(OPEN_DELAY);
      myservo.write(CLOSE_ANGLE);
    } else {
      lcd.clear();
      lcd.print(" Invalid Card");
      lcd.setCursor(0, 1);
      lcd.print(" Access Denied");
      delay(1500);
    }
    mfrc522.PICC_HaltA();
    mfrc522.PCD_StopCrypto1();
  }
}

void taskBluetooth() {
  while (BLE_JDY_16.available()) {
    char c = BLE_JDY_16.read();
    
    if (c == '1') {
      digitalWrite(STM_PIN, HIGH);
      delay(300);
      digitalWrite(STM_PIN, LOW);
    }
  }
}

void loop() {
  taskEmergencyStop();
  readDHTData();
  taskPIRSensor();
  taskVoiceSensor();
  taskIRRemote();
  taskRFID();
  taskBluetooth();
  showLCD();
}