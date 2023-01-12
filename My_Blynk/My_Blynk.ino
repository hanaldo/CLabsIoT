/*
  My_Blynk.ino
  Improved firmware for Sparkfun Blynk board.
  Compile with ESP8266 core v3.1.0, Blynk library v1.1.0, Adafruit NeoPixel 1.10.7, SparkFun HTU21D 1.1.3 (Fully Tested).

  Author: Shenshen Han @ Creativity Labs

  License:
  This is released under the MIT license (http://opensource.org/licenses/MIT).
  Please see the included LICENSE.txt for more information.
*/
#define BLYNK_PRINT Serial
//#define DEBUG
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <Adafruit_NeoPixel.h>
#include "SparkFunHTU21D.h"
#include "Servo.h"
#include <Ticker.h>
#include "FS.h"
#include <EEPROM.h>

HTU21D therSense;
Adafruit_NeoPixel rgb = Adafruit_NeoPixel(1, 4, NEO_GRB + NEO_KHZ800);

bool configured = false;
String settings[] = { "", "", "", "", "" };  //token, ssid, pass, url, port

const int EEPROM_CONFIG_FLAG_ADDRESS = 0;
const String BLYNK_AUTH_SPIFF_FILE = "/blynk.txt";
const String SSID_SPIFF_FILE = "/ssid.txt";
const String PASS_SPIFF_FILE = "/pass.txt";
const String URL_SPIFF_FILE = "/url.txt";
const String PORT_SPIFF_FILE = "/port.txt";
const int BLYNK_AUTH_TOKEN_SIZE = 32;

Ticker v9Ticker;
const int BUTTON_PIN = 0;
int buttonPinValue = 0;
int buttonPinValueOld = 0;
int rainbowCounter = 0;
BlynkTimer timerPushVirtual;

WidgetLCD thLCD(V10);
WidgetTerminal chat(V30);  //SERIAL_VIRTUAL

void setup() {
  Serial.begin(9600);
  rgb.begin();
  pinMode(16, INPUT_PULLDOWN_16);
  pinMode(BUTTON_PIN, INPUT);
  therSense.begin();
  if (!SPIFFS.begin()) {
    Serial.println(F("Failed to initialize SPIFFS"));
    rgb.setPixelColor(0, 0xFF0000);
    rgb.show();
    return;
  }
  showRGB(255, 255, 255, 30);
  delay(5000);

  Serial.println(WiFi.macAddress());
  Serial.println(F("v2.0"));
  settings[0] = getFile(BLYNK_AUTH_SPIFF_FILE);
  settings[1] = getFile(SSID_SPIFF_FILE);
  settings[2] = getFile(PASS_SPIFF_FILE);
  settings[3] = getFile(URL_SPIFF_FILE);
  settings[4] = getFile(PORT_SPIFF_FILE);
  if (settings[0] != "") {
    Serial.print(F("Your current auth token is: "));
    Serial.println(settings[0]);
    Serial.print(F("Your are connecting: "));
    Serial.println(settings[1]);
    configured = true;
  }
  if (digitalRead(BUTTON_PIN) == 0) {
    Serial.println(F("Reset"));
    configured = false;
    resetConfig();
    settings[0] = "";
  }
  if (!configured) {
    showRGB(0, 0, 255, 30);
    delay(3000);
    Serial.println(F("Please enter your auth token and wifi info:"));
  } else {
    Blynk.begin(settings[0].c_str(), settings[1].c_str(), settings[2].c_str(), settings[3].c_str(), atoi(settings[4].c_str()));
    showRGB(0, 255, 0, 30);
    timerPushVirtual.setInterval(1000L, pushVirtualPins);
  }
}

BLYNK_CONNECTED() {
  Serial.println(F("Blynk Connected"));
  Blynk.syncAll();
  SPIFFS.end();
}

void loop() {
  if (!configured) {
    if (Serial.available()) {
      String input = Serial.readString();
      input = input + ",";
      parseString(input);
      writeBlynkSettings(settings[0], BLYNK_AUTH_SPIFF_FILE);
      writeBlynkSettings(settings[1], SSID_SPIFF_FILE);
      writeBlynkSettings(settings[2], PASS_SPIFF_FILE);
      writeBlynkSettings(settings[3], URL_SPIFF_FILE);
      writeBlynkSettings(settings[4], PORT_SPIFF_FILE);
      Serial.println(settings[0]);
      Serial.println(settings[1]);
      Serial.println(settings[2]);
      Serial.println(settings[3]);
      Serial.println(settings[4]);
      Blynk.begin(settings[0].c_str(), settings[1].c_str(), settings[2].c_str(), settings[3].c_str(), atoi(settings[4].c_str()));
      showRGB(0, 255, 0, 30);
      configured = true;
    }
    return;
  }

  Blynk.run();
  if (Blynk.connected()) {
    checkButton();
    checkSerialInput();
    timerPushVirtual.run();
  } else {
    Serial.println(F("Blynk Disconnected"));
    showRGB(255, 0, 0, 255);
  }
}

void checkButton() {
  buttonPinValue = digitalRead(BUTTON_PIN);
  if (buttonPinValue != buttonPinValueOld) {
    buttonPinValueOld = buttonPinValue;
    Serial.print(F("Button: "));
    Serial.println(buttonPinValue);
    if (buttonPinValue) {
      Blynk.virtualWrite(V1, 0);
    } else {
      Blynk.virtualWrite(V1, 255);
    }
  }
}

int red;
int green;
int blue;
int rgbMaxBrightness = 64;

void updateRGB() {
  rgb.setPixelColor(0, rgb.Color(map(red, 0, 255, 0, rgbMaxBrightness),
                                 map(green, 0, 255, 0, rgbMaxBrightness),
                                 map(blue, 0, 255, 0, rgbMaxBrightness)));
  rgb.show();
}

void showRGB(int r, int g, int b, int br) {
  rgb.setPixelColor(0, rgb.Color(map(r, 0, 255, 0, br),
                                 map(g, 0, 255, 0, br),
                                 map(b, 0, 255, 0, br)));
  rgb.show();
}

BLYNK_WRITE(V2) {
  int pinValue = param.asInt();
  red = pinValue;
  updateRGB();
}

BLYNK_WRITE(V3) {
  int pinValue = param.asInt();
  green = pinValue;
  updateRGB();
}

BLYNK_WRITE(V4) {
  int pinValue = param.asInt();
  blue = pinValue;
  updateRGB();
}

BLYNK_WRITE(V15) {
  int pinValue = param.asInt();
  rgbMaxBrightness = pinValue;
  updateRGB();
}

float tempCOffset = -8.33;  //-8.33;
float tempCUpperLimit = 27;

//TEMPERATURE_C_VIRTUAL
void pushV6() {
  float tempC = therSense.readTemperature();
  tempC += tempCOffset;  // Add any offset
  Blynk.virtualWrite(V6, tempC);
#ifdef DEBUG
  Serial.println(F("TempC:V6"));
#endif
}

//TEMPERATURE_F_VIRTUAL
void pushV5() {
  float tempC = therSense.readTemperature();
  tempC += tempCOffset;
  float tempF = tempC * 9.0 / 5.0 + 32.0;
  Blynk.virtualWrite(V5, tempF);
#ifdef DEBUG
  Serial.println(F("TempF:V5"));
#endif
}

//HUMIDITY_VIRTUAL
void pushV7() {
  float humidity = therSense.readHumidity();
  Blynk.virtualWrite(V7, humidity);
#ifdef DEBUG
  Serial.println(F("Humi:V7"));
#endif
}

//ADC_VOLTAGE_VIRTUAL
void pushV8(int adcRaw) {
  float voltage = ((float)adcRaw / 1024.0) * 3.2;
  Blynk.virtualWrite(V8, voltage);
#ifdef DEBUG
  Serial.println(F("Vol:V8"));
#endif
}

//ADC_BATT_VIRTUAL
// BLYNK_READ(V20) {
//   int rawADC = analogRead(A0);
//   float voltage = ((float)rawADC / 1024.0) * 3.2;
//   voltage *= 2.0;  // Assume dividing VIN by two with another divider
//   Blynk.virtualWrite(V20, voltage);
// }

BLYNK_WRITE(V9) {
  int pinValue = param.asInt();
  if (pinValue == 0) {
    v9Ticker.detach();
  } else {
    v9Ticker.attach_ms(100, flashRainbow);
  }
  Serial.print(F("V9: "));
  Serial.println(pinValue);
}

void flashRainbow() {
  switch (rainbowCounter) {
    case 0:
      red = 255;
      green = 0;
      blue = 0;
      updateRGB();
      break;
    case 1:
      red = 0;
      green = 255;
      blue = 0;
      updateRGB();
      break;
    case 2:
      red = 0;
      green = 0;
      blue = 255;
      updateRGB();
      break;
    case 3:
      red = 255;
      green = 255;
      blue = 0;
      updateRGB();
      break;
    case 4:
      red = 255;
      green = 0;
      blue = 255;
      updateRGB();
      break;
    case 5:
      red = 0;
      green = 255;
      blue = 255;
      updateRGB();
      break;
    default:
      break;
  }
  rainbowCounter += 1;
  if (rainbowCounter >= 6) {
    rainbowCounter = 0;
  }
}

BLYNK_WRITE(V11) {
  if (param.asInt() > 0) {
    float humidity = therSense.readHumidity();
    float tempC = therSense.readTemperature();
    tempC += tempCOffset;
    float tempF = tempC * 9.0 / 5.0 + 32.0;
    String tempLine = String(tempF, 2) + "F / " + String(tempC, 2) + "C";
    String humidityLine = "Humidity: " + String(humidity, 1) + "%";
    thLCD.clear();
    thLCD.print(0, 0, tempLine.c_str());
    thLCD.print(0, 1, humidityLine.c_str());
  }
}

BLYNK_WRITE(V12) {
  if (param.asInt() > 0) {
    String firstLine = "V1: ";
    if (digitalRead(BUTTON_PIN)) {
      firstLine += "HIGH";
    } else {
      firstLine += "LOW";
    }
    String secondLine = "ADC0: " + String(analogRead(A0));
    thLCD.clear();
    thLCD.print(0, 0, firstLine.c_str());
    thLCD.print(0, 1, secondLine.c_str());
  }
}

BLYNK_WRITE(V13) {
  if (param.asInt() > 0) {
    String topLine = "RunTime";
    String botLine = "";
    float seconds, minutes, hours;
    seconds = (float)millis() / 1000;
    minutes = seconds / 60;
    hours = minutes / 60;
    seconds = (int)seconds % 60;
    minutes = (int)minutes % 60;
    if (hours < 10) {
      botLine += "0";
    }
    botLine += String((int)hours) + ":";
    if (minutes < 10) {
      botLine += "0";
    }
    botLine += String((int)minutes) + ":";
    if (seconds < 10) {
      botLine += "0";
    }
    botLine += String((int)seconds);
    thLCD.clear();
    thLCD.print(0, 0, topLine.c_str());
    thLCD.print(0, 1, botLine.c_str());
  }
}

unsigned int servoMax = 180;  // Default maximum servo angle
const int SERVO_MINIMUM = 5;
int servoX = 0;  // Servo angle x component
int servoY = 0;  // Servo angle y component
Servo myServo;   // Servo object
bool firstServoRun = true;

//SERVO_XY_VIRTUAL
BLYNK_WRITE(V14) {
  if (firstServoRun) {
    myServo.attach(15);
    myServo.write(15);
    firstServoRun = false;
  }
  int servoXIn = param[0].asInt();
  int servoYIn = param[1].asInt();
  servoX = servoXIn - 128;  // Center xIn around 0 (+/-128)
  servoY = servoYIn - 128;  // Center xIn around 0 (+/-128)
  // Calculate the angle, given x and y components:
  float pos = atan2(servoY, servoX) * 180.0 / PI;  // Convert to degrees
  // atan2 will give us an angle +/-180
  if (pos < 0) {
    pos = 360.0 + pos;
  }
  int servoPos = map(pos, 0, 360, 5, servoMax);
  myServo.write(servoPos);
  Blynk.virtualWrite(V17, servoPos);  //SERVO_ANGLE_VIRUTAL
}

//SERVO_MAX_VIRTUAL
BLYNK_WRITE(V16) {
  int sMax = param.asInt();
  servoMax = constrain(sMax, SERVO_MINIMUM + 1, 360);
  Serial.print(F("ServoMax: "));
  Serial.println(String(servoMax));
}

BLYNK_WRITE(V30) {
  String incoming = param.asStr();
  Serial.print(F("V30: "));
  Serial.println(incoming);
}

BLYNK_WRITE(V39) {
  Serial.print(F("V39 Webhook"));
  Blynk.virtualWrite(V40, param.asStr());  //Relay the message to trigger webhook
}

int switchState = 0;
void pushV20() {
  int state = digitalRead(16);  //DOOR_SWITCH_PIN
  if (state != switchState) {
    Blynk.virtualWrite(V20, state);
    switchState = state;
  }
}

//Push ADC to a virtual pin
void pushVirtualPins() {
#ifdef DEBUG
  Serial.println(F("Push Virtual"));
#endif
  int adcRaw = analogRead(A0);  //A0 - ADC
  Blynk.virtualWrite(V0, adcRaw);
  pushV8(adcRaw);
  pushV5();
  pushV6();
  pushV7();
  pushV20();
}

//=================================================================================================================================

void checkSerialInput() {
  if (Serial.available()) {
    String input = Serial.readString();
    Serial.println(input);
    chat.print(input);
    chat.flush();
    //Serial.print(F("FreeHeap:"));
    //Serial.println(ESP.getFreeHeap());
  }
}

void writeBlynkSettings(String setting, String fileName) {
  File file = SPIFFS.open(fileName, "w");
  if (file) {
    file.print(setting);
    file.close();
  } else {
    Serial.println(F("Cannot write file."));
    return;
  }
  Serial.println(F("Settings saved."));
}

String getFile(String fileName) {
  String content = "";
  if (SPIFFS.exists(fileName)) {
    File file = SPIFFS.open(fileName, "r");
    if (file) {
      while (file.available()) {
        content += (char)file.read();
      }
      file.close();
    }
  } else {
    Serial.println(F("No setting file."));
  }
  return content;
}

void parseString(String message) {
  int index = 0;
  int commaPosition = 0;
  do {
    commaPosition = message.indexOf(",");
    if (commaPosition != -1) {
      String item = message.substring(0, commaPosition);
      item.trim();
      settings[index] = item;
      message = message.substring(commaPosition + 1, message.length());
      index += 1;
    }
  } while (commaPosition >= 0 && index < 5);
}

void resetConfig() {
  EEPROM.write(EEPROM_CONFIG_FLAG_ADDRESS, 0);
  EEPROM.commit();
  SPIFFS.remove(BLYNK_AUTH_SPIFF_FILE);
  SPIFFS.remove(SSID_SPIFF_FILE);
  SPIFFS.remove(PASS_SPIFF_FILE);
}
