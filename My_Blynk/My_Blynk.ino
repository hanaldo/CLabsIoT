#define BLYNK_PRINT Serial
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

char ssid[] = "UCInet Mobile Access";
char pass[] = "";
bool configured = false;
String authToken = "";
String boardName = "BlynkUCI";
String emailAddress = "";

const int EEPROM_CONFIG_FLAG_ADDRESS = 0;
const String BLYNK_AUTH_SPIFF_FILE = "/blynk.txt";
const int BLYNK_AUTH_TOKEN_SIZE = 32;

Ticker v9Ticker;
const int BUTTON_PIN = 0;
int buttonPinValue = 0;
int buttonPinValueOld = 0;
long ramTimer = 0;
int rainbowCounter = 0;

WidgetLED ledV1(V1);
WidgetLCD thLCD(V10);
WidgetTerminal terminal(V21);//SERIAL_VIRTUAL
WidgetTerminal chat(V30);//SERIAL_VIRTUAL

void setup() {
  Serial.begin(9600);
  rgb.begin();
  pinMode(2, INPUT_PULLUP);
  therSense.begin();
  if (!SPIFFS.begin()) {
    Serial.println(F("Failed to initialize SPIFFS"));
    rgb.setPixelColor(0, 0xFF0000);
    rgb.show();
    return;
  }
  rgb.setPixelColor(0, 0xFFFFFF);
  rgb.show();
  delay(5000);

  authToken = getBlynkAuth();
  if (authToken != "") {
    Serial.print(F("Your current auth token is: "));
    Serial.println(authToken);
    configured = true;
  }
  if (digitalRead(BUTTON_PIN) == 0) {
    Serial.println(F("Reset"));
    configured = false;
    resetEEPROM();
    authToken = "";
  }
  if (!configured) {
    rgb.setPixelColor(0, 0x0000FF);
    rgb.show();
    delay(3000);
    Serial.println(F("Please enter your auth token"));
  } else {
    Blynk.begin(authToken.c_str(), ssid, pass);
  }
}

BLYNK_CONNECTED() {
  Serial.println(F("Blynk Connected"));
  Blynk.syncAll();
}

void loop() {
  if (!configured) {
    if (Serial.available()) {
      authToken = Serial.readString();
      authToken.trim();
      Serial.println(authToken);
      writeBlynkConfig(authToken);
      configured = true;
      Blynk.begin(authToken.c_str(), ssid, pass);
    }
    return;
  }

  Blynk.run();
  if (Blynk.connected()) {
    checkButton();
    checkTwitter();
    checkSerialInput();
  }
  else {
    Serial.println(F("Blynk Disconnected"));
  }
  //  Serial.println("Loop");
  //  rgb.setPixelColor(0, 0x00FF00);
  //  rgb.show();
  //  delay(1000);
  //  rgb.setPixelColor(0, 0x0000FF);
  //  rgb.show();
  //  delay(1000);
  //  rgb.setPixelColor(0, 0xFF0000);
  //  rgb.show();
  //  delay(1000);
  //  rgb.setPixelColor(0, 0xFFFF00);
  //  rgb.show();
  //  delay(1000);
  //  if (ramTimer == 0) {
  //    ramTimer = millis();
  //  } else {
  //    if (millis() - ramTimer > 10000) {
  //      ramTimer = 0;
  //      Serial.println(ESP.getFreeHeap());
  //    }
  //  }
}

void checkButton() {
  buttonPinValue = digitalRead(BUTTON_PIN);
  if (buttonPinValue != buttonPinValueOld) {
    buttonPinValueOld = buttonPinValue;
    Serial.println(buttonPinValue);
    if (buttonPinValue) {
      ledV1.off();
    } else {
      ledV1.on();
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

float tempCOffset = -8.33; //-8.33;
float tempCUpperLimit = 27;

//TEMPERATURE_C_VIRTUAL
BLYNK_READ(V6) {
  float tempC = therSense.readTemperature();
  tempC += tempCOffset; // Add any offset
  Blynk.virtualWrite(V6, tempC);
}

//TEMPERATURE_F_VIRTUAL
BLYNK_READ(V5) {
  float tempC = therSense.readTemperature();
  tempC += tempCOffset;
  float tempF = tempC * 9.0 / 5.0 + 32.0;
  Blynk.virtualWrite(V5, tempF);
}

//HUMIDITY_VIRTUAL
BLYNK_READ(V7) {
  float humidity = therSense.readHumidity();
  Blynk.virtualWrite(V7, humidity);
}

//ADC_VOLTAGE_VIRTUAL
BLYNK_READ(V8) {
  float adcRaw = analogRead(A0);
  float voltage = ((float)adcRaw / 1024.0) * 3.2;
  Blynk.virtualWrite(V8, voltage);
}

BLYNK_WRITE(V9) {
  int pinValue = param.asInt();
  if (pinValue == 0) {
    v9Ticker.detach();
  } else {
    v9Ticker.attach_ms(100, flashRainbow);
  }
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
  if (param.asInt() > 0)
  {
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

unsigned int servoMax = 180; // Default maximum servo angle
const int SERVO_MINIMUM = 5;
int servoX = 0; // Servo angle x component
int servoY = 0; // Servo angle y component
Servo myServo; // Servo object
bool firstServoRun = true;

//SERVO_XY_VIRTUAL
BLYNK_WRITE(V14) {
  if (firstServoRun) {
    myServo.attach(15);
    myServo.write(15);
    firstServoRun = false;
  }
  // Read in the servo value:
  int servoXIn = param[0].asInt();
  int servoYIn = param[1].asInt();
  servoX = servoXIn - 128; // Center xIn around 0 (+/-128)
  servoY = servoYIn - 128; // Center xIn around 0 (+/-128)

  // Calculate the angle, given x and y components:
  float pos = atan2(servoY, servoX) * 180.0 / PI; // Convert to degrees
  // atan2 will give us an angle +/-180
  if (pos < 0) // Convert it to 0-360:
    pos = 360.0 + pos;

  int servoPos = map(pos, 0, 360, 5, servoMax);

  // Constrain the angle between min/max:
  myServo.write(servoPos); // And set the servo position

  Blynk.virtualWrite(V17, servoPos);//SERVO_ANGLE_VIRUTAL
}

//SERVO_MAX_VIRTUAL
BLYNK_WRITE(V16) {
  int sMax = param.asInt();
  servoMax = constrain(sMax, SERVO_MINIMUM + 1, 360);
  Serial.println("ServoMax: " + String(servoMax));
}

BLYNK_READ(V20) {
  int rawADC = analogRead(A0);
  float voltage = ((float) rawADC / 1024.0) * 3.2;
  voltage *= 2.0; // Assume dividing VIN by two with another divider

  Blynk.virtualWrite(V20, voltage);
}

BLYNK_READ(V18) {
  int lightADC = analogRead(A0);
  Blynk.virtualWrite(V18, lightADC);
}

#define NOTIFICATION_LIMIT 5000
unsigned long lastDoorSwitchNotification = 0;
uint8_t lastSwitchState = 255;

//DOOR_STATE_VIRTUAL
BLYNK_READ(V25) {
  uint8_t switchState = digitalRead(16);//DOOR_SWITCH_PIN
  if (switchState) {
    Blynk.virtualWrite(V25, "Close");
  } else {
    Blynk.virtualWrite(V25, "Open");
  }
  if (lastSwitchState != switchState) {
    if (switchState) {
      Serial.println(F("Door closed"));
      if (lastDoorSwitchNotification && (lastDoorSwitchNotification + NOTIFICATION_LIMIT > millis())) {
        int timeLeft = (lastDoorSwitchNotification + NOTIFICATION_LIMIT - millis()) / 1000;
        terminal.println("Door closed. Can't notify for " + String(timeLeft) + "s.");
        terminal.flush();
      } else {
        Blynk.notify("Door closed\r\nFrom: " + boardName + "\r\n[" + String(millis()) + "]");
        terminal.println("Door closed!");
        terminal.flush();
        lastDoorSwitchNotification = millis();
      }
    } else {
      Serial.println(F("Door opened"));
      if (lastDoorSwitchNotification && (lastDoorSwitchNotification + NOTIFICATION_LIMIT > millis())) {
        int timeLeft = (lastDoorSwitchNotification + NOTIFICATION_LIMIT - millis()) / 1000;
        terminal.println("Door open. Can't notify for " + String(timeLeft) + "s");
        terminal.flush();
      } else {
        Blynk.notify("Door open\r\nFrom: " + boardName + "\r\n[" + String(millis()) + "]");
        terminal.println("Door opened!");
        terminal.flush();
        lastDoorSwitchNotification = millis();
      }
    }
    lastSwitchState = switchState;
  }
}

#define EMAIL_UPDATE_RATE 5000
unsigned long lastEmailUpdate = 0;

//EMAIL_ENABLED_VIRTUAL
//BLYNK_WRITE(V27) {
//  int emailEnableIn = param.asInt();
//  if (emailEnableIn) {
//    if (emailAddress != "") {
//      if ((lastEmailUpdate == 0) || (lastEmailUpdate + EMAIL_UPDATE_RATE < millis())) {
//        emailUpdate(); // Send an email
//        lastEmailUpdate = millis(); // Update the email time
//      } else {
//        // Print how many seconds before the next print
//        int waitTime = (lastEmailUpdate + EMAIL_UPDATE_RATE) - millis();
//        waitTime /= 1000;
//        terminal.println("Please wait " + String(waitTime) + " seconds");
//        terminal.flush();
//      }
//    } else {
//      terminal.println("Type !email@address.com to set the email address");
//      terminal.flush();
//    }
//  }
//}

BLYNK_WRITE(V30) {
  String incoming = param.asStr();
  Serial.println(incoming);
}

void checkSerialInput() {
  if (Serial.available()) {
    String input = Serial.readString();
    Serial.println(input);
    chat.print(input);
    chat.flush();
  }
}

BLYNK_WRITE(V21) {
  String incoming = param.asStr();
  Serial.println(incoming);

  if (incoming.charAt(0) == '$') {
    String newName = incoming.substring(1, incoming.length());
    boardName = newName;
    terminal.println("Board name set to: " + boardName + ".");
    terminal.flush();
  } else {
    terminal.println(F("Unrecognized command!"));
    terminal.flush();
  }
}

unsigned long tweetRate = 0;
unsigned long lastTweet = 0;
unsigned int moistureThreshold = 900;

//TWITTER_THRESHOLD_VIRTUAL
BLYNK_WRITE(V23) {
  moistureThreshold = param.asInt();
  String msg = "Moisture Threshold is set to: " + String(moistureThreshold) + ".\r\n";
  terminal.println(msg);
  terminal.flush();
  Serial.println(moistureThreshold);
  lastTweet = millis();
}

//TWITTER_RATE_VIRTUAL
BLYNK_WRITE(V24) {
  int r = param.asInt();
  if (r < 10) {
    r = 0;
  }
  tweetRate = r * 1000;
  String msg = "Tweet Rate is set to: " + String(r) + " seconds.\r\n";
  terminal.println(msg);
  terminal.flush();
  Serial.println(tweetRate);
  lastTweet = millis();
}

void checkTwitter() {
  if (tweetRate > 0) {
    unsigned long now = millis();
    if (now - lastTweet > tweetRate) {
      lastTweet = now;
      unsigned int moisture = analogRead(A0);
      if (moisture < moistureThreshold) {
        String msg = boardName + " thirsty!\r\nSoil reading: " + String(moisture) + ".\r\n";
        msg += "[" + String(millis()) + "]";
        Blynk.tweet(msg);
        Serial.println(F("tweet"));
      }
    }
  }
}

//========================================================================================================================================

void writeBlynkConfig(String authToken) {
  File authFile = SPIFFS.open(BLYNK_AUTH_SPIFF_FILE, "w");
  if (authFile) {
    authFile.print(authToken);
    authFile.close();
  } else {
    Serial.println("Cannot write file.");
    return;
  }
  Serial.println("Auth saved.");
}

String getBlynkAuth() {
  String retAuth = "";
  if (SPIFFS.exists(BLYNK_AUTH_SPIFF_FILE)) {
    File authFile = SPIFFS.open(BLYNK_AUTH_SPIFF_FILE, "r");
    if (authFile) {
      size_t authFileSize = authFile.size();
      // Only return auth token if it's the right size (32 bytes)
      if (authFileSize == BLYNK_AUTH_TOKEN_SIZE) {
        while (authFile.available()) {
          retAuth += (char)authFile.read();
        }
      } else {
        Serial.println(F("Wrong size."));
      }
      authFile.close();
    }
  }
  else {
    Serial.println(F("File does not exist."));
  }
  return retAuth;
}

void resetEEPROM() {
  EEPROM.write(EEPROM_CONFIG_FLAG_ADDRESS, 0);
  EEPROM.commit();
  SPIFFS.remove(BLYNK_AUTH_SPIFF_FILE);
}
