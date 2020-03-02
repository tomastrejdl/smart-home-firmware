/*
  Universal Smart Home Firmware
  Author: Tomáš Trejdl <tom.trejdl@seznam.cz>
  Features:
    - WiFi
    - OTA update
    - MQTT
    - sending JSON over MQTT
    - Device Types: Light, Wall Socket, Temperature Sensor, Door Sensor
*/


#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "EspMQTTClient.h"
#include <ArduinoJson.h>
#include "DHT.h"

#ifndef STASSID
#define STASSID "Yolo"
#define STAPSK  "44141158"
#endif

// Pin definition for Wemos D1 mini
#define TEMPERATURE_PIN D3
#define DOOR_PIN D4





// ---------------------------------------------------------------------------------------------------------------------------------------------------
// GLOBAL VARIABLES

StaticJsonDocument<64> PINS;
StaticJsonDocument<64> STATE;
bool DOOR_STATE = LOW;

DHT myDHT(TEMPERATURE_PIN, DHT11);
unsigned int tempInterval = 0;
String tempAttId;

unsigned int doorInterval = 0;
bool invertDoor = false;
String doorAttId;

unsigned int prevTempMillis;
unsigned int prevDoorMillis;

String mac = WiFi.macAddress();
String mqttClientName = "ESP-" + mac;

EspMQTTClient client(
  "192.168.1.100",
  1883,
  mqttClientName.c_str()
);






// ---------------------------------------------------------------------------------------------------------------------------------------------------
// FUNCTIONS

void turnOnLight(String pin) {
  if (STATE[pin] == LOW) {
    for (int i = 0; i < 1024; i++) {
      analogWrite(PINS[pin], i);
      delay(1);
    }
    STATE[pin] = HIGH;
    digitalWrite(PINS[pin], STATE[pin]);
  }
}

void turnOffLight(String pin) {
  if (STATE[pin] == HIGH) {
    for (int i = 1023; i >= 0; i--) {
      analogWrite(PINS[pin], i);
      delay(1);
    }
    STATE[pin] = LOW;
    digitalWrite(PINS[pin], STATE[pin]);
  }
}

void turnOnSocket(String pin) {
  STATE[pin] = HIGH;
  digitalWrite(PINS[pin], STATE[pin]);
}

void turnOffSocket(String pin) {
  STATE[pin] = HIGH;
  digitalWrite(PINS[pin], STATE[pin]);
}

void blinkLed(uint8_t pinNum) {
  pinMode(pinNum, OUTPUT);
  for (int i = 0; i <= 10; i++) {
    digitalWrite(pinNum, i % 2);
    delay(100);
  }
  digitalWrite(pinNum, LOW);
}



// ---------------------------------------------------------------------------------------------------------------------------------------------------
// MQTT

void handleLight(String deviceId, String pin) {
  pinMode(PINS[pin], OUTPUT);
  turnOffLight(pin);
  client.subscribe("lights/" + deviceId + "/" + pin, [pin](const String & topic, const String & payload) {
    if (payload == "on") turnOnLight(pin);
    if (payload == "off") turnOffLight(pin);
  });
}

void handleSocket(String deviceId, String pin) {
  pinMode(PINS[pin], OUTPUT);
  turnOffSocket(pin);
  client.subscribe("sockets/" + deviceId + "/" + pin, [pin](const String & topic, const String & payload) {
    if (payload == "on") turnOnSocket(pin);
    if (payload == "off") turnOffSocket(pin);
  });
}

void handleTemperatureSensor(String attachmentId, int tempI) {
  tempAttId = attachmentId;
  tempInterval = tempI;
  pinMode(TEMPERATURE_PIN, INPUT_PULLUP);
}

void handleDoorSensor(String attachmentId, int doorI, bool invert) {
  doorAttId = attachmentId;
  doorInterval = doorI;
  invertDoor = invert;
  pinMode(DOOR_PIN, INPUT_PULLUP);
}

void onConnectionEstablished()
{
  client.subscribe("global/discovery", [](const String & topic, const String & payload) {
    client.publish("global/discoveryResponse", "{\"deviceType\": \"ESP8266\",\"macAddress\": \"" + WiFi.macAddress() + "\"}");
  });

  client.subscribe("device/" + WiFi.macAddress(), [](const String & topic, const String & payload) {
    // Parse payload
    StaticJsonDocument<512> doc;
    deserializeJson(doc, payload);

    // Potencially Unsubscribe from old topics here

    // Subscribe to new topics
    String deviceId = doc["deviceId"];
    String attachmentId = doc["attachmentId"];
    String attachmentType = doc["attachmentType"];
    String pinStr = doc["pin"];
    int pinNum = PINS[pinStr];
    int sampleInterval = doc["sampleInterval"];
    bool invert = doc["invert"];

    // Check pin
    if (pinNum == NULL & pinStr != "D3") {    // Because pin "D3" is GPIO 0, e.g., 0 == NULL evaluates to true, and the Json library return 0 for non defined pins as well ...
      client.publish("global/error", "{\"error\": \"Error: Pin " + pinStr + " is not available on device: " + WiFi.macAddress() + "\"}");
      return;
    }

    if (attachmentType == "light") handleLight(deviceId, pinStr);
    if (attachmentType == "socket") handleSocket(deviceId, pinStr);
    if (attachmentType == "temperature-sensor") handleTemperatureSensor(attachmentId, sampleInterval);
    if (attachmentType == "door-sensor") handleDoorSensor(attachmentId, sampleInterval, invert);
  });

  String isOnlineMessage = "{\"macAddress\": \"" + WiFi.macAddress() + "\",\"ipAddress\":\"" + WiFi.localIP().toString() + "\",\"isOnline\":true}";
  client.subscribe("global/reportOnlineState", [isOnlineMessage](const String & topic, const String & payload) {
    client.publish("global/deviceState", isOnlineMessage, true);
  });
  client.publish("global/deviceState", isOnlineMessage, true);
}





// ---------------------------------------------------------------------------------------------------------------------------------------------------
// OTA

void setupOTA() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(STASSID, STAPSK);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("myesp8266");

  // No authentication by default
  //  ArduinoOTA.setPassword("8266");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  Serial.println("OTA Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}





// ---------------------------------------------------------------------------------------------------------------------------------------------------
//SETUP

void setup() {
  PINS["D1"] = D1;
  PINS["D2"] = D2;
  PINS["D3"] = D3;
  PINS["D4"] = D4;

  STATE["D1"] = STATE["D2"] = STATE["D3"] = STATE["D4"] = LOW;

  myDHT.begin();

  Serial.begin(115200);
  Serial.println("Booting");

  client.enableDebuggingMessages();

  // MQTT Last will
  char message[100];
  StaticJsonDocument<128> doc;
  doc["macAddress"] = mac;
  doc["isOnline"] = false;
  serializeJson(doc, message);
  client.enableLastWillMessage("global/deviceState", message, true);

  setupOTA();

  prevTempMillis = prevDoorMillis = millis();

  blinkLed(LED_BUILTIN);
  Serial.println("All systems GO");
}





// ---------------------------------------------------------------------------------------------------------------------------------------------------
// LOOP

void loopTemperatureSensor() {
  unsigned int currentMillis = millis();

  if (currentMillis - prevTempMillis > tempInterval) {
    float temp = myDHT.readTemperature();
    float hum = myDHT.readHumidity();

    if (isnan(temp) || isnan(hum)) {
      String message = "{\"error\": \"Error reading temperature\", \"values\":[" + String(temp) + "," + String(hum) + "]}";
      Serial.println(message);
      client.publish("global/error", message);
    } else {
      StaticJsonDocument<512> doc;
      doc["attachmentId"] = tempAttId;
      doc["temperature"] = temp;
      doc["humidity"] = hum;

      String message;
      serializeJson(doc, message);
      client.publish("global/temperature", message);
    }
    prevTempMillis = currentMillis;
  }
}

void loopDoorSensor() {
  unsigned int currentMillis = millis();

  if (currentMillis - prevDoorMillis > doorInterval) {
    bool prevDoorState = DOOR_STATE;
    DOOR_STATE = (bool)digitalRead(DOOR_PIN);
    if (DOOR_STATE != prevDoorState) {
      bool realDoorState = DOOR_STATE;
      if(invertDoor) realDoorState = !realDoorState;

      String isOpen;
      if(realDoorState == HIGH) isOpen = "false";
      else isOpen = "true";
      
      client.publish("global/door", "{\"attachmentId\":\"" + doorAttId + "\",\"isOpen\":" + isOpen + "}");
    }
    prevDoorMillis = currentMillis;
  }
}

void loop() {
  ArduinoOTA.handle();
  client.loop();

  if (tempAttId != NULL && tempInterval > 0) loopTemperatureSensor();
  if (doorAttId != NULL && doorInterval > 0) loopDoorSensor();
}
