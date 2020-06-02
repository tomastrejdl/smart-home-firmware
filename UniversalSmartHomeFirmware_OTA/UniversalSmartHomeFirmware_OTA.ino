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

// For web interface
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>         // https://github.com/tomastrejdl/WiFiManager

#include "EspMQTTClient.h"
#include "ArduinoJson.h"
#include "DHT.h"


// DON'T FORGET TO CHANGE BOARD IN ARDUINO WHEN UPLOADING !!!
// Board types
//#define WEMOSD1MINI         // uncomment when uploading to Wemos D1 Mini
#define ESP01_TEMP               // uncomment when uploading to ESP-01 with a Temperature sensor attached to GPIO2

#ifdef WEMOSD1MINI
  // Pin definition for Wemos D1 mini
  #define TEMPERATURE_PIN D3
  #define DOOR_PIN D4
#endif

#ifdef ESP01_TEMP
  // Pin definition for ESP-01
  #define TEMPERATURE_PIN 2
  #define DOOR_PIN 0
#endif





// ---------------------------------------------------------------------------------------------------------------------------------------------------
// GLOBAL VARIABLES

// 
StaticJsonDocument<64> PINS;        // Pin mapping from strings ("D1", "D2, ...) to GRPIO numbers (0, 1, 2, ...) on Wemos D1 mini
StaticJsonDocument<64> STATE;       // State of pins, LOW/HIGH
bool DOOR_STATE = LOW;              // State of DOOR_PIN, LOW/HIGH

DHT myDHT(TEMPERATURE_PIN, DHT11);  // Definition for DHT11 temperature and humidity sensor
unsigned int tempInterval = 0;      // Interval in milliseconds to check temperature
String tempAttId;                   // database ID of the teperature sensor Attachment

unsigned int doorInterval = 0;      // Interval in milliseconds to check door state
bool invertDoor = false;            // if true invert the door state, e.g., door is open if DOOR_PIN is LOW => door is open if DOOR_PIN is HIGH
String doorAttId;                   // database ID of the door attachment

unsigned int prevTempMillis;        // time of previous temperature reading
unsigned int prevDoorMillis;        // time of previous door state reading

String mac = WiFi.macAddress();
String mqttClientName = "ESP-" + mac;

// MQTT client definition, args: broker_ip, port, MQTT_Client_Name
EspMQTTClient client(
  "smarthome.local",
  1883,
  mqttClientName.c_str()
);






// ---------------------------------------------------------------------------------------------------------------------------------------------------
// FUNCTIONS

// Turn ON light on selected pin, with 1 second fade in
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

// Turn OFF light on selected pin, with 1 second fade out
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

// Turn ON socket on selected pin
void turnOnSocket(String pin) {
  STATE[pin] = HIGH;
  digitalWrite(PINS[pin], STATE[pin]);
}

// Turn OFF socket on selected pin
void turnOffSocket(String pin) {
  STATE[pin] = HIGH;
  digitalWrite(PINS[pin], STATE[pin]);
}

// Blink led on pin pinNum 5 times, duration 1 second
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

// Subscribe to MQTT topic for Lights, and change light ON/OFF state based on received messages
void handleLight(String deviceId, String pin) {
  pinMode(PINS[pin], OUTPUT);
  turnOffLight(pin);
  client.subscribe("lights/" + deviceId + "/" + pin, [pin](const String & topic, const String & payload) {
    if (payload == "on") turnOnLight(pin);
    if (payload == "off") turnOffLight(pin);
  });
}

// Subscribe to MQTT topic for Sockets, and change socket ON/OFF state based on received messages
void handleSocket(String deviceId, String pin) {
  pinMode(PINS[pin], OUTPUT);
  turnOffSocket(pin);
  client.subscribe("sockets/" + deviceId + "/" + pin, [pin](const String & topic, const String & payload) {
    if (payload == "on") turnOnSocket(pin);
    if (payload == "off") turnOffSocket(pin);
  });
}


// Register Temperature Sensor Attachment ID and start reading temperature from TEMPERATURE_PIN in defined interval tempI
void handleTemperatureSensor(String attachmentId, int tempI) {
  tempAttId = attachmentId;
  tempInterval = tempI;
  pinMode(TEMPERATURE_PIN, INPUT_PULLUP);
}

// Register Door Sensor Attachment ID and start reading door state from DOOR_PIN in defined interval doorI
void handleDoorSensor(String attachmentId, int doorI, bool invert) {
  doorAttId = attachmentId;
  doorInterval = doorI;
  invertDoor = invert;
  pinMode(DOOR_PIN, INPUT_PULLUP);
}

// This functions is called by the EspMQTTClient library once the connection the the broker is established
void onConnectionEstablished()
{
  // Subscribe to global device discovery MQTT topic, send information about this device to topic "global/discoveryResponse" when a message is received to "global/discovery" topic 
  client.subscribe("global/discovery", [](const String & topic, const String & payload) {
    client.publish("global/discoveryResponse", "{\"deviceType\": \"ESP8266\",\"macAddress\": \"" + WiFi.macAddress() + "\"}");
  });

  // Subsribe to device specific MQTT topic, this topic receives configuration messages intended to configure this device's pins and functions
  client.subscribe("device/" + WiFi.macAddress(), [](const String & topic, const String & payload) {
    // Parse payload
    StaticJsonDocument<512> doc;
    deserializeJson(doc, payload);

    // Unsubscribe from old topics

    // Subscribe to new topics
    // Parse incomming JSON
    String deviceId = doc["deviceId"];
    String attachmentId = doc["attachmentId"];
    String attachmentType = doc["attachmentType"];
    String pinStr = doc["pin"];
    int pinNum = PINS[pinStr];
    int sampleInterval = doc["sampleInterval"];
    bool invert = doc["invert"];

    // Check if pin exists
    if (pinNum == NULL & pinStr != "D3") {    // Because pin "D3" is GPIO 0, e.g., 0 == NULL evaluates to true, and the Json library return 0 for non defined pins as well ...
      client.publish("global/error", "{\"error\": \"Error: Pin " + pinStr + " is not available on device: " + WiFi.macAddress() + "\"}");
      return;
    }

    // handle different attachment types
    if (attachmentType == "light") handleLight(deviceId, pinStr);
    if (attachmentType == "socket") handleSocket(deviceId, pinStr);
    if (attachmentType == "temperature-sensor") handleTemperatureSensor(attachmentId, sampleInterval);
    if (attachmentType == "door-sensor") handleDoorSensor(attachmentId, sampleInterval, invert);
  });

  // Send a status message to "global/deviceState" MQTT topic on startup and every time a message is received to "global/reportOnlineState" topic
  String isOnlineMessage = "{\"macAddress\": \"" + WiFi.macAddress() + "\",\"ipAddress\":\"" + WiFi.localIP().toString() + "\",\"isOnline\":true}";
  client.subscribe("global/reportOnlineState", [isOnlineMessage](const String & topic, const String & payload) {
    client.publish("global/deviceState", isOnlineMessage, true);
  });
  client.publish("global/deviceState", isOnlineMessage, true);
}





// ---------------------------------------------------------------------------------------------------------------------------------------------------
// OTA

// Setup for Over the Air update (OTA) taken from official ESP8266 Arduino examples
void setupOTA() {
//  WiFi.mode(WIFI_STA);
//  WiFi.begin(STASSID, STAPSK);
//  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
//    Serial.println("Connection Failed! Rebooting...");
//    delay(5000);
//    ESP.restart();
//  }

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
//  Serial.print("IP address: ");
//  Serial.println(WiFi.localIP());
}





// ---------------------------------------------------------------------------------------------------------------------------------------------------
//SETUP

// Set all pins to their default states, e.g. Lights and Sockets OFF (LOW)
void pinsToDefault() {
  #ifdef WEMOSD1MINI
    pinMode(PINS["D1"], OUTPUT);
    pinMode(PINS["D2"], OUTPUT);
    pinMode(PINS["D3"], OUTPUT);
    pinMode(PINS["D4"], OUTPUT);
    digitalWrite(PINS["D1"], LOW);
    digitalWrite(PINS["D2"], LOW);
    digitalWrite(PINS["D3"], LOW);
    digitalWrite(PINS["D4"], LOW);
    STATE["D1"] = STATE["D2"] = STATE["D3"] = STATE["D4"] = LOW;
  #endif

  #ifdef ESP01_TEMP
    pinMode(PINS["D3"], OUTPUT);
    digitalWrite(PINS["D3"], LOW);
    STATE["D3"] = LOW;
  #endif
}

// This code runs once on statup
void setup() {
  #ifdef WEMOSD1MINI
    PINS["D1"] = D1;
    PINS["D2"] = D2;
    PINS["D3"] = D3;
    PINS["D4"] = D4;
  #endif

  #ifdef ESP01_TEMP
      PINS["D3"] = 2;
  #endif
  

  myDHT.begin();

  Serial.begin(115200);
  Serial.println("Booting");

  // Wifi manager, for web interface to enter SSID and password to WiFi
  WiFiManager wifiManager;
  int result = wifiManager.autoConnect();
  Serial.println("WiFiManager result: " + result);

  client.enableDebuggingMessages();

  // MQTT Last will, used for detecting that a device has gone offline
  char message[100];
  StaticJsonDocument<128> doc;
  doc["macAddress"] = mac;
  doc["isOnline"] = false;
  serializeJson(doc, message);
  client.enableLastWillMessage("global/deviceState", message, true);

  setupOTA();

  prevTempMillis = prevDoorMillis = millis();

  pinsToDefault();

  blinkLed(LED_BUILTIN);

  Serial.println("All systems GO");
}





// ---------------------------------------------------------------------------------------------------------------------------------------------------
// LOOP

// Check conditions for reading temperature (the Attachment ID is defined, and interval is greater than 0), read data (temperature + humidity) and send over MQTT to topic "global/temprerature"
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

// Check conditions for reading temperature (the Attachment ID is defined, and interval is greater than 0), read door state (open/closed) and send over MQTT to topic "global/door"
void loopDoorSensor() {
  unsigned int currentMillis = millis();

  if (currentMillis - prevDoorMillis > doorInterval) {
    bool prevDoorState = DOOR_STATE;
    DOOR_STATE = (bool)digitalRead(DOOR_PIN);
    if (DOOR_STATE != prevDoorState) {
      bool realDoorState = DOOR_STATE;
      if (invertDoor) realDoorState = !realDoorState;

      String isOpen;
      if (realDoorState == HIGH) isOpen = "false";
      else isOpen = "true";

      client.publish("global/door", "{\"attachmentId\":\"" + doorAttId + "\",\"isOpen\":" + isOpen + "}");
    }
    prevDoorMillis = currentMillis;
  }
}


// Classic Arduino loop, runs repetedly, like while(true) {...}
void loop(){
  ArduinoOTA.handle();
  client.loop();

  if (tempAttId != NULL && tempInterval > 0) loopTemperatureSensor();
  if (doorAttId != NULL && doorInterval > 0) loopDoorSensor();
}
