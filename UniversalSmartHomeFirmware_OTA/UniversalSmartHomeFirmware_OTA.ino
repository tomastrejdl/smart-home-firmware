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

#define MAX_SUBSCRIBED_TOPICS 10

const char* ssid = STASSID;
const char* password = STAPSK;

StaticJsonDocument<64> PINS;
StaticJsonDocument<64> STATE;

int temperaturePin = D4;
DHT myDHT(temperaturePin, DHT11);
int tempInterval = 0;
String tempAttId;

String doorState = "closed";
int doorPin = D3;
int doorInterval = 0;
String doorAttId;

unsigned int prevTempMillis;
unsigned int prevDoorMillis;

String mac = WiFi.macAddress();
String mqttClientName = "ESP-" + mac;

EspMQTTClient client(
  "Yolo",
  "44141158",
  "192.168.1.100",  // MQTT Broker server ip
  "MQTTUsername",   // Can be omitted if not needed
  "MQTTPassword",   // Can be omitted if not needed
  mqttClientName.c_str(),     // Client name that uniquely identify your device
  1883              // The MQTT port, default to 1883. this line can be omitted
);

void turnOnLight(String pin) {
  if (STATE[pin] == LOW) {
    for (int i = 0; i < 1024; i++) {
      analogWrite(PINS[pin], i);
      delay(1);
    }
    digitalWrite(PINS[pin], HIGH);
    STATE[pin] = HIGH;
  }
}

void turnOffLight(String pin) {
  if (STATE[pin] == HIGH) {
    for (int i = 1023; i >= 0; i--) {
      analogWrite(PINS[pin], i);
      delay(1);
    }
    digitalWrite(PINS[pin], LOW);
    STATE[pin] = LOW;
  }
}

void turnOnSocket(String pin) {
  digitalWrite(PINS[pin], HIGH);
}

void turnOffSocket(String pin) {
  digitalWrite(PINS[pin], LOW);
}

void onConnectionEstablished()
{
  client.subscribe("global/discovery", [](const String & topic, const String & payload) {
    client.publish("global/discoveryResponse", "{\"deviceType\": \"ESP8266\",\"macAddress\": \"" + WiFi.macAddress() + "\"}");
  });

  client.subscribe("global/" + WiFi.macAddress(), [](const String & topic, const String & payload) {
    // Parse payload
    StaticJsonDocument<512> doc;
    deserializeJson(doc, payload);

    // Potencially Unsubscribe from old topics here

    // Subscribe to new topics
    String deviceId = doc["deviceId"];
    String attachmentId = doc["attachmentId"];
    String attachmentType = doc["attachmentType"];
    String pin = doc["pin"];
    bool value = doc["value"];

    // Check pin
    if (PINS[pin] == NULL) client.publish("global/error", "{\"error\": \"Error: Pin " + pin + " is not available on device: " + WiFi.macAddress() + "\"}");


    if (attachmentType == "light") {
      pinMode(PINS[pin], OUTPUT);
      digitalWrite(PINS[pin], LOW);
      client.subscribe("lights/" + deviceId + "/" + pin, [pin](const String & topic, const String & payload) {
        if (payload == "on") turnOnLight(pin);
        if (payload == "off") turnOffLight(pin);
      });
    }

    if (attachmentType == "socket") {
      pinMode(PINS[pin], OUTPUT);
      digitalWrite(PINS[pin], LOW);
      client.subscribe("sockets/" + deviceId + "/" + pin, [pin](const String & topic, const String & payload) {
        if (payload == "on") turnOnSocket(pin);
        if (payload == "off") turnOffSocket(pin);
      });
    }

    if (attachmentType == "temperature-sensor") {
      tempInterval = doc["tempInterval"];
      tempAttId = attachmentId;
      pinMode(PINS["D4"], INPUT_PULLUP);
    }

    if (attachmentType == "door-sensor") {
      doorInterval = doc["doorInterval"];
      doorAttId = attachmentId;
      pinMode(PINS["D3"], INPUT_PULLUP);
    }

  });

  String isOnlineMessage = "{\"macAddress\": \"" + WiFi.macAddress() + "\",\"ipAddress\":\"" + WiFi.localIP().toString() + "\",\"isOnline\":true}";

  client.subscribe("global/reportOnlineState", [isOnlineMessage](const String & topic, const String & payload) {
    client.publish("global/deviceState", isOnlineMessage, true);
  });

  client.publish("global/deviceState", isOnlineMessage, true);
}

void setupOTA() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
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
  // ArduinoOTA.setPassword("admin");

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

  // Last will
  char message[100];
  StaticJsonDocument<128> doc;
  doc["macAddress"] = mac;
  doc["isOnline"] = false;
  serializeJson(doc, message);
  client.enableLastWillMessage("global/deviceState", message, true);

  setupOTA();

  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = 0; i <= 10; i++) {
    digitalWrite(LED_BUILTIN, i % 2);
    delay(100);
  }
  digitalWrite(LED_BUILTIN, LOW);

  prevTempMillis = prevDoorMillis = millis();
  Serial.println("All systems GO");
}

void loop() {
  ArduinoOTA.handle();
  client.loop();

  unsigned int currentMillis = millis();

  if (tempInterval != 0 && tempAttId != NULL) {
    if (currentMillis - prevTempMillis > tempInterval) {
      StaticJsonDocument<512> doc;
      float temp = myDHT.readTemperature();
      float hum = myDHT.readHumidity();

      if (isnan(temp) || isnan(hum)) {
        String message = "{\"error\": \"Error reading temperature\", \"values\":[" + String(temp) + "," + String(hum) + "]}";
        Serial.println(message);
        client.publish("global/error", message);
      } else {
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
  if (doorInterval != 0 && doorAttId != NULL) {
    if (currentMillis - prevDoorMillis > doorInterval) {
      if (digitalRead(doorPin) == LOW) doorState = "open";
      else doorState = "closed";
      client.publish("global/door", "{\"attachmentId\":\"" + doorAttId + "\",\"state\":\"" + doorState + "\"}");
      prevDoorMillis = currentMillis;
    }
  }
}
