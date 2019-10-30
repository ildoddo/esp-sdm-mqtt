/*
    esp-sdm-mqtt - patrik.mayer@codm.de

    Work in progress sketch to read power values via modbus from the well know
    SDMXXX (SDM120 / SDM220 / SDM630) series power meters from eastron over modbus
    using an esp8266 and an 3.3V RS485 tranceiver.

    This uses the SDM library by reaper7 https://github.com/reaper7/SDM_Energy_Meter
    mqtt client from https://github.com/knolleary/pubsubclient/
    Tasker from https://github.com/sticilface/Tasker

    Arduino OTA from ESP8266 Arduino Core V2.4

    As this is not finished yet, the documentation will get better by time.

*/

#include <Arduino.h>
#include <Tasker.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
//#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <SDM.h>
//#include <SoftwareSerial.h>
#include "RemoteDebug.h"
#include <ESP8266WebServer.h>
#include <WiFiManager.h>

//--------- Configuration
// WiFi
//const char* ssid = "IoT";
//const char* password = "55aa44bb33cc22dd11ee";
//MQTT
const char* mqttServer = "m23.cloudmqtt.com";
const char* mqttUser = "sgpjlnfw";
const char* mqttPass = "UwGiZrpG9uBS";
const char* mqttClientName = "SDM120"; //will also be used hostname and OTA name
const char* mqttTopicPrefix = "sensEnergy/SDM120/";
const long mqttport = 15288;

unsigned long measureDelay = 10000; //read every 60s

// internal vars
WiFiClient espClient;
PubSubClient mqttClient(espClient);
Tasker tasker;
//SoftwareSerial swSerSDM(13, 12); //D7, D6
SDM sdm(Serial, 2400, NOT_A_PIN, SERIAL_8N1, false);

char mqttTopicStatus[64];
char mqttTopicIp[64];

char mqttTopicVoltage[64];
char mqttTopicCurrent[64];
char mqttTopicPower[64];
char mqttTopicFreq[64];
char mqttTopicTotEner[64];

long lastReconnectAttempt = 0; //For the non blocking mqtt reconnect (in millis)
RemoteDebug Debug;
void meassureSDM();
void setup_wifi();
bool MqttReconnect();

void setup() {
  Serial.begin(115200);                                                         //initialize serial
  Serial.println("Starting...");

  sdm.begin();

  //put in mqtt prefix
  sprintf(mqttTopicStatus, "%sstatus", mqttTopicPrefix);
  sprintf(mqttTopicIp, "%sip", mqttTopicPrefix);
  sprintf(mqttTopicVoltage, "%svoltage", mqttTopicPrefix);
  sprintf(mqttTopicCurrent, "%scurrent", mqttTopicPrefix);
  sprintf(mqttTopicPower, "%spower", mqttTopicPrefix);
  sprintf(mqttTopicFreq, "%sfreq", mqttTopicPrefix);
  sprintf(mqttTopicTotEner, "%stotener", mqttTopicPrefix);

  //setup_wifi();
  // WiFiManager
  WiFiManager wifiManager;
  wifiManager.autoConnect("SDM120W","123456");

  mqttClient.setServer(mqttServer, mqttport);

  tasker.setInterval(meassureSDM, measureDelay);

  //----------- OTA
  ArduinoOTA.setHostname(mqttClientName);
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }
    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
    delay(1000);
    ESP.restart();
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();

  Debug.begin(mqttClientName);
  Debug.setSerialEnabled(true);
}

void loop() {
  //handle mqtt connection, non-blocking
  if (!mqttClient.connected()) {
    long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      // Attempt to reconnect
      if (MqttReconnect()) {
        lastReconnectAttempt = 0;
      }
    }
  }
  mqttClient.loop();
  tasker.loop();
  //handle OTA
  ArduinoOTA.handle();
  Debug.handle();
}

void meassureSDM() {
  float v = sdm.readVal(SDM120C_VOLTAGE);                                       //read voltage
  float c = sdm.readVal(SDM120C_CURRENT);                                       //read curren
  float p = sdm.readVal(SDM120C_POWER);                                         //read power
  float f = sdm.readVal(SDM120C_FREQUENCY);                                     //read frequency
  float te = sdm.readVal(SDM120C_TOTAL_ACTIVE_ENERGY)*1000;                          //read active energy

  if (!isnan(v)) {
    mqttClient.publish(mqttTopicVoltage, String(v, 2).c_str(), false);
  }

  if (!isnan(c)) {
    mqttClient.publish(mqttTopicCurrent, String(c, 2).c_str(), false);
  }

  if (!isnan(p)) {
    mqttClient.publish(mqttTopicPower, String(p, 2).c_str(), false);
  }

  if (!isnan(f)) {
    mqttClient.publish(mqttTopicFreq, String(f, 2).c_str(), false);
  }
  if (!isnan(te)) {
    mqttClient.publish(mqttTopicTotEner, String(te, 2).c_str(), false);
  }
  debugA("\nVoltage:   %sV\nCurrent:   %sA\nPower:     %sW\nFrequency: %sHz\n Energy: %skWh\n", String(v, 2).c_str(), String(c, 2).c_str(), String(p, 2).c_str(), String(f, 2).c_str(), String(te, 2).c_str());

}

// void setup_wifi() {
//   delay(10);
//   rdebugA("Connecting to ");
//   //rdebugAln(ssid);
//   WiFi.mode(WIFI_STA); //disable AP mode, only station
//   WiFi.hostname(mqttClientName);
//   WiFi.begin(ssid, password);
//   while (WiFi.status() != WL_CONNECTED) {
//     delay(500);
//     Serial.print(".");
//   }
//   rdebugA("");
//   rdebugA("WiFi connected");
//   rdebugA("IP address: ");
//   //rdebugAln(WiFi.localIP());
// }

bool MqttReconnect() {
  if (!mqttClient.connected()) {
    rdebugA("Attempting MQTT connection...");
    if (mqttClient.connect(mqttClientName, mqttUser, mqttPass, mqttTopicStatus, 1, true, "offline")) {
      rdebugA("connected");
    // Once connected, publish an announcement...
      char curIp[16];
      sprintf(curIp, "%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3]);
      mqttClient.publish(mqttTopicStatus, "online", true);
      mqttClient.publish(mqttTopicIp, curIp, true);
    } else {
      rdebugA("failed, rc=");
      //rdebugAln(mqttClient.state());
      rdebugA(" try again in 5 seconds");
    }
  }
  return mqttClient.connected();
}
