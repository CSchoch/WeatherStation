#include <Arduino.h>

/***************************************************************************************************/
/* 
  Example for ROHM BH1750FVI Ambient Light Sensor library

  Power supply voltage: 2.4 - 3.6v
  Default range:        1 - 65'535 lux
  Measurement accuracy: ±20%, possible to calibrate
  Peak wave length:     560nm, yellow-green

  written by : enjoyneering79
  sourse code: https://github.com/enjoyneering/

  This chip uses I2C bus to communicate, specials pins are required to interface
  Board:                                    SDA                    SCL                    Level
  Uno, Mini, Pro, ATmega168, ATmega328..... A4                     A5                     5v
  Mega2560................................. 20                     21                     5v
  Due, SAM3X8E............................. 20                     21                     3.3v
  Leonardo, Micro, ATmega32U4.............. 2                      3                      5v
  Digistump, Trinket, ATtiny85............. 0/physical pin no.5    2/physical pin no.7    5v
  Blue Pill, STM32F103xxxx boards.......... PB7                    PB6                    3.3v/5v
  ESP8266 ESP-01........................... GPIO0/D5               GPIO2/D3               3.3v/5v
  NodeMCU 1.0, WeMos D1 Mini............... GPIO4/D2               GPIO5/D1               3.3v/5v
  ESP32.................................... GPIO21/D21             GPIO22/D22             3.3v

  Frameworks & Libraries:
  ATtiny Core           - https://github.com/SpenceKonde/ATTinyCore
  ESP32 Core            - https://github.com/espressif/arduino-esp32
  ESP8266 Core          - https://github.com/esp8266/Arduino
  ESP8266 I2C lib fixed - https://github.com/enjoyneering/ESP8266-I2C-Driver
  STM32 Core            - https://github.com/rogerclarkmelbourne/Arduino_STM32

  GNU GPL license, all text above must be included in any redistribution, see link below for details:
  - https://www.gnu.org/licenses/licenses.html
*/
/***************************************************************************************************/
#define LED_PIN 2
#define DEBUGLEVEL NONE
#define MAX_PACKET_SIZE 256 // Max data packet size

#define uS_TO_S_FACTOR 1000000 // Conversion factor for micro seconds to seconds
#define TIME_TO_SLEEP 60       // Time ESP32 will go to sleep (in seconds)

#include <WiFi.h>
#include "OTA.h"
#include <Wire.h>
#include <SPI.h>
#include <BH1750FVI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <PubSubClient.h> // https://github.com/knolleary/pubsubclient/blob/master/examples/mqtt_esp8266/mqtt_esp8266.ino
#include <HardwareSerial.h>
#include <DebugUtils.h>
#include <ArduinoJson.h>

//#include "Config.h" // make your own config file or remove this line and use the following lines
const char *clientId = "WinterGarden";
const char *mqtt_server = "192.168.2.64";
#include "WifiCredentials.h"       // const char* ssid = "MySSID"; const char* WifiPassword = "MyPw";
IPAddress ip(192, 168, 2, 5);      // Static IP
IPAddress dns(192, 168, 2, 1);     // most likely your router
IPAddress gateway(192, 168, 2, 1); // most likely your router
IPAddress subnet(255, 255, 255, 0);

unsigned long lastUpdated;
unsigned long lastLed;
const char *nameprefix = "WinterGarden";
uint8_t stateLed = HIGH;
boolean updateActive;
RTC_DATA_ATTR unsigned long bootCount;
RTC_DATA_ATTR boolean enableUpdate;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

/*
BH1750FVI(address, resolution, sensitivity, accuracy)

BH1750_DEFAULT_I2CADDR            - address pin LOW
BH1750_SECOND_I2CADDR             - address pin HIGH

BH1750_CONTINUOUS_HIGH_RES_MODE   - continuous measurement, 1.0 lx resolution
BH1750_CONTINUOUS_HIGH_RES_MODE_2 - continuous measurement, 0.5 lx resolution
BH1750_CONTINUOUS_LOW_RES_MODE    - continuous measurement, 0.5 lx resolution
BH1750_ONE_TIME_HIGH_RES_MODE     - one measurement & power down, 1.0 lx resolution
BH1750_ONE_TIME_HIGH_RES_MODE_2   - one measurement & power down, 0.5 lx resolution
BH1750_ONE_TIME_LOW_RES_MODE      - one measurement & power down, 4.0 lx resolution

sensitivity                       - value have to be between 0.45 - 3.68, default 1.00 or use macros BH1750_SENSITIVITY_DEFAULT
accuracy                          - value have to be between 0.96 - 1.44, default 1.20 or use macros BH1750_ACCURACY_DEFAULT
*/
BH1750FVI LightSensor(BH1750_DEFAULT_I2CADDR, BH1750_CONTINUOUS_HIGH_RES_MODE_2, BH1750_SENSITIVITY_DEFAULT, BH1750_ACCURACY_DEFAULT);
Adafruit_BME280 InsideSensor;  // I2C
Adafruit_BME280 OutsideSensor; // I2C

void setup_wifi()
{
  const int maxlen = 40;
  char fullhostname[maxlen];
  uint8_t mac[6];
  delay(10);
  DEBUGPRINTNONE("Connecting to ");
  DEBUGPRINTLNNONE(ssid);

  WiFi.macAddress(mac);
  snprintf(fullhostname, maxlen, "%s-%02x%02x%02x", nameprefix, mac[3], mac[4], mac[5]);
  WiFi.setHostname(fullhostname);
  WiFi.config(ip, dns, gateway, subnet);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, WifiPassword);
  int counter = 0;
  while (WiFi.status() != WL_CONNECTED)
  {
    DEBUGPRINTNONE(".");
    counter++;
    unsigned long timer = millis();
    while (millis() - timer <= 500)
    {
    }
    //delay(500);
    if (counter >= 20)
    {
      counter = 0;
      DEBUGPRINTLNNONE("Retry");
      WiFi.disconnect();
      while (WiFi.status() == WL_CONNECTED)
      {
        DEBUGPRINTNONE(".");

        delay(10);
      }
      WiFi.begin(ssid, WifiPassword);
    }
  }
  DEBUGPRINTLNNONE("WiFi connected");
  DEBUGPRINTNONE("IP address: ");
  DEBUGPRINTLNNONE(WiFi.localIP());

  if (enableUpdate)
  {
    setupOTA(fullhostname);
  }
}

void wifiReconnect()
{
  WiFi.disconnect();
  while (WiFi.status() == WL_CONNECTED)
  {
    DEBUGPRINTNONE(".");
    delay(10);
  }
  DEBUGPRINTNONE("Reconnecting to ");
  DEBUGPRINTLNNONE(ssid);
  int counter = 0;
  WiFi.begin(ssid, WifiPassword);
  while (WiFi.status() != WL_CONNECTED and counter < 20)
  {
    DEBUGPRINTNONE(".");
    counter++;
    unsigned long timer = millis();
    while (millis() - timer <= 500)
    {
    }
    //delay(500);
  }
  DEBUGPRINTLNNONE("WiFi connected");
  DEBUGPRINTNONE("IP address: ");
  DEBUGPRINTLNNONE(WiFi.localIP());
}

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  char *topic1 = "/enableUpdate";
  char *path = (char *)malloc(1 + strlen(clientId) + strlen(topic1));
  strcpy(path, clientId);
  strcat(path, topic1);
  //if (topic == path) {
  DEBUGPRINTLNNONE("NewMessage");
  DEBUGPRINTLNNONE(topic);
  DEBUGPRINTLNNONE(length);
  free(path);
  enableUpdate = true;
  topic = "/enableUpdateAck";
  path = (char *)malloc(1 + strlen(clientId) + strlen(topic));
  strcpy(path, clientId);
  strcat(path, topic);
  mqttClient.publish(path, "true");
  free(path);
  delay(100);
  //}
}

void mqttReconnect()
{
  // Loop until we're reconnected
  while (!mqttClient.connected())
  {
    if (WiFi.status() != WL_CONNECTED)
    {
      wifiReconnect();
    }
    DEBUGPRINTNONE("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect(clientId))
    {
      DEBUGPRINTLNNONE("connected");
      // ... and resubscribe
      char *topic = "/enableUpdate";
      char *path = (char *)malloc(1 + strlen(clientId) + strlen(topic));
      strcpy(path, clientId);
      strcat(path, topic);
      DEBUGPRINTLNNONE(path);
      mqttClient.subscribe(path);
      free(path);
    }
    else
    {
      DEBUGPRINTNONE("failed, rc=");
      DEBUGPRINTNONE(mqttClient.state());
      DEBUGPRINTLNNONE(" try again in 1 seconds");
      // Wait 1 seconds before retrying
      unsigned long timer = millis();
      while (millis() - timer <= 1000)
      {
      }
      //delay(1000);
    }
  }
}

void setup()
{
  Serial.begin(115200);
  DEBUGPRINTLNNONE("\nHardware serial started");
  bootCount++;
  updateActive = enableUpdate;
  lastUpdated = millis() - (TIME_TO_SLEEP * 1000);
  lastLed = millis();
  pinMode(LED_PIN, OUTPUT);
  DEBUGPRINTLNNONE(bootCount);

  if (!enableUpdate)
  {
    while (!InsideSensor.begin(0x76, &Wire))
    {
      DEBUGPRINTLNNONE("InsideSensor is not present");
      delay(5000);
    }
    DEBUGPRINTLNNONE("InsideSensor is present");
    InsideSensor.setSampling(Adafruit_BME280::MODE_FORCED,
                             Adafruit_BME280::SAMPLING_X1, // temperature
                             Adafruit_BME280::SAMPLING_X1, // pressure
                             Adafruit_BME280::SAMPLING_X1, // humidity
                             Adafruit_BME280::FILTER_OFF);

    while (!LightSensor.begin())
    {
      DEBUGPRINTLNNONE("Light Sensor is not present");
      delay(5000);
    }
    DEBUGPRINTLNNONE("Light Sensor is present");
    LightSensor.setResolution(BH1750_ONE_TIME_HIGH_RES_MODE_2);

    while (!OutsideSensor.begin(0x77, &Wire))
    {
      DEBUGPRINTLNNONE("OutsideSensor is not present");
      delay(5000);
    }
    DEBUGPRINTLNNONE("OutsideSensor is present");
    OutsideSensor.setSampling(Adafruit_BME280::MODE_FORCED,
                              Adafruit_BME280::SAMPLING_X1, // temperature
                              Adafruit_BME280::SAMPLING_X1, // pressure
                              Adafruit_BME280::SAMPLING_X1, // humidity
                              Adafruit_BME280::FILTER_OFF);
  }
  setup_wifi();
  mqttClient.setServer(mqtt_server, 1883);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(MAX_PACKET_SIZE);
}

void loop()
{
  if (enableUpdate && !updateActive)
  {
    esp_deep_sleep(10 * uS_TO_S_FACTOR);
  }
  else if (enableUpdate)
  {
    ArduinoOTA.handle();
    digitalWrite(LED_PIN, HIGH);
  }
  else
  {
    char Data[MAX_PACKET_SIZE];
    if (!mqttClient.connected())
    {
      mqttReconnect();
    }
    mqttClient.loop();

    if (millis() - lastUpdated >= TIME_TO_SLEEP * 1000)
    {
      lastUpdated = millis();
      const size_t capacity = JSON_OBJECT_SIZE(3) + JSON_OBJECT_SIZE(1) + 4 * JSON_OBJECT_SIZE(3);
      DynamicJsonDocument doc(capacity);

      float value;

      JsonObject Light = doc.createNestedObject("Light");
      value = LightSensor.readLightLevel();
      DEBUGPRINTNONE("Light level: ");
      DEBUGPRINTNONE(value);
      DEBUGPRINTLNNONE(" +-0.5 lx");
      Light["value"] = value;

      InsideSensor.takeForcedMeasurement();

      JsonObject TempIn = doc.createNestedObject("TempIn");
      value = InsideSensor.readTemperature();
      DEBUGPRINTNONE("Inside Temperature: ");
      DEBUGPRINTNONE(value);
      DEBUGPRINTLNNONE(" °C");
      TempIn["Temperature"] = value;
      value = InsideSensor.readHumidity();
      DEBUGPRINTNONE("Inside Humidity: ");
      DEBUGPRINTNONE(value);
      DEBUGPRINTLNNONE(" %");
      TempIn["Humidity"] = value;
      value = InsideSensor.readPressure() / 100.0F;
      DEBUGPRINTNONE("Inside Pressure: ");
      DEBUGPRINTNONE(value);
      DEBUGPRINTLNNONE(" hPa");
      TempIn["Pressure"] = value;

      OutsideSensor.takeForcedMeasurement();

      JsonObject TempOut = doc.createNestedObject("TempOut");
      value = OutsideSensor.readTemperature();
      DEBUGPRINTNONE("Outside Temperature: ");
      DEBUGPRINTNONE(value);
      DEBUGPRINTLNNONE(" °C");
      TempOut["Temperature"] = value;
      value = OutsideSensor.readHumidity();
      DEBUGPRINTNONE("Outside Humidity: ");
      DEBUGPRINTNONE(value);
      DEBUGPRINTLNNONE(" %");
      TempOut["Humidity"] = value;
      value = OutsideSensor.readPressure() / 100.0F;
      DEBUGPRINTNONE("Outside Pressure: ");
      DEBUGPRINTNONE(value);
      DEBUGPRINTLNNONE(" hPa");
      TempOut["Pressure"] = value;

      DEBUGPRINTNONE("MemUsage.........: ");
      DEBUGPRINTLNNONE(doc.memoryUsage());

      serializeJson(doc, Data, sizeof(Data));
      char *topic = "/Data";
      char *path = (char *)malloc(1 + strlen(clientId) + strlen(topic));
      strcpy(path, clientId);
      strcat(path, topic);
      if (!mqttClient.publish(path, Data, false))
      {
        lastUpdated = millis() - (TIME_TO_SLEEP * 1000);
        DEBUGPRINTLNNONE("MQTT publish failed");
      }
      free(path);

      DEBUGPRINTLNDEBUG(Data);

      LightSensor.reset();
      LightSensor.powerDown();
    }
  }
}
