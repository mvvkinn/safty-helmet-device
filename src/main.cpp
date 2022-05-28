#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <PubSubClientTools.h>
#include <ArduinoJson.h>

#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>

#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

#include <DHT.h>

#include <config.h>

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

const char *helmet_id = "1";

// Setting Network
const char *wifi_ssid = WIFI_SSID;
const char *wifi_passwd = WIFI_PASSWD;
const char *mqtt_server = MQTT_HOST;
const int mqtt_port = MQTT_PORT;
const char *mqtt_user = MQTT_USER;
const char *mqtt_passwd = MQTT_PASSWD;

WiFiClient ESPClient;
PubSubClient client(ESPClient);
PubSubClientTools mqtt(client); // Easier to use subscribe and callback

// Setting DHT11 temperature humidity
#define DHTPIN 15
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
float humid, temp;

// GPS Setting
static const int GPSRX = 16, GPSTX = 17;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial ss(GPSRX, GPSTX);
float latitude, longitude;
// float latitude = 37.58510543, longitude = 126.92524348;

int buzzer_pin = 32;

// Setting photoresistor lightness detection
int photoresistor_pin = 34;
int photo_val = 0;

// Setting SW-420 shock detection
int shock_pin = 18;
bool shock_val = false;

// Setting HC-SR04 distance
int uw_trig_pin = 14;
int uw_echo_pin = 12;
long duration, distance;

// Setting LED
int led_red = 32;
int led_green = 33;
int led_blue = 25;
int led_front = 19;

bool worker_danger = false;

// Setting OLED Display
#define OLED_ADDR 0x3c
Adafruit_SSD1306 display(-1); // -1 = no reset pin

// FreeRTOS multitask variable
// SemaphoreHandle_t xMutex;

static void gpsDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

static float getGPSFloat(float val, bool valid)
{
  if (!valid)
  {
    return 99.0;
  }
  else
  {
    return val;
  }
  gpsDelay(0);
}

// show texts OLED display with color,
void display_showText(String msg, uint16_t color)
{
  display.clearDisplay();
  display.setTextColor(color);
  display.setCursor(0, 16);
  display.println(msg);
  display.display();
}

// Callback from MQTT Message
void message_callback(String topic, String message)
{
  Serial.println("스마트안전모 > " + topic + " : " + message);

  if (topic == "helmet" && message == "Online")
  {
    display_showText(message, WHITE);
  }
}

// WiFi, Network connection
void network_conn(void *parameter)
{
  while (1)
  {
    // Connect to WiFi when not connected
    while (!client.connected() || WiFi.status() != WL_CONNECTED)
    {
      WiFi.begin(wifi_ssid, wifi_passwd);
      Serial.print("Connecting to WiFi");

      display_showText("Connecting to WiFi..", WHITE);

      while (WiFi.status() != WL_CONNECTED)
      {
        delay(500);
        Serial.print(".");
      }
      Serial.println("\nConnected to WiFi");

      // Connect to MQTT when not connected
      client.setServer(mqtt_server, mqtt_port);
      Serial.print("Connecting to MQTT");
      while (!client.connected())
      {
        Serial.print(".");

        display_showText("Connecting to Server..", WHITE);

        if (client.connect("ESP32Client", mqtt_user, mqtt_passwd))
        {
          Serial.println("\nConnected to MQTT");

          display_showText("Connected", WHITE);

          mqtt.subscribe("helmet", message_callback);
        }
        else
        {
          Serial.print("Error > Failed with state ");
          Serial.println(client.state());

          display_showText(String(client.state()), WHITE);

          delay(2000);
        }
      }
      mqtt.publish("helmet", "Online");
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// Convert data to JSON and publish
void dataToJson_publish(void *parameter)
{
  while (1)
  {
    String json_data;

    StaticJsonDocument<256> doc;

    doc["helmet_id"] = helmet_id;
    doc["latitude"] = latitude;
    doc["longitude"] = longitude;
    doc["humid"] = humid;
    doc["temp"] = temp;
    doc["photoresistor"] = photo_val;
    doc["distance"] = distance;
    doc["shock"] = shock_val;

    /*
    JsonObject sensor = doc.createNestedObject("sensor");

    JsonObject sensor_gps = sensor.createNestedObject("gps"); // "sensor" : {"gps" : [latitude, longitude]}
    sensor_gps["latitude"] = latitude;
    sensor_gps["longitude"] = longitude;

    JsonObject sensor_dht = sensor.createNestedObject("dht"); // "sensor":  {"dht" : [humid, temp] }
    sensor_dht["humid"] = humid;
    sensor_dht["temp"] = temp;

    sensor["photoresistor"] = photo_val;
    sensor["distance"] = distance;

    sensor["shock"] = shock_val;
    */

    doc["worker_danger"] = worker_danger;

    serializeJson(doc, json_data);

    mqtt.publish("helmet", json_data);

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// Read temp humid from DHT11
void dhtState(void *parameter)
{
  while (1)
  {
    float t_humid = dht.readHumidity();
    delay(500);
    float t_temp = dht.readTemperature();

    if (!isnan(t_humid) || !isnan(t_temp))
    {
      humid = t_humid;
      temp = t_temp;
    }

    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

// Get latitude, longitude from GPS Sensor
void gpsState(void *parameter)
{
  while (1)
  {
    latitude = getGPSFloat(gps.location.lat(), gps.location.isValid());
    longitude = getGPSFloat(gps.location.lng(), gps.location.isValid());

    while (client.connected())
    {
      display.clearDisplay();
      display.setTextColor(WHITE);
      display.setCursor(0, 16);
      display.print("lat : ");
      display.println(latitude);
      display.print("lng : ");
      display.println(longitude);
      display.display();
    }

    gpsDelay(1000);

    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

// Read lightness from photoresistor
void photoresistor(void *parameter)
{
  while (1)
  {
    photo_val = analogRead(photoresistor_pin);

    vTaskDelay(600 / portTICK_PERIOD_MS);
  }
}

// Read shock from SW-240
void shock(void *parameter)
{
  while (1)
  {
    int count = 0;
    if (digitalRead(shock_pin) == HIGH)
    {
      shock_val = true;
    }
    else
    {
      shock_val = false;
    }

    vTaskDelay(800 / portTICK_PERIOD_MS);
  }
}

// Read distance from HC-SR04
void uw_distance(void *parameter)
{
  while (1)
  {
    digitalWrite(uw_trig_pin, LOW);
    delayMicroseconds(2);
    digitalWrite(uw_trig_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(uw_trig_pin, LOW);

    duration = pulseIn(uw_echo_pin, HIGH);
    distance = duration / 29.1 / 2;

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// RGB led
void rgb(bool red, bool green, bool blue)
{
  digitalWrite(led_red, red);
  digitalWrite(led_green, green);
  digitalWrite(led_blue, blue);
}

// Change bool worker_danger to true when shock detected
void workerSituation(void *parameter)
{
  while (1)
  {
    // change to worker_danger = shock && gyro_shock
    // gyro_shock == if there is no changes after shock
    if (shock_val == true)
    {
      // this informs admin worker is in danger
      worker_danger = true;
      digitalWrite(buzzer_pin, HIGH);
      // rgb(1, 0, 0);
      delay(500);
    }
    else
    {
      worker_danger = false;
      digitalWrite(buzzer_pin, LOW);
    } // add distance, and more

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

int i = 0;

void setup()
{
  Serial.begin(115200);
  ss.begin(GPSBaud);

  dht.begin();

  pinMode(led_red, OUTPUT);
  pinMode(led_green, OUTPUT);
  pinMode(led_blue, OUTPUT);

  pinMode(shock_pin, INPUT);

  pinMode(uw_trig_pin, OUTPUT);
  pinMode(uw_echo_pin, INPUT);

  pinMode(buzzer_pin, OUTPUT);

  // setup OLED Display
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay();
  display.display();

  xTaskCreate(network_conn, "network_conn", 4000, NULL, 10, NULL);
  xTaskCreate(photoresistor, "photoresistor", 4000, NULL, 10, NULL);
  xTaskCreate(dhtState, "dhtState", 4000, NULL, 10, NULL);
  xTaskCreate(shock, "shock", 4000, NULL, 10, NULL);
  xTaskCreate(uw_distance, "uw_distance", 4000, NULL, 10, NULL);
  xTaskCreate(dataToJson_publish, "dataToJson_Publish", 4000, NULL, 10, NULL);
  xTaskCreate(gpsState, "gpsState", 4000, NULL, 10, NULL);
  xTaskCreate(workerSituation, "workerSituation", 4000, NULL, 10, NULL);
}

void loop()
{
  client.loop();
}
