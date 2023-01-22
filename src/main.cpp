#include <Arduino.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include <PubSubClient.h>

#include "settings.h"

#define echoPin 15 //13 // attach pin D13 Arduino to pin Echo of JSN-SR04T
#define trigPin 27 //12 //attach pin D12 Arduino to pin Trig of JSN-SR04T
#define LED_BUILTIN 13

#define uS_TO_S_FACTOR 1000000   /* Conversion factor for micro seconds to seconds */

RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR int distance_prev = 0;
RTC_DATA_ATTR bool prev_wifistate = false;
RTC_DATA_ATTR struct  {
  int time_to_sleep = TIME_TO_SLEEP;
  int heartbeat = HEARTBEAT;
  int wifi_timeout = WIFI_TIMEOUT;
  int sensor_timeout = SENSOR_TIMEOUT;
} settings;

WiFiClient wificlient;
PubSubClient mqttclient(wificlient);

bool done = false;
byte msgBuf[64];
uint32_t msgLen;

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Received message on topic ");
  Serial.print(topic);
  Serial.print(": ");
  Serial.println(String((char*)payload, length));

  if (strcmp(topic, MQTT_SETTINGS_TOPIC "/time_to_sleep") == 0) {
    if (settings.time_to_sleep != String((char*)payload, length).toInt()) {
      settings.time_to_sleep = String((char*)payload, length).toInt();
      Serial.print("Setting time_to_sleep to ");
      Serial.println(settings.time_to_sleep);
    }
    return;
  } 

  if (strcmp(topic, MQTT_SETTINGS_TOPIC "/heartbeat") == 0) {
    if (settings.heartbeat != String((char*)payload, length).toInt()) {
      settings.heartbeat = String((char*)payload, length).toInt();
      Serial.print("Setting heartbeat to ");
      Serial.println(settings.heartbeat);
    }
    return;
  }

  if (strcmp(topic, MQTT_SETTINGS_TOPIC "/wifi_timeout") == 0) {
    if (settings.wifi_timeout != String((char*)payload, length).toInt()) {
      settings.wifi_timeout = String((char*)payload, length).toInt();
      Serial.print("Setting WIFI timeout to ");
      Serial.println(settings.wifi_timeout);
    }
    return;
  }

  if (strcmp(topic, MQTT_SETTINGS_TOPIC "/sensor_timeout") == 0) {
    if (settings.sensor_timeout != String((char*)payload, length).toInt()) {
      settings.sensor_timeout = String((char*)payload, length).toInt();
      Serial.print("Setting Sensor timeout to ");
      Serial.println(settings.sensor_timeout);
    }
    return;
  }


  if (strcmp(topic, MQTT_SETTINGS_TOPIC "/ping") == 0 && 
           String((char*)payload, length).toInt() == bootCount) {
    done = true;
    Serial.println("ping received");
    return;
  }
}

int measure_distance(){
  int duration = 0;
  int distance = 0;
  int retries = 0;

  // Clears the trigPin condition
  digitalWrite(trigPin, LOW);  //
  delayMicroseconds(2);

  // retry measurement until it is within acceptable range or until timeout is reached
  while((distance < SENSOR_RANGE_MIN || distance > SENSOR_RANGE_MAX) &&
        retries <= settings.sensor_timeout) {
  
    Serial.println("Waking up AJ-SR04M module");
    // Sets the trigPin HIGH (ACTIVE) for 1000 microseconds to wake module
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(trigPin, LOW);

    Serial.println("Read measured distance");
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);
    // Calculating the distance
    distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)

    // Displays the distance on the Serial Monitor
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    if (distance < SENSOR_RANGE_MIN || distance > SENSOR_RANGE_MAX) {
      Serial.println("Distance out of range.");
      distance = 0;
      if (retries < settings.sensor_timeout) {
        delay(50); // wait for module to settle before retrying
      } 
    }
    ++retries;
  }

  return distance;
}

int setup_wifi() {
  // Make sure Wifi settings in flash are off so it doesn't start automagically at next boot
  if (WiFi.getMode() != WIFI_OFF) {
    printf("Wifi wasn't off!\n");
    WiFi.persistent(true);
    WiFi.mode(WIFI_OFF);
  }

  // Connect to wifi
  WiFi.persistent(false); 
  WiFi.mode(WIFI_STA); //Optional
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to " WIFI_SSID);
  
  int retries = 0;
  while(WiFi.status() != WL_CONNECTED && retries < settings.wifi_timeout * 10){
    Serial.print(".");
    ++retries;
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
    delay(50);
  }
 
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Failed!!");
    prev_wifistate = false;
    return 0;
  }
  
  prev_wifistate = true;
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Connected");
  Serial.print("Local ESP32 IP: ");
  Serial.println(WiFi.localIP());   
  return 1;
}

int connect_mqtt() {
  mqttclient.setServer(MQTT_HOST, MQTT_PORT);
  mqttclient.setCallback(mqttCallback);
  Serial.print("Connecting to MQTT server ");
  Serial.print(MQTT_HOST);
  Serial.print("...");
  String clientId = "ESP32-Waterput";
  if (mqttclient.connect(clientId.c_str())) {
    Serial.println("Connected");
    return 1;
  }
  Serial.println("Failed!!");
  return 0;
}

void deep_sleep() {
  Serial.println("Going to sleep now");

  // Explicitely stop wifi before sleep otherwise reconnect on wake fails 
  esp_wifi_stop();

  delay(20);
  Serial.flush(); 
  
  //Set wakeup timer
  esp_sleep_enable_timer_wakeup(settings.time_to_sleep * uS_TO_S_FACTOR);

  // Enter deep sleep
  esp_deep_sleep_start();
}

void setup() {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
  pinMode (LED_BUILTIN, OUTPUT);

  Serial.begin(9600); // // Serial Communication is starting with 9600 of baud rate speed
  Serial.println("\nUltrasonic Sensor HC-SR04M Waterput"); // print some text in Serial Monitor
  Serial.println("Settings:");
  Serial.print("- heartbeat: ");
  Serial.println(settings.heartbeat);
  Serial.print("- time_to_sleep: ");
  Serial.println(settings.time_to_sleep);
  Serial.print("- wifi_timeout: ");
  Serial.println(settings.wifi_timeout);
  Serial.print("- sensor_timeout: ");
  Serial.println(settings.sensor_timeout);

  //Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));

  int distance = measure_distance();
  if ((distance != distance_prev && distance != 0) || // don't send if no change or invalid measurement
      (bootCount * settings.time_to_sleep) % settings.heartbeat == 0 || // but do send on heartbeat (even if invalid)
      !prev_wifistate) {  // or when previous wifi connect attempt failed
    if (setup_wifi() && connect_mqtt()) {
      Serial.print("Publishing to " MQTT_PUBLISH_TOPIC ": ");
      Serial.println(distance);
      mqttclient.publish(MQTT_PUBLISH_TOPIC, String(distance).c_str(), true);

      Serial.print("Subscribing to " MQTT_SETTINGS_TOPIC "/#...");
      if (mqttclient.subscribe(MQTT_SETTINGS_TOPIC "/#")) {
        Serial.println("subscribed");
        delay(100);
        // Send ping to check if mqttclient receives messages
        Serial.println("Publishing ping");
        mqttclient.publish(MQTT_SETTINGS_TOPIC "/ping", String(bootCount).c_str());
        // Assume we received all messages when our ping message comes in
        // or stop trying at timeout
        Serial.println("Waiting for ping...");
        while(!done && millis() < 20*1000) {
          if (!mqttclient.connected()) {
            connect_mqtt();
          }
          mqttclient.loop();
        }
        if (!done) {
          Serial.println("Ping not received, giving up!!");
        }
      } else {
        Serial.println("failed!!");
      };
      
      // disconnect mqtt and wifi
      mqttclient.disconnect();
      //WiFi.disconnect();
      WiFi.mode(WIFI_OFF);
      digitalWrite(LED_BUILTIN, LOW);
    }
  }

  // Save distance for reference
  distance_prev = distance;
  
  deep_sleep();
}

void loop() {
  // Program will never come here as we enter deep sleep during setup.
}