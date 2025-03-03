// ESP32 Smart Irrigation Code with ThingsBoard Cloud

#include <WiFi.h>
#include <PubSubClient.h>
#include "DHT.h"

#define WIFI_SSID "Your WIFI"
#define WIFI_PASSWORD "Your WIFI Password"

#define TOKEN "Access Token from thingsboard" // Access token for ThingsBoard Cloud
#define THINGSBOARD_SERVER "thingsboard.cloud"

WiFiClient espClient;
PubSubClient client(espClient);

#define DHTPIN 4
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

const int soilMoisturePin = 35;
const int waterLevelPin = 34;
const int ledPin = 2;     // LED for soil moisture indication
const int pumpPin = 5;    // Pump control pin

const int SOIL_MOISTURE_LOW = 50;
const int SOIL_MOISTURE_HIGH = 50;
const int WATER_LEVEL_THRESHOLD = 30;  // Pump forced OFF if water level is below this

int pumpState = LOW;   // Current pump state (LOW = OFF, HIGH = ON)
int pumpMode = 1;      // Pump mode: 0 = Manual, 1 = Automatic
int waterLevel = 0;    // Mapped water level (0-100)
unsigned long previousMillis = 0;
const unsigned long interval = 1000;  // 1-second interval for readings
float mappedSoilMoisture = 0;  // Mapped soil moisture (0-100)

void setup() {
  Serial.begin(115200);
  
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");

  client.setServer(THINGSBOARD_SERVER, 1883);
  client.setCallback(mqttCallback);

  pinMode(ledPin, OUTPUT);
  pinMode(pumpPin, OUTPUT);
  dht.begin();

  reconnect();
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Connecting to ThingsBoard...");
    if (client.connect("ESP32Client", TOKEN, NULL)) {
      Serial.println(" connected");
      // Subscribe to attribute updates from ThingsBoard
      client.subscribe("v1/devices/me/attributes");
    } else {
      Serial.print(" failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(message);

  if (String(topic) == "v1/devices/me/attributes") {
    // Check for mode update ("state")
    if (message.indexOf("\"state\":0") >= 0) {
      pumpMode = 0;        // Switch to manual mode
      pumpState = LOW;     // Always start with pump OFF in manual mode
      Serial.println("Manual mode activated, pump set to OFF");
    } else if (message.indexOf("\"state\":1") >= 0) {
      pumpMode = 1;        // Switch to automatic mode
      Serial.println("Automatic mode activated");
    }
    
    // In manual mode, check for pump control ("state2")
    if (pumpMode == 0) {
      if (message.indexOf("\"state2\":1") >= 0) {
        pumpState = HIGH;
        Serial.println("Manual command: Pump ON");
      } else if (message.indexOf("\"state2\":0") >= 0) {
        pumpState = LOW;
        Serial.println("Manual command: Pump OFF");
      }
    }
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Read soil moisture sensor
    int soilSensorValue = analogRead(soilMoisturePin);
    mappedSoilMoisture = map(soilSensorValue, 2000, 1203, 0, 100);
    // Clamp soil moisture to 0-100
    if (mappedSoilMoisture < 0) {
      mappedSoilMoisture = 0;
    } else if (mappedSoilMoisture > 100) {
      mappedSoilMoisture = 100;
    }

    // Read water level sensor
    int waterSensorValue = analogRead(waterLevelPin);
    Serial.print("Water sensor raw value: ");
    Serial.println(waterSensorValue);
    waterLevel = map(waterSensorValue, 0, 2000, 0, 100);
    // Clamp water level to 0-100
    if (waterLevel < 0) {
      waterLevel = 0;
    } else if (waterLevel > 100) {
      waterLevel = 100;
    }

    // Pump control: Force pump OFF if water level is too low
    if (waterLevel < WATER_LEVEL_THRESHOLD) {
      pumpState = LOW;
      digitalWrite(pumpPin, pumpState);
      Serial.println("Pump forced OFF due to low water level.");
    } else {
      if (pumpMode == 0) {
        // Manual mode: use pumpState set by MQTT (via "state2")
        digitalWrite(pumpPin, pumpState);
      } else {
        // Automatic mode: control pump based on soil moisture
        autoControlPump(mappedSoilMoisture);
      }
    }

    // Control LED based on soil moisture for visual feedback
    controlLED(mappedSoilMoisture);

    // Read temperature and humidity from the DHT sensor, and send telemetry
    readAndSendSensorData();

    // Print status information to the serial monitor
    printStatus();
  }
}

void autoControlPump(float moisture) {
  // Automatic control based on soil moisture thresholds
  if (moisture <= SOIL_MOISTURE_LOW) {
    pumpState = HIGH;
  } else if (moisture >= SOIL_MOISTURE_HIGH) {
    pumpState = LOW;
  }
  digitalWrite(pumpPin, pumpState);
}

void controlLED(float moisture) {
  // LED indication: turn on if soil moisture is low, off otherwise
  if (moisture <= SOIL_MOISTURE_LOW) {
    digitalWrite(ledPin, HIGH);
  } else if (moisture >= SOIL_MOISTURE_HIGH) {
    digitalWrite(ledPin, LOW);
  }
}

void readAndSendSensorData() {
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();
  if (!isnan(humidity) && !isnan(temperature)) {
    // Format the telemetry payload as JSON
    String payload = "{";
    payload += "\"temperature\":" + String(temperature) + ",";
    payload += "\"humidity\":" + String(humidity) + ",";
    payload += "\"soilMoisture\":" + String(mappedSoilMoisture) + ",";
    payload += "\"waterLevel\":" + String(waterLevel) + ",";
    payload += "\"pumpState\":" + String(pumpState);
    payload += "}";
    
    client.publish("v1/devices/me/telemetry", payload.c_str());
    Serial.println("Data sent to ThingsBoard: " + payload);
  } else {
    Serial.println("Failed to read from DHT sensor!");
  }
}

void printStatus() {
  // Print current mode, pump state, and sensor readings to the serial monitor
  Serial.print("Mode: ");
  Serial.print(pumpMode == 0 ? "Manual" : "Automatic");
  Serial.print(" | Pump State: ");
  Serial.print(pumpState == HIGH ? "ON" : "OFF");
  Serial.print(" | Soil Moisture: ");
  Serial.print(mappedSoilMoisture);
  Serial.print("% | Water Level: ");
  Serial.print(waterLevel);
  Serial.println("%");
}

