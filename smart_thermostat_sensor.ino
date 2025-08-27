#include <ESP8266WiFi.h>       // WiFi library for ESP8266
#include <ESP8266mDNS.h>       // Multicast DNS for easy network discovery
#include <Wire.h>              // I2C library
#include <Adafruit_Sensor.h>   // Base sensor library from Adafruit
#include <Adafruit_BME280.h>   // BME280 sensor library
#include <ArduinoOTA.h>        // Over-the-air update support

Adafruit_BME280 bme;          // Create BME280 sensor object

// WiFi credentials
const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";

WiFiServer server(80);        // Create a TCP server on port 80

unsigned long previousMillis = 0; // Stores last sensor read time
const long interval = 5000;       // Sensor read interval (ms)
unsigned long startMillis = 0;    // Stores start time

// Sensor readings
float t = 0.0;  // Temperature
float h = 0.0;  // Humidity
float p = 0.0;  // Pressure

// Connect to WiFi network
void connectToWiFi() {
  WiFi.mode(WIFI_STA);       // Set WiFi to station mode
  WiFi.begin(ssid, password);

  int retryCount = 0;
  // Wait for connection or timeout after 20 attempts
  while (WiFi.status() != WL_CONNECTED && retryCount < 20) {
    delay(500);
    retryCount++;
  }
}

// Setup OTA updates
void setupOTA() {

  // Set OTA hostname; increment number for each sensor
  // (e.g., esp01-1, esp01-2, esp01-3, etc.)
  ArduinoOTA.setHostname("esp01-1");  


  // OTA event handlers (empty but could log events)
  ArduinoOTA.onStart([]() {});
  ArduinoOTA.onEnd([]() {});
  ArduinoOTA.onProgress([](unsigned int, unsigned int) {});
  ArduinoOTA.onError([](ota_error_t) {});

  ArduinoOTA.begin(); // Start OTA service
}

void setup() {
  Wire.begin(0, 2);       // Initialize I2C on pins SDA=0, SCL=2
  bme.begin(0x76);        // Initialize BME280 at I2C address 0x76

  connectToWiFi();         // Connect to WiFi

  if (WiFi.status() == WL_CONNECTED) {
    setupOTA();            // Enable OTA if WiFi connected
    MDNS.begin("esp01-3"); // Start mDNS responder for network discovery
    server.begin();        // Start TCP server
  }

  startMillis = millis();  // Record the start time
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    ArduinoOTA.handle();   // Handle OTA updates
    MDNS.update();         // Update mDNS responder

    unsigned long currentMillis = millis();
    // Read sensor at defined interval
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;

      t = bme.readTemperature();
      h = bme.readHumidity();
      p = bme.readPressure() / 100.0F; // Convert Pa to hPa
    }

    // Handle incoming HTTP clients
    WiFiClient client = server.available();
    if (client) {
      String request = client.readStringUntil('\r'); // Read request
      client.flush();

      // Send plain text HTTP response with sensor data
      client.println("HTTP/1.1 200 OK");
      client.println("Content-type:text/plain");
      client.println();
      client.println(h);
      client.println(t);
      client.println(p);
      delay(10); // Short delay for stability
    }
  } else {
    connectToWiFi(); // Reconnect if WiFi lost
  }
}
