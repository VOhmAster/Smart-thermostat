
#include <U8g2lib.h>
#include <HTTPClient.h>
#include <ESP32Encoder.h>
#include <math.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <TimeLib.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include "esp_system.h"

#define BLYNK_TEMPLATE_ID    "YOUR_TEMPLATE_ID"
#define BLYNK_TEMPLATE_NAME  "YOUR_TEMPLATE_NAME"
#define BLYNK_AUTH_TOKEN     "YOUR_AUTH_TOKEN"


#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>


// These pins are assigned to Blynk Virtual Pins,
// used for writing sensor data to the Blynk app.
#define VPIN_TEMP   V4
#define VPIN_HUM    V5
#define VPIN_PRESS  V6


// NTP configuration
WiFiUDP ntpUDP;
NTPClient ntpClient(ntpUDP, "pool.ntp.org", 0, 60000); // UTC, update every 60 seconds

volatile bool buttonPressed = false;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 2000; // Debounce time

// Daylight saving time: From the last Sunday in March at 02:00 until the last Sunday in October at 02:00
bool isSummerTime(int year, int month, int day, int hour) {
  // Last Sunday of March
  int lastSundayMarch = 31 - ((5 + year * 5 / 4) % 7);
  // Last Sunday of October
  int lastSundayOctober = 31 - ((2 + year * 5 / 4) % 7);
  
  if ((month > 3 && month < 10) || 
      (month == 3 && (day > lastSundayMarch || (day == lastSundayMarch && hour >= 2))) || 
      (month == 10 && (day < lastSundayOctober || (day == lastSundayOctober && hour < 2)))) {
    return true;
  }
  return false;
}

int ConnectDev = 10; // Maximum number of sensors that can be operated
std::vector<String> sensorIPs(ConnectDev);

float GlobalT = 0, GlobalH = 0, GlobalP = 0, SetTemp = 20, TempRange = 0.5, NightSetTemp = 19, NightTimeMin = 1260, NightTimeMax = 360;
float previousP = 0, AverageTemp = 0;
int AvFixSet = 0, PressCount = 3; 
int OutdoorSet = 0, AverageTempOutdoor = 0;
bool relayState = LOW;
unsigned long lastCheckTime = 0;

char MintimeStr[6];
char MaxtimeStr[6];

int menuCount = 0;

#define CLK 5 // CLK ENCODER
#define DT 18 // DT ENCODER
#define SW 19 // SW ENCODER
#define RELAY_PIN 14  // GPIO pin for the relay IN connection

int buttonState;
int lastButtonState = HIGH;
int prevPosition = 0;
 
int NowHour = 0;
int NowMinute = 0;

ESP32Encoder encoder;

unsigned long previousMillis = 0; 
const unsigned long interval = 10000;  // Timer (10 seconds) to read sensor data

unsigned long menuStartTime = 0; // Start time for the menu timer
const unsigned long menuTimeout = 10000; // Timeout for resetting the menu to the default state

// WiFi settings
const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";

char ssidb[] = "YOUR_SSID";
char pass[] = "YOUR_PASSWORD";

// Maximum number of sensors that can be connected.
// The actual usable number may depend on network conditions and signal strength.
#define MAX_SENSORS 10

float Temperature[MAX_SENSORS];
float Humidity[MAX_SENSORS];
float Pressure[MAX_SENSORS];

// OLED display initialization
U8G2_SSD1309_128X64_NONAME0_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

void IRAM_ATTR handleButtonInterrupt() {
  buttonPressed = true;
}


void setup() {
  Blynk.begin(BLYNK_AUTH_TOKEN, ssidb, pass);
 
  pinMode(RELAY_PIN, OUTPUT);  // Set RELAY_PIN as output
  digitalWrite(RELAY_PIN, LOW); // Turn off the relay by default (LOW)
  
  pinMode(SW, INPUT);
  attachInterrupt(digitalPinToInterrupt(SW), handleButtonInterrupt, FALLING);
  encoder.attachHalfQuad(DT, CLK);
  encoder.setCount(0);
  
  // OLED display initialization
  u8g2.begin();
  u8g2.enableUTF8Print();
  u8g2.clearBuffer();

   // WiFi connection
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  // Wait until WiFi is connected
  while (WiFi.status() != WL_CONNECTED) {
    delay(2000);
   }
  ntpClient.begin();

   // Set OTA password
  ArduinoOTA.setPassword("otaadmin");

  // OTA events
  ArduinoOTA
  .onStart([]() {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 20, "Update started...");
    u8g2.sendBuffer();
  })
  .onEnd([]() {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 20, "Update finished!");
    u8g2.sendBuffer();
  })
  .onProgress([](unsigned int progress, unsigned int total) {
    char buf[32];
    uint8_t percent = progress * 100 / total;
    sprintf(buf, "Uploading: %u%%", percent);
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 20, "Updating...");
    u8g2.drawStr(0, 40, buf);
    u8g2.sendBuffer();
  })
  .onError([](ota_error_t error) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf);
    switch (error) {
      case OTA_AUTH_ERROR: u8g2.drawStr(0, 20, "Error: Auth"); break;
      case OTA_BEGIN_ERROR: u8g2.drawStr(0, 20, "Error: Begin"); break;
      case OTA_CONNECT_ERROR: u8g2.drawStr(0, 20, "Error: Conn"); break;
      case OTA_RECEIVE_ERROR: u8g2.drawStr(0, 20, "Error: Receive"); break;
      case OTA_END_ERROR: u8g2.drawStr(0, 20, "Error: End"); break;
    }
    u8g2.sendBuffer();
  });
  ArduinoOTA.begin();
 
  SensorDetect();

}

BLYNK_WRITE(V9) {
  SetTemp = param.asFloat(); 
   menuCount = 1;  
   menuStartTime = millis();
}
BLYNK_WRITE(V0) {
  AvFixSet = param.asFloat(); 
   menuCount = 7;  
   menuStartTime = millis();
}
BLYNK_WRITE(V10) {
  OutdoorSet = param.asFloat(); 
   menuCount = 8;  
   menuStartTime = millis();
}

BLYNK_WRITE(V11) {
   TempRange = param.asFloat(); 
   menuCount = 3;  
   menuStartTime = millis();
  
}

BLYNK_WRITE(V12) {
  NightSetTemp = param.asFloat(); 
  menuCount = 4;
  menuStartTime = millis();
}

BLYNK_WRITE(V14) {
  menuCount = 4;
  NightTimeMin = param.asFloat();
  int h = (int)(NightTimeMin / 60.0);
  int m = (int)fmod(NightTimeMin, 60.0);

  char buf[6];
  snprintf(buf, sizeof(buf), "%d:%02d", h, m);

  Blynk.virtualWrite(V16, String("Night start: ") + buf);
  menuCount = 5;
  menuStartTime = millis();

}

BLYNK_WRITE(V15) {
  menuCount = 4;
  NightTimeMax = param.asFloat();
  int h = (int)(NightTimeMax / 60.0);
  int m = (int)fmod(NightTimeMax, 60.0);

  char buf[6];
  snprintf(buf, sizeof(buf), "%d:%02d", h, m);

  Blynk.virtualWrite(V17, String("Night end: ") + buf);
  menuCount = 6;
  menuStartTime = millis();
}

void loop() {

Blynk.run();

 if (buttonPressed) {
    unsigned long now = millis();
    if (now - lastDebounceTime > debounceDelay) {
      lastDebounceTime = now;
      buttonPressed = false;
    menuStartTime = millis();

    switch (menuCount) {
        case 0: menuCount = 1; break;
        case 1: menuCount = 2; break;
        case 2: menuCount = 3; break;
        case 3: menuCount = 4; break;
        case 4: menuCount = 5; break;
        case 5: menuCount = 6; break;
        case 6: menuCount = 7; break;
        case 7: menuCount = 8; break;
        case 8: menuCount = 0; break;
    }
 }
}

lastButtonState = buttonState;

if (menuCount != 0 && (millis() - menuStartTime) > menuTimeout) {
    menuCount = 0; // Return to the main screen
}

int minHour, minMinute;
int maxHour, maxMinute;
sscanf(MintimeStr, "%d:%d", &minHour, &minMinute);
sscanf(MaxtimeStr, "%d:%d", &maxHour, &maxMinute);

int OnTime = (minHour * 60) + minMinute;
int OffTime = (maxHour * 60) + maxMinute;
OffTime += (OffTime < OnTime) ? (24 * 60) : 0;

ntpClient.update();
setTime(ntpClient.getEpochTime());
int utcHour = hour();
NowHour = utcHour;
NowMinute = minute();

if (isSummerTime(year(), month(), day(), utcHour)) {
    NowHour += 2;
} else {
    NowHour += 1;
}
if (NowHour >= 24) {
    NowHour -= 24;
}

int NowTime = (NowHour * 60) + NowMinute;
NowTime += (NowTime < OnTime) ? 24 * 60 : 0;

unsigned long currentMillis = millis();
if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    SensorData(Temperature, Humidity, Pressure, ConnectDev);
    updateSensorData();
}

switch (menuCount) {
    case 0: StartScreen(); break;
    case 1: TempSetting(); break;
    case 2: 
    displaySensorData(Temperature, Humidity, Pressure, ConnectDev); break;
    case 3: TemperatureRange(); break;
    case 4: NightSet(); break;
    case 5: NightTimeMinSet(); break;
    case 6: NightTimeMaxSet(); break;
    case 7: AverageOrFix(); break;
    case 8: OutdoorONOFF(); break;
}

switch (menuCount) {
    case 1:
        handleEncoderAdjustment(SetTemp, 0.5, 5, 40, TempSetting);
        break;
    case 3:
        handleEncoderAdjustment(TempRange, 0.1, 0.2, 0.5, TemperatureRange);
        break;
    case 4:
        handleEncoderAdjustment(NightSetTemp, 0.5, 5, 30, NightSet);
        break;
    case 5:
        handleEncoderAdjustment(NightTimeMin, 30, 0, 24 * 60, NightTimeMinSet);
        break;
    case 6:
        handleEncoderAdjustment(NightTimeMax, 30, 0, 24 * 60, NightTimeMaxSet);
        break;
    case 7:
        handleAvFixAdjustment();
        break;
    case 8:
        HandleOutdoorSensor();
        break;    
}

if (menuCount != 0 && (millis()- menuStartTime) > menuTimeout) {
    menuCount = 0;
    StartScreen();
}

// Conditions for mode based on time of day
if ((OnTime < OffTime && NowTime >= OnTime && NowTime < OffTime) || 
    (OnTime > OffTime && (NowTime >= OnTime || NowTime < OffTime))) {
    // Night mode
    if (GlobalT >= (NightSetTemp + TempRange) && relayState == HIGH) {
        digitalWrite(RELAY_PIN, LOW);
        relayState = LOW;
    } else if (GlobalT < (NightSetTemp - TempRange) && relayState == LOW) {
        digitalWrite(RELAY_PIN, HIGH);
        relayState = HIGH;
    }
} else {
    // Day mode
    if (GlobalT >= (SetTemp + TempRange) && relayState == HIGH) {
        digitalWrite(RELAY_PIN, LOW);
        relayState = LOW;
    } else if (GlobalT <= (SetTemp - TempRange) && relayState == LOW) {
        digitalWrite(RELAY_PIN, HIGH);
        relayState = HIGH;
    }
}
ArduinoOTA.handle();

}

void handleEncoderAdjustment(float& targetValue, float increment, float minValue, float maxValue, void (*menuFunction)()) {

  // Encoder handling
  
    long newPosition = encoder.getCount() / 2;

    targetValue += (newPosition > prevPosition) ? increment : (newPosition < prevPosition) ? -increment : 0;
    targetValue = constrain(targetValue, minValue, maxValue);

    if (newPosition != prevPosition) {
        menuStartTime = millis(); 
        menuFunction();  // Call the appropriate menu function
    }

    prevPosition = newPosition;
}

void handleAvFixAdjustment() {

   // Encoder handling for switching between average temperature and fixed temperature
   
    long newPosition = encoder.getCount() / 2;

    if (newPosition != prevPosition) {
        // Toggle between 0 and 1
        AvFixSet = (AvFixSet == 0) ? 1 : 0;  // If 0, set to 1, if 1, set to 0

        menuStartTime = millis();
        AverageOrFix();  // Call the AverageOrFix function
    }

    prevPosition = newPosition;
}


void HandleOutdoorSensor(){
 // Encoder handling for switching between average temperature and fixed temperature
   
    long newPosition = encoder.getCount() / 2;

    if (newPosition != prevPosition) {
        // Toggle between 0 and 1
        OutdoorSet = (OutdoorSet == 0) ? 1 : 0;  // If 0, set to 1, if 1, set to 0

        menuStartTime = millis();
        OutdoorONOFF();  // Call the OutdoorONOFF function
    }

    prevPosition = newPosition;
}

void OutdoorONOFF(){
// Select between average or single sensor temperature control
    
   u8g2.clearBuffer();
   u8g2.setFont(u8g2_font_6x12_me);
   u8g2.setCursor(10, 12);
   u8g2.print("Outdoor Temperature");
    
    if (OutdoorSet == 0) {
        // If 0, display "OFF"
        u8g2.setCursor(55, 30);
        u8g2.print("OFF   ");
        AverageTemp = AverageTemp;
    } else {
        // If 1, display "ON"
        u8g2.setCursor(55, 30);
        u8g2.print("ON    ");
        AverageTemp = AverageTempOutdoor;
    }
    u8g2.sendBuffer();
}

void AverageOrFix() {

// Select between average or single sensor temperature control
    
   u8g2.clearBuffer();
   u8g2.setFont(u8g2_font_6x12_me);
   u8g2.setCursor(25, 10);
   u8g2.print("Control Mode:");
    
    if (AvFixSet == 0) {
        // If 0, display "Fixed"
        u8g2.setCursor(15, 35);
        u8g2.print("  Single sensor ");
    } else {
        // If 1, display "Average"
        u8g2.setCursor(10, 35);
        u8g2.print("Average Temperature");
    }
    u8g2.sendBuffer();
}


void SensorDetect() {

  // Detect and connect sensors
     
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_me);
  
  MDNS.begin("Thermostat"); // Initialize mDNS

  int detectedDevices = 0; // Number of detected devices
  unsigned long startTime = millis();
  const unsigned long timeout = 3000; // 6 seconds timeout

  while (true) {
    String deviceName = "esp01-" + String(detectedDevices + 1); // Dynamic device name

    IPAddress deviceIP = MDNS.queryHost(deviceName); // Query IP address
    u8g2.setCursor(3, 10);
    u8g2.print("Searching for sensors...");
    u8g2.sendBuffer();

    if (deviceIP != INADDR_NONE) { // If IP is found
      // Save and display the IP address
      sensorIPs[detectedDevices] = "http://" + String(deviceIP[0]) + "." + String(deviceIP[1]) + "." + String(deviceIP[2]) + "." + String(deviceIP[3]);
      u8g2.setCursor(3, 25);
      u8g2.print(deviceName + " found          ");
      u8g2.sendBuffer();
      delay(1000); // Short delay before next step

      detectedDevices++; // Increase the number of detected devices
      startTime = millis(); // Reset the timer
    } 
    // Exit the loop if the timeout is exceeded
    if (millis() - startTime > timeout) {
      break;
    }
  }

  // Save the number of detected devices to the ConnectDev variable
  ConnectDev = detectedDevices;
  
  // Display summary
  u8g2.clearBuffer();
  u8g2.setCursor(3, 10);
  u8g2.print("Detected devices: " + String(ConnectDev));
  u8g2.sendBuffer();
  delay(2000); 

  // Restart ESP32 if no devices found
  if (ConnectDev == 0) {
    u8g2.clearBuffer();
    u8g2.setCursor(3, 20);
    u8g2.print("No sensors found");
    u8g2.setCursor(3, 35);
    u8g2.print("Restarting...");
    u8g2.sendBuffer();
    delay(1500);
    esp_restart();
  }
}


void TempSetting(){
   // Temperature setting menu
   
   u8g2.clearBuffer();
   u8g2.setFont(u8g2_font_6x12_me);
   u8g2.setCursor(10, 20);
   u8g2.print("Temperature Setting");
   u8g2.setCursor(55, 40);
   u8g2.print(SetTemp, 1);
   u8g2.print("°");
   u8g2.sendBuffer();
}



void SensorData(float Temperature[], float Humidity[], float Pressure[], size_t sensorCount) {
    HTTPClient http;
    float TotalTemp = 0;

    for (size_t i = 1; i < sensorCount && i < sensorIPs.size(); i++) {
        String sensorIP = sensorIPs[i];

        http.begin(sensorIP + "/getData");
        int httpCode = http.GET();

        if (httpCode > 0) {
            String payload = http.getString();
            int firstIndex = payload.indexOf('\n');
            int secondIndex = payload.indexOf('\n', firstIndex + 1);

            if (firstIndex != -1 && secondIndex != -1) {
                String humidityStr = payload.substring(0, firstIndex);
                String tempStr = payload.substring(firstIndex + 1, secondIndex);
                String pressureStr = payload.substring(secondIndex + 1);

                Temperature[i - 1] = tempStr.toFloat();
                Humidity[i - 1] = humidityStr.toFloat();
                Pressure[i - 1] = pressureStr.toFloat();

                TotalTemp += Temperature[i - 1];
            }
        }
        http.end();
    }

    float lastTemperature = Temperature[sensorCount - 1];
    AverageTemp = (TotalTemp + GlobalT) / ConnectDev;
    AverageTempOutdoor = ((TotalTemp + GlobalT) - lastTemperature) / (ConnectDev - 1);
    
}

void updateSensorData() {
    HTTPClient http;

    if (!sensorIPs.empty()) {
        String sensorIP = sensorIPs[0];
        http.begin(sensorIP + "/getData");
        int httpCode = http.GET();

        if (httpCode > 0) {
            String payload = http.getString();
            int firstIndex = payload.indexOf('\n');
            int secondIndex = payload.indexOf('\n', firstIndex + 1);

            if (firstIndex != -1 && secondIndex != -1) {
                String humidityStr = payload.substring(0, firstIndex);
                String tempStr = payload.substring(firstIndex + 1, secondIndex);
                String pressureStr = payload.substring(secondIndex + 1);

                GlobalT = tempStr.toFloat();
                GlobalH = humidityStr.toFloat();
                GlobalP = pressureStr.toFloat();

                //  Blynk data sending of the main sensor values
                Blynk.virtualWrite(VPIN_TEMP, GlobalT);
                Blynk.virtualWrite(VPIN_HUM, GlobalH);
                Blynk.virtualWrite(VPIN_PRESS, GlobalP);
            }
        }
        http.end();
    }

    unsigned long currentTime = millis();
    if (currentTime - lastCheckTime >= 300000) {
        lastCheckTime = currentTime;
        float pressureChange = GlobalP - previousP;
        previousP = GlobalP;

        if (pressureChange < -2) {
            PressCount = 1;
        }
        else if (pressureChange > 2) {
            PressCount = 2;
        }
        else {
            PressCount = 3;
        }
    }
}

void displaySensorData(float Temperature[], float Humidity[], float Pressure[], size_t sensorCount) {
    u8g2.clearBuffer();  // Clear the screen
    u8g2.setFont(u8g2_font_5x7_mf);  // Set the font

    // Display data for each sensor
    for (size_t i = 0; i < sensorCount && i < (ConnectDev-1); i++) {
        u8g2.setCursor(0, 10 + (i) * 12);  // Set position
        u8g2.print("S" + String(i + 2) + ": ");
        u8g2.setCursor(20, 10 + (i) * 12);  // Set position
        u8g2.print(Temperature[i], 1);
        u8g2.print("C ");
        u8g2.print(Humidity[i], 1);
        u8g2.print("% ");
        u8g2.print((int)Pressure[i]);
        u8g2.print(" hPa");
    }

    u8g2.sendBuffer();  // Update the screen
}

void TemperatureRange(){
   // Set temperature range menu
   
   u8g2.clearBuffer();
   u8g2.setFont(u8g2_font_6x12_me);
   u8g2.setCursor(1, 25);
   u8g2.print("Heating Range: ");
   u8g2.print(TempRange, 1);
   u8g2.print("°");
   u8g2.sendBuffer();
}

void NightSet(){
   // Night-time heating temperature setting
     
   u8g2.clearBuffer();
   u8g2.setFont(u8g2_font_6x12_me);
   u8g2.setCursor(15, 20);
   u8g2.print("Night-time Temp");
   u8g2.setCursor(50, 40);
   u8g2.print(NightSetTemp, 1);
   u8g2.print("°");
   u8g2.sendBuffer();
}

void NightTimeMinSet(){
   // Set night-time heating start time
    
   int hoursNT = NightTimeMin / 60;  // Convert minutes to hours
   int minutesNT = fmod(NightTimeMin, 60);  // Remaining minutes
   u8g2.clearBuffer();
   u8g2.setCursor(1, 40);
   snprintf(MintimeStr, sizeof(MintimeStr), "%d:%02d", hoursNT, minutesNT);
   u8g2.print("Night start:");
   u8g2.print(MintimeStr);
   u8g2.print("h ");
   u8g2.sendBuffer();
}

void NightTimeMaxSet(){
   // Set night-time heating stop time
   
   int hoursNT = NightTimeMax / 60;  // Convert minutes to hours
   int minutesNT = fmod(NightTimeMax, 60);  // Remaining minutes
   u8g2.clearBuffer();
   u8g2.setCursor(1, 40);
   snprintf(MaxtimeStr, sizeof(MaxtimeStr), "%d:%02d", hoursNT, minutesNT);
   u8g2.print("Night end:");
   u8g2.print(MaxtimeStr);
   u8g2.print("h ");
   u8g2.sendBuffer();
}

void StartScreen(){
  // Main screen
  
  u8g2.clearBuffer();
   // Check WiFi connection
  if (WiFi.status() == WL_CONNECTED) {
    u8g2.setFont(u8g2_font_open_iconic_www_1x_t); // WiFi connected
    u8g2.drawGlyph(4, 18, 72); // WiFi icon
   
  } else {
    u8g2.setFont(u8g2_font_open_iconic_www_1x_t); // WiFi not connected
    u8g2.drawGlyph(4, 18, 69); // WiFi icon
  }

 // Display pressure values
     
  u8g2.setFont(u8g2_font_unifont_t_weather);
 switch (PressCount) {
  case 1:
    // Rain is approaching
    u8g2.drawGlyph(1, 40, 56);  // Rain icon
    break;
  
  case 2:
    // Good weather expected
    u8g2.drawGlyph(1, 40, 46);  // Sun icon
    break;
  
  case 3:
    // Stable weather
    u8g2.drawGlyph(1, 40, 51);  // Stable weather icon
    break;
}

    // Display current time:
  u8g2.setFont(u8g2_font_6x12_me);
  u8g2.setCursor(28, 8);
  u8g2.print(String (NowHour) + ":" + String(NowMinute)+ "h");
 
    // Display temperature
  u8g2.setFont(u8g2_font_fur30_tn);
  u8g2.setCursor(26, 47);

if (AvFixSet == 0) {
        // If 0, display "Fixed"
        u8g2.print(GlobalT, 1);
    } else {
        // If 1, display "Average"
        u8g2.print(AverageTemp, 1);
    }
  u8g2.setFont(u8g2_font_helvB14_tf);
  u8g2.setCursor(104, 29);
  u8g2.print("°");
  
  // Display humidity
  u8g2.setFont(u8g2_font_6x12_me);
  u8g2.setCursor(28, 64);
  u8g2.print(GlobalH, 1);
  u8g2.print("%  ");
  
   // Display pressure
  u8g2.print((int)GlobalP);
  u8g2.print(" hPa ");
  
  // Display set temperature
  u8g2.setCursor(70, 10);
  u8g2.print("SET");
  u8g2.print(SetTemp, 1);
  u8g2.print("°");
  
  // Display heating state with icon 
  if(relayState == HIGH){
  u8g2.setFont(u8g2_font_unifont_t_76);
  u8g2.drawGlyph(1, 60, 9832); 
  }
  u8g2.sendBuffer();
}
