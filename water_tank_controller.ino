#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// =====================================================
//                    PIN CONFIGURATION
// =====================================================

// OHT Ultrasonic
#define TRIG_OHT  5
#define ECHO_OHT  18

// Sump Ultrasonic
#define TRIG_SUMP 19
#define ECHO_SUMP 27

// Relay
#define RELAY_PIN 26

// Buzzer
#define BUZZER_PIN 25

// LCD I2C
#define SDA_PIN 21
#define SCL_PIN 22

// =====================================================
//                    LCD CONFIGURATION
// =====================================================

LiquidCrystal_I2C lcd(0x27, 16, 2);

// =====================================================
//                    WIFI CONFIGURATION
// =====================================================

const char* WIFI_SSID = "ABCD";
const char* WIFI_PASSWORD = "aniket05";

const char* SERVER_URL =
    "http://10.246.41.16:3000/api/esp32/telemetry";

const char* DEVICE_ID = "ESP32_WATER_CONTROLLER_01";

// =====================================================
//                    SYSTEM SETTINGS
// =====================================================

// Sensor-to-full-water distance
#define FULL_DISTANCE_CM 5.0

// Calibration time
#define CALIBRATION_TIME 1000

// Minimum OHT level / minimum allowed target
#define OHT_START_LEVEL 25.0

// Default OHT stop level
#define OHT_STOP_LEVEL 90.0

// Sump minimum safe level
#define SUMP_DRY_LEVEL 15.0

// Sump level required for automatic starting
#define SUMP_START_LEVEL 20.0

// Data send interval
#define BACKEND_INTERVAL 2000

// Sensor reading interval
#define SENSOR_INTERVAL 300

// WiFi reconnect interval
#define WIFI_RECONNECT_INTERVAL 10000

// =====================================================
//                    RELAY CONFIGURATION
// =====================================================

#define RELAY_ACTIVE_LOW true

// =====================================================
//                    VARIABLES
// =====================================================

float ohtEmptyDistance = 0.0;
float sumpEmptyDistance = 0.0;

float ohtDistance = 0.0;
float sumpDistance = 0.0;

float ohtPercent = 0.0;
float sumpPercent = 0.0;

bool calibrationDone = false;

bool motorRunning = false;

bool dryRunAlarm = false;

bool wifiConnected = false;

// AUTO or MANUAL
String controlMode = "AUTO";

// Backend manual command
bool manualPumpCommand = false;

// Dynamic stop level
// Defaults to 90% if no custom limit is received
float dynamicStopLevel = 90.0;

// =====================================================
//                    TIMERS
// =====================================================

unsigned long calibrationStartTime = 0;

unsigned long lastSensorRead = 0;

unsigned long lastBackendSend = 0;

unsigned long lastWiFiReconnect = 0;

unsigned long motorStartTime = 0;

// Total motor runtime since ESP32 boot
unsigned long totalMotorRuntimeMs = 0;

// =====================================================
//                    CALIBRATION
// =====================================================

float ohtCalibrationSum = 0.0;
float sumpCalibrationSum = 0.0;

int ohtCalibrationCount = 0;
int sumpCalibrationCount = 0;

// =====================================================
//                    RELAY ON
// =====================================================

void relayON()
{
  if (RELAY_ACTIVE_LOW)
  {
    digitalWrite(RELAY_PIN, LOW);
  }
  else
  {
    digitalWrite(RELAY_PIN, HIGH);
  }

  if (!motorRunning)
  {
    motorRunning = true;
    motorStartTime = millis();
  }
}

// =====================================================
//                    RELAY OFF
// =====================================================

void relayOFF()
{
  if (RELAY_ACTIVE_LOW)
  {
    digitalWrite(RELAY_PIN, HIGH);
  }
  else
  {
    digitalWrite(RELAY_PIN, LOW);
  }

  if (motorRunning)
  {
    totalMotorRuntimeMs += millis() - motorStartTime;
    motorRunning = false;
  }
}

// =====================================================
//                  MOTOR RUNTIME
// =====================================================

unsigned long getMotorRuntimeMs()
{
  if (motorRunning)
  {
    return totalMotorRuntimeMs + (millis() - motorStartTime);
  }

  return totalMotorRuntimeMs;
}

float getMotorRuntimeHours()
{
  return getMotorRuntimeMs() / 3600000.0;
}

// =====================================================
//                 ULTRASONIC SENSOR
// =====================================================

float readDistance(int trigPin, int echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 25000);

  if (duration == 0)
  {
    return -1;
  }

  float distance = duration * 0.0343 / 2.0;

  if (distance < 2 || distance > 400)
  {
    return -1;
  }

  return distance;
}

// =====================================================
//              WATER PERCENTAGE CALCULATION
// =====================================================

float calculatePercentage(float emptyDistance, float currentDistance)
{
  if (emptyDistance <= FULL_DISTANCE_CM)
  {
    return 0;
  }

  float totalHeight = emptyDistance - FULL_DISTANCE_CM;

  float currentWaterHeight = emptyDistance - currentDistance;

  float percentage =
    (currentWaterHeight / totalHeight) * 100.0;

  if (percentage < 0)
  {
    percentage = 0;
  }

  if (percentage > 100)
  {
    percentage = 100;
  }

  return percentage;
}

// =====================================================
//                 READ BOTH SENSORS
// =====================================================

void readTankLevels()
{
  float newOHT = readDistance(TRIG_OHT, ECHO_OHT);

  delay(10);

  float newSump = readDistance(TRIG_SUMP, ECHO_SUMP);

  if (newOHT > 0)
  {
    ohtDistance = newOHT;

    ohtPercent =
      calculatePercentage(ohtEmptyDistance, ohtDistance);
  }

  if (newSump > 0)
  {
    sumpDistance = newSump;

    sumpPercent =
      calculatePercentage(sumpEmptyDistance, sumpDistance);
  }

  // Serial output
  Serial.print("OHT Distance: ");
  Serial.print(ohtDistance);
  Serial.print(" cm | ");

  Serial.print("OHT Level: ");
  Serial.print(ohtPercent);
  Serial.print("% | ");

  Serial.print("Sump Distance: ");
  Serial.print(sumpDistance);
  Serial.print(" cm | ");

  Serial.print("Sump Level: ");
  Serial.print(sumpPercent);
  Serial.println("%");
}

// =====================================================
//                    CALIBRATION
// =====================================================

void calibrateTanks()
{
  unsigned long elapsed =
    millis() - calibrationStartTime;

  // Read OHT
  float ohtReading =
    readDistance(TRIG_OHT, ECHO_OHT);

  delay(10);

  // Read sump
  float sumpReading =
    readDistance(TRIG_SUMP, ECHO_SUMP);

  // Save valid readings
  if (ohtReading > 0)
  {
    ohtCalibrationSum += ohtReading;
    ohtCalibrationCount++;
  }

  if (sumpReading > 0)
  {
    sumpCalibrationSum += sumpReading;
    sumpCalibrationCount++;
  }

  // LCD
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("CALIBRATING");

  lcd.setCursor(0, 1);

  unsigned long remainingMs =
    (CALIBRATION_TIME > elapsed)
    ? (CALIBRATION_TIME - elapsed)
    : 0;

  int remainingSeconds =
    (remainingMs + 999) / 1000;

  lcd.print("Empty Tank ");
  lcd.print(remainingSeconds);
  lcd.print("s");

  Serial.print("Calibration time: ");
  Serial.print(elapsed);
  Serial.println(" ms");

  delay(100);

  // =================================================
  // CALIBRATION FINISHED
  // =================================================

  if (elapsed >= CALIBRATION_TIME)
  {
    if (ohtCalibrationCount > 0)
    {
      ohtEmptyDistance =
        ohtCalibrationSum /
        ohtCalibrationCount;
    }

    if (sumpCalibrationCount > 0)
    {
      sumpEmptyDistance =
        sumpCalibrationSum /
        sumpCalibrationCount;
    }

    calibrationDone = true;

    relayOFF();

    Serial.println();
    Serial.println("=================================");
    Serial.println("CALIBRATION COMPLETE");
    Serial.println("=================================");

    Serial.print("OHT Empty Distance = ");
    Serial.print(ohtEmptyDistance);
    Serial.println(" cm");

    Serial.print("Sump Empty Distance = ");
    Serial.print(sumpEmptyDistance);
    Serial.println(" cm");

    Serial.println("=================================");

    lcd.clear();

    lcd.setCursor(0, 0);
    lcd.print("CALIBRATION");

    lcd.setCursor(0, 1);
    lcd.print("COMPLETE");

    delay(1500);
  }
}

// =====================================================
//          LOCAL MOTOR CONTROL - DYNAMIC TARGET
// =====================================================

void automaticMotorControl()
{
  // =================================================
  // CRITICAL DRY RUN PROTECTION
  // =================================================

  if (sumpPercent < SUMP_DRY_LEVEL)
  {
    relayOFF();

    dryRunAlarm = true;

    digitalWrite(BUZZER_PIN, HIGH);

    Serial.println("!!! DRY RUN PROTECTION !!!");

    return;
  }

  dryRunAlarm = false;

  digitalWrite(BUZZER_PIN, LOW);

  // =================================================
  // DYNAMIC AUTO START CONDITION
  // =================================================
  //
  // IMPORTANT:
  // The motor now starts whenever the current OHT
  // level is below the CURRENT TARGET.
  //
  // Example:
  //
  // Current OHT = 40%
  // New target  = 60%
  //
  // 40 < 60
  // Therefore motor starts again.
  //
  // =================================================

  if (!motorRunning)
  {
    if (ohtPercent < dynamicStopLevel &&
        sumpPercent > SUMP_START_LEVEL)
    {
      relayON();

      Serial.print("AUTO: MOTOR ON - TARGET ");
      Serial.print(dynamicStopLevel);
      Serial.println("%");
    }
  }

  // =================================================
  // DYNAMIC STOP CONDITION
  // =================================================

  if (motorRunning &&
      ohtPercent >= dynamicStopLevel)
  {
    relayOFF();

    Serial.print("AUTO: TARGET LIMIT REACHED (");
    Serial.print(dynamicStopLevel);
    Serial.println("%) - MOTOR OFF");
  }
}

// =====================================================
//                  MANUAL MOTOR CONTROL
// =====================================================

void manualMotorControl()
{
  // Safety interlock always has priority
  if (sumpPercent < SUMP_DRY_LEVEL)
  {
    relayOFF();

    dryRunAlarm = true;

    digitalWrite(BUZZER_PIN, HIGH);

    Serial.println("MANUAL BLOCKED: SUMP DRY");

    return;
  }

  dryRunAlarm = false;

  digitalWrite(BUZZER_PIN, LOW);

  // Overflow safety also remains active
  if (ohtPercent >= OHT_STOP_LEVEL)
  {
    relayOFF();

    Serial.println("MANUAL BLOCKED: OHT 90%");

    return;
  }

  // Backend command
  if (manualPumpCommand)
  {
    relayON();
  }
  else
  {
    relayOFF();
  }
}

// =====================================================
//                 MOTOR CONTROL DECISION
// =====================================================

void controlMotor()
{
  if (controlMode == "MANUAL")
  {
    manualMotorControl();
  }
  else
  {
    automaticMotorControl();
  }
}

// =====================================================
//                    LCD DISPLAY
// =====================================================

void updateLCD()
{
  lcd.clear();

  // -------------------------------
  // OHT
  // -------------------------------

  lcd.setCursor(0, 0);

  lcd.print("O:");
  lcd.print((int)ohtPercent);
  lcd.print("% ");

  if (ohtPercent >= OHT_STOP_LEVEL)
  {
    lcd.print("FULL");
  }
  else if (ohtPercent < OHT_START_LEVEL)
  {
    lcd.print("REFILL");
  }
  else
  {
    lcd.print("OK");
  }

  // -------------------------------
  // SUMP
  // -------------------------------

  lcd.setCursor(0, 1);

  lcd.print("S:");
  lcd.print((int)sumpPercent);
  lcd.print("% ");

  if (sumpPercent < SUMP_DRY_LEVEL)
  {
    lcd.print("DRY");
  }
  else if (sumpPercent <= SUMP_START_LEVEL)
  {
    lcd.print("LOW");
  }
  else
  {
    lcd.print("OK");
  }
}

// =====================================================
//                 CONNECT TO WIFI
// =====================================================

void connectWiFi()
{
  Serial.println();
  Serial.println("Connecting to WiFi...");

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long startAttempt = millis();

  while (WiFi.status() != WL_CONNECTED &&
         millis() - startAttempt < 10000)
  {
    delay(500);

    Serial.print(".");
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED)
  {
    wifiConnected = true;

    Serial.println("WiFi connected");

    Serial.print("ESP32 IP: ");
    Serial.println(WiFi.localIP());
  }
  else
  {
    wifiConnected = false;

    Serial.println("WiFi connection failed");
    Serial.println("System will continue locally");
  }
}

// =====================================================
//                 SEND DATA TO BACKEND
// =====================================================

void sendDataToBackend()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    wifiConnected = false;
    return;
  }

  wifiConnected = true;

  HTTPClient http;

  http.begin(SERVER_URL);

  http.addHeader(
    "Content-Type",
    "application/json"
  );

  // -------------------------------------------------
  // JSON DATA
  // -------------------------------------------------

  String json = "{";

  json += "\"device_id\":\"";
  json += DEVICE_ID;
  json += "\",";

  json += "\"oht_distance_cm\":";
  json += String(ohtDistance, 2);
  json += ",";

  json += "\"oht_percent\":";
  json += String(ohtPercent, 2);
  json += ",";

  json += "\"sump_distance_cm\":";
  json += String(sumpDistance, 2);
  json += ",";

  json += "\"sump_percent\":";
  json += String(sumpPercent, 2);
  json += ",";

  json += "\"motor_running\":";
  json += motorRunning ? "true" : "false";
  json += ",";

  json += "\"motor_runtime_hours\":";
  json += String(getMotorRuntimeHours(), 4);
  json += ",";

  json += "\"control_mode\":\"";
  json += controlMode;
  json += "\",";

  json += "\"dry_run_alarm\":";
  json += dryRunAlarm ? "true" : "false";
  json += ",";

  json += "\"wifi_connected\":";
  json += wifiConnected ? "true" : "false";
  json += ",";

  json += "\"uptime_ms\":";
  json += String(millis());

  json += "}";

  // -------------------------------------------------
  // SEND POST REQUEST
  // -------------------------------------------------

  int httpCode = http.POST(json);

  Serial.println();
  Serial.println("----- BACKEND DATA -----");

  Serial.println(json);

  Serial.print("HTTP Code: ");
  Serial.println(httpCode);

  // -------------------------------------------------
  // READ BACKEND COMMAND
  // -------------------------------------------------

  if (httpCode > 0)
  {
    String response = http.getString();

    Serial.print("Backend Response: ");
    Serial.println(response);

    // -----------------------------------------------
    // MODE
    // -----------------------------------------------

    if (response.indexOf("\"mode\":\"MANUAL\"") >= 0)
    {
      controlMode = "MANUAL";
    }
    else if (response.indexOf("\"mode\":\"AUTO\"") >= 0)
    {
      controlMode = "AUTO";
    }

    // -----------------------------------------------
    // MANUAL PUMP
    // -----------------------------------------------

    if (response.indexOf("\"manual_pump\":true") >= 0)
    {
      manualPumpCommand = true;
    }
    else if (response.indexOf("\"manual_pump\":false") >= 0)
    {
      manualPumpCommand = false;
    }

    // -----------------------------------------------
    // DYNAMIC TARGET OHT
    // -----------------------------------------------

    // Backend JSON:
    //
    // {
    //   "mode":"AUTO",
    //   "manual_pump":false,
    //   "target_oht":65.0
    // }

    int targetIndex =
      response.indexOf("\"target_oht\":");

    if (targetIndex >= 0)
    {
      int valStart =
        targetIndex + 13;

      int valEnd =
        response.indexOf("}", valStart);

      if (valEnd < 0)
      {
        valEnd =
          response.indexOf(",", valStart);
      }

      if (valEnd > valStart)
      {
        float newTarget =
          response.substring(
            valStart,
            valEnd
          ).toFloat();

        // Minimum target = 25%
        // Maximum target = 100%

        if (newTarget >= 25.0 &&
            newTarget <= 100.0)
        {
          dynamicStopLevel = newTarget;

          Serial.print("NEW DYNAMIC TARGET: ");
          Serial.print(dynamicStopLevel);
          Serial.println("%");
        }
      }
    }
  }

  http.end();
}

// =====================================================
//                 SERIAL STATUS
// =====================================================

void printSystemStatus()
{
  Serial.println();
  Serial.println("====================================");

  Serial.print("OHT Level       : ");
  Serial.print(ohtPercent);
  Serial.println("%");

  Serial.print("Sump Level      : ");
  Serial.print(sumpPercent);
  Serial.println("%");

  Serial.print("Motor           : ");

  if (motorRunning)
    Serial.println("ON");
  else
    Serial.println("OFF");

  Serial.print("Control Mode    : ");
  Serial.println(controlMode);

  Serial.print("Target Level    : ");
  Serial.print(dynamicStopLevel);
  Serial.println("%");

  Serial.print("Runtime Hours   : ");
  Serial.println(getMotorRuntimeHours(), 4);

  Serial.print("Dry Run Alarm   : ");

  if (dryRunAlarm)
    Serial.println("YES");
  else
    Serial.println("NO");

  Serial.print("WiFi            : ");

  if (wifiConnected)
    Serial.println("CONNECTED");
  else
    Serial.println("DISCONNECTED");

  Serial.println("====================================");
}

// =====================================================
//                       SETUP
// =====================================================

void setup()
{
  Serial.begin(115200);

  // =================================================
  // PIN MODES
  // =================================================

  pinMode(TRIG_OHT, OUTPUT);
  pinMode(ECHO_OHT, INPUT);

  pinMode(TRIG_SUMP, OUTPUT);
  pinMode(ECHO_SUMP, INPUT);

  pinMode(RELAY_PIN, OUTPUT);

  pinMode(BUZZER_PIN, OUTPUT);

  // =================================================
  // SAFE START
  // =================================================

  relayOFF();

  digitalWrite(BUZZER_PIN, LOW);

  // =================================================
  // LCD
  // =================================================

  Wire.begin(SDA_PIN, SCL_PIN);

  lcd.init();
  lcd.backlight();

  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("Water Tank");

  lcd.setCursor(0, 1);
  lcd.print("Controller");

  delay(1500);

  // =================================================
  // WIFI
  // =================================================

  connectWiFi();

  delay(1000);

  // =================================================
  // START CALIBRATION
  // =================================================

  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("Empty Tanks");

  lcd.setCursor(0, 1);
  lcd.print("Calibrating");

  delay(1000);

  calibrationStartTime = millis();

  Serial.println();
  Serial.println("====================================");
  Serial.println("5 SECOND TANK CALIBRATION STARTED");
  Serial.println("KEEP BOTH TANKS EMPTY");
  Serial.println("====================================");
}

// =====================================================
//                       LOOP
// =====================================================

void loop()
{
  // =================================================
  // CALIBRATION
  // =================================================

  if (!calibrationDone)
  {
    calibrateTanks();

    return;
  }

  // =================================================
  // SENSOR READING
  // =================================================

  if (millis() - lastSensorRead >= SENSOR_INTERVAL)
  {
    lastSensorRead = millis();

    readTankLevels();

    // =================================================
    // MOTOR CONTROL
    // =================================================

    controlMotor();

    // =================================================
    // LCD
    // =================================================

    updateLCD();

    // =================================================
    // SERIAL MONITOR
    // =================================================

    printSystemStatus();
  }

  // =================================================
  // SEND DATA TO BACKEND
  // =================================================

  if (millis() - lastBackendSend >= BACKEND_INTERVAL)
  {
    lastBackendSend = millis();

    sendDataToBackend();
  }

  // =================================================
  // WIFI RECONNECT
  // =================================================

  if (WiFi.status() != WL_CONNECTED)
  {
    wifiConnected = false;

    if (millis() - lastWiFiReconnect >=
        WIFI_RECONNECT_INTERVAL)
    {
      lastWiFiReconnect = millis();

      connectWiFi();
    }
  }
}