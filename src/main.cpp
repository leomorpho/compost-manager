#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <stdbool.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define DHTTYPE DHT11

// TODO: hook up O2 sensor and enable.
bool TEST_ALL_EFFECTORS_ON_STARTUP = false;
// --------CONSTANTS (won't change)---------------
// Digital pin connected to the DHT sensor
// Pin 15 can work but DHT must be disconnected during program upload.
const int DEBUG_TIME_ACCEL = 1; // divide all times by this to accelerate investigations during dev. Set to 1 when not needed.

// PINS: sensors
const int DHT_PIN = A0; // DHT hum/temp sensor pin
const int ONE_WIRE_PIN = 16; // DS18B20 temp sensor pin
const int SOIL_HUM_PIN = 8;
// PINS: effectors
const int WATER_PUMP_PIN = 3;
const int BLOWER_PIN = 4;
const int RADIATOR_VALVE_PIN = 6;
const int AIR_RENEWAL_VALVE_PIN = 5;

const int RASPBERRY_PI_PIN = 7;

const int DEFAULT_SENSOR_INTERVAL = 3000;
const int DHT_INTERVAL = DEFAULT_SENSOR_INTERVAL; // Min 3s
const long BLOWER_OFF_INTERVAL = 3600000 / DEBUG_TIME_ACCEL; // Turn blower on every hour
const int BLOWER_ON_INTERVAL = 15000 / DEBUG_TIME_ACCEL; // How long the blower is turned on
const int VALVE_BUFFER_INTERVAL = 5000 / DEBUG_TIME_ACCEL; // How long more than the blower the valve should be open

// --- Sensor Values ---
const float AIR_O2_MIN = 15; // Minimum O2 % of system air
const float AIR_O2_NORM = 18; // Normal O2 % of system air
const float MAX_TEMP_C = 60; // Max temperature allowed during thermophilic composting phase
const float SOIL_H2O_MAX = 60; // Max % humidity in soil
const float SOIL_H2O_NORM = 50; // Normal % humidity in soil
const float SOIL_H2O_MIN = 45; // Minimum % humidity in soil

const float CAP_HUM_SENSOR_AIR = 817;   // Output of capacitive moisture sensor in air and water. Values normalized in % in this range.
const float CAP_HUM_SENSOR_WATER = 395; // Water level to above the "v" in "v.1.2". May need to be retested once waterproofed.

// --- Enabled/Disabled sensors and effectors
const bool IS_O2_SENSOR_ENABLED = false;

// --------------- Serial Messages -------------------------
const char BLOWER_ON_MSG = 'a';
const char BLOWER_OFF_MSG = 'b';
const char RADIATOR_ON_MSG = 'c';
const char RADIATOR_OFF_MSG = 'd';
const char AIR_RENEW_ON_MSG = 'e';
const char AIR_RENEW_OFF_MSG = 'f';
const char WATER_PUMP_ON_MSG = 'g';
const char WATER_PUMP_OFF_MSG = 'h';

const char SENSOR_VALUES_MSG_HEADER = 'i';
const char TEST_ALL_SYSTEM_MSG = 'j';
//========================================

// Initialize connected hardware if required
// Sensor dht = Sensor()
DHT dht(DHT_PIN, DHTTYPE);

// Setup a oneWire instance to communicate with any OneWire device
OneWire oneWire(ONE_WIRE_PIN);
// Pass oneWire reference to DallasTemperature library
DallasTemperature soilTempProbe( & oneWire);

// --------------- Structures -------------------------
struct Measurements {
  float airHumidity;
  float airTemp;
  float airHeatIndex;
  float airO2;
  float airMethane;
  float soilHumidity;
  float soilTemp;
}
measurements;

// Sensor is a non-specific implementation for an sensor
struct Sensor {
  int signalPin;
};

// Effector is a non-specific implementation for an effector
struct Effector {
  int pin;
  byte prevState; // TODO: remove field
  unsigned long prevMillis; // TODO: remove field
  unsigned long onInterval; // TODO: remove field
  unsigned long offInterval; // TODO: remove field
};

// Effectors unites all effectors for easy bookkeeping
struct Effectors {
  struct Effector waterPump;
  struct Effector blower;
  struct Effector radiatorValve; // Opens air system to radiator
  struct Effector airRenewalValve; // Opens air circulation to ambient atmosphere
};

// Sensors unites all sensors for easy bookkeeping
struct Sensors {
  DHT dht; // Air humidity and temp
  DallasTemperature soilTemp;
  Sensor airO2; // TODO
  Sensor methane; // TODO
};

//----------- STRUCT INSTANCES ---------------

Sensors sensors {
  .dht = dht,
  .soilTemp = soilTempProbe,
};

Effectors effectors {
  .waterPump = {
      .pin = WATER_PUMP_PIN,
      .prevState = LOW, 
      .prevMillis = 0,
      .onInterval = 1000,
      .offInterval = 0
    },
    .blower = {
      .pin = BLOWER_PIN,
      .prevState = LOW,
      .prevMillis = 0,
      .onInterval = BLOWER_ON_INTERVAL,
      .offInterval = BLOWER_OFF_INTERVAL
    },
    .radiatorValve = {
      .pin = RADIATOR_VALVE_PIN,
      .prevState = LOW,
      .prevMillis = 0,
      .onInterval = BLOWER_ON_INTERVAL + VALVE_BUFFER_INTERVAL,
      .offInterval = 0
    },
    .airRenewalValve = {
      .pin = AIR_RENEWAL_VALVE_PIN,
      .prevState = LOW,
      .prevMillis = 0,
      .onInterval = BLOWER_ON_INTERVAL,
      .offInterval = BLOWER_OFF_INTERVAL * 4
    }
};
//------------ HEADERS---------------------
//void changeState(unsigned long currentMillis, struct Measurements measurements, struct Effectors *effectors);
void readSensors(unsigned long currentMillis, Sensors sensors, Measurements measurements);
// void logEffectorStateOnConsole(struct Effectors effectors);
void display(struct Measurements measurements);
void logInfoOnConsole(struct Measurements measurements);
void testAllEffector(struct Effectors effectors);

//------------ VARIABLES (will change)---------------------
unsigned long prevSensorsMillis = 0;
unsigned long prevSoilMoistureMillis = 0;

//========================================

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  sensors.dht.begin();
  soilTempProbe.begin();

  // Set all effectors pins as output pins
  pinMode(effectors.waterPump.pin, OUTPUT);
  pinMode(effectors.blower.pin, OUTPUT);
  pinMode(effectors.radiatorValve.pin, OUTPUT);
  pinMode(effectors.airRenewalValve.pin, OUTPUT);
  pinMode(RASPBERRY_PI_PIN, OUTPUT);

  // Start all effectors in the off state
  digitalWrite(effectors.waterPump.pin, LOW);
  digitalWrite(effectors.blower.pin, LOW);
  digitalWrite(effectors.radiatorValve.pin, LOW);
  digitalWrite(effectors.airRenewalValve.pin, LOW);
  //digitalWrite(RASPBERRY_PI_PIN, HIGH);

  if (TEST_ALL_EFFECTORS_ON_STARTUP) {
    testAllEffector(effectors);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long currentMillis = millis();
  readSensors(currentMillis, sensors, measurements);

  if (Serial.available()) {
    char ch = Serial.read();
    bool commandLegal = true;
    bool turnOn = false;
    int pin = 0;
  
    switch (ch){
      case BLOWER_ON_MSG: // Turn on blower
        turnOn = true;
        pin = effectors.blower.pin;
        break;
      case BLOWER_OFF_MSG: // Turn off blower
        turnOn = false;
        pin = effectors.blower.pin;
        break;
      case RADIATOR_ON_MSG: // Turn on radiator valve
        turnOn = true;
        pin = effectors.radiatorValve.pin;
        break;
      case RADIATOR_OFF_MSG: // Turn off radiator valve
        turnOn = false;
        pin = effectors.radiatorValve.pin;
        break;
      case AIR_RENEW_ON_MSG: // Turn on air renewal valve
        turnOn = true;
        pin = effectors.airRenewalValve.pin;
        break;
      case AIR_RENEW_OFF_MSG: // Turn off air renewal valve
        turnOn = false;
        pin = effectors.airRenewalValve.pin;
        break;
      case WATER_PUMP_ON_MSG: // Turn on water pump
        turnOn = true;
        pin = effectors.waterPump.pin;
        break;
      case WATER_PUMP_OFF_MSG: // Turn off water pump
        turnOn = false;
        pin = effectors.waterPump.pin;
        break;
      case TEST_ALL_SYSTEM_MSG:
        testAllEffector(effectors);
        break;
      default:
        Serial.print("error: message '");
        Serial.print(ch);
        Serial.println("' not supported.");
        commandLegal = false;
    }
    if (turnOn){
      digitalWrite(pin, HIGH);
    } else {
      digitalWrite(pin, LOW);
    }
    if (commandLegal){
      Serial.println(ch); // Confirm command was received and acted out
    }
  }
}

void readSensors(unsigned long currentMillis, Sensors s, Measurements m) {
  bool refreshDisplay = false;

  if (currentMillis - prevSensorsMillis >= DEFAULT_SENSOR_INTERVAL) {
    prevSensorsMillis = currentMillis;
    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    m.airHumidity = s.dht.readHumidity();
    // Read temperature as Celsius (the default)
    m.airTemp = s.dht.readTemperature();

    // Send the command to get temperatures
    soilTempProbe.requestTemperatures();
    m.soilTemp = s.soilTemp.getTempCByIndex(0);

    // See this page: https://makersportal.com/blog/2020/5/26/capacitive-soil-moisture-calibration-with-arduino
    m.soilHumidity = 100 - ((analogRead(SOIL_HUM_PIN) - CAP_HUM_SENSOR_WATER) / (CAP_HUM_SENSOR_AIR - CAP_HUM_SENSOR_WATER) * 100);
    // Check if any reads failed and exit early (to try again).
    if (isnan(m.airHumidity) || isnan(m.airTemp)) {
      Serial.println(F("Failed to read from DHT sensor!\n"));
      return;
    }

    // Compute heat index in Celsius (isFahreheit = false)
    m.airHeatIndex = s.dht.computeHeatIndex(
      m.airTemp, m.airHumidity, false);
    refreshDisplay = true;
  }

  // TODO: read O2, methane, and any other sensor connected.

  if (refreshDisplay) {
    display(m);
  }
}

void display(Measurements measurements) {
  logInfoOnConsole(measurements);
  // TODO: log info on LCD
}

void logInfoOnConsole(Measurements m) {
  Serial.print(SENSOR_VALUES_MSG_HEADER);
  Serial.print(F(" sHum: "));
  Serial.print(m.soilHumidity);
  Serial.print(F("% "));
  Serial.print(F("sTemp.: "));
  Serial.print(m.soilTemp);
  Serial.print(("°C "));
  Serial.print(F("airHum: "));
  Serial.print(m.airHumidity);
  Serial.print(F("% "));
  Serial.print(F("airTemp.: "));
  Serial.print(m.airTemp);
  Serial.print(F("°C "));
  Serial.print(F(" heatIndex: "));
  Serial.print(m.airHeatIndex);
  Serial.println(F("°C "));
}

void testAllEffector(struct Effectors effectors) {
  unsigned long runtime = 1500;
  unsigned long breakTime = 1500;
  Serial.print(TEST_ALL_SYSTEM_MSG);
  Serial.println(F(" start testing all effectors"));
  Serial.print(TEST_ALL_SYSTEM_MSG);
  Serial.println(F(" test blower..."));
  digitalWrite(effectors.blower.pin, HIGH);
  delay(runtime);
  digitalWrite(effectors.blower.pin, LOW);

  delay(breakTime);

  Serial.print(TEST_ALL_SYSTEM_MSG);
  Serial.println(F(" test air renewal valve..."));
  digitalWrite(effectors.airRenewalValve.pin, HIGH);
  delay(runtime);
  digitalWrite(effectors.airRenewalValve.pin, LOW);

  delay(breakTime);

  Serial.print(TEST_ALL_SYSTEM_MSG);
  Serial.println(F(" test radiator valve..."));
  digitalWrite(effectors.radiatorValve.pin, HIGH);
  delay(runtime);
  digitalWrite(effectors.radiatorValve.pin, LOW);

  delay(breakTime);

  Serial.print(TEST_ALL_SYSTEM_MSG);
  Serial.println(F(" test water pump..."));
  digitalWrite(effectors.waterPump.pin, HIGH);
  delay(runtime);
  digitalWrite(effectors.waterPump.pin, LOW);

  Serial.print(TEST_ALL_SYSTEM_MSG);
  Serial.println(F(" test water pump and radiator valve..."));
  digitalWrite(effectors.waterPump.pin, HIGH);
  digitalWrite(effectors.radiatorValve.pin, HIGH);
  delay(runtime);
  digitalWrite(effectors.waterPump.pin, LOW);
  digitalWrite(effectors.radiatorValve.pin, LOW);

  Serial.print(TEST_ALL_SYSTEM_MSG);
  Serial.println(F(" finished testing all effectors"));
}