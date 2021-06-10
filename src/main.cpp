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
  byte prevState; // Used to log to console only if there's a change
  unsigned long prevMillis;
  unsigned long onInterval;
  unsigned long offInterval;
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
void testAllEffectorOnStartup(struct Effectors effectors);

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
    testAllEffectorOnStartup(effectors);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long currentMillis = millis();
  readSensors(currentMillis, sensors, measurements);

  if (Serial.available()) {
    char ch = Serial.read();
    bool turnOn = false;
    int pin = 0;
    switch (ch){
      case BLOWER_ON_MSG: // Turn on blower
        turnOn = true;
        pin = effectors.blower.pin;
      case BLOWER_OFF_MSG: // Turn off blower
        turnOn = false;
        pin = effectors.blower.pin;
      case RADIATOR_ON_MSG: // Turn on radiator valve
        turnOn = true;
        pin = effectors.radiatorValve.pin;
      case RADIATOR_OFF_MSG: // Turn off radiator valve
        turnOn = false;
        pin = effectors.radiatorValve.pin;
      case AIR_RENEW_ON_MSG: // Turn on air renewal valve
        turnOn = true;
        pin = effectors.airRenewalValve.pin;
      case AIR_RENEW_OFF_MSG: // Turn off air renewal valve
        turnOn = false;
        pin = effectors.airRenewalValve.pin;
      case WATER_PUMP_ON_MSG: // Turn on water pump
        turnOn = true;
        pin = effectors.waterPump.pin;
      case WATER_PUMP_OFF_MSG: // Turn off water pump
        turnOn = false;
        pin = effectors.waterPump.pin;
      default:
        Serial.print("error: message '");
        Serial.print(ch);
        Serial.print("' not supported.");
        break;
    }
    if (turnOn){
      digitalWrite(pin, HIGH);
    } else {
      digitalWrite(pin, LOW);
    }
    Serial.print(ch); // Confirm command was received and acted out
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

// void changeState(unsigned long currentMillis, Measurements m, Effectors *e) {
//   bool circulateAir = false;

//   // --------- Air Renewal --------
//   // O2 should NEVER go below minimum.
//   e->airRenewalValve.prevState = digitalRead(e->airRenewalValve.pin);
//   if (IS_O2_SENSOR_ENABLED) {
//     if (m.airO2 < AIR_O2_MIN) {
//       // If O2 is low, continuously renew air until the normal level is reached.
//       digitalWrite(e->airRenewalValve.pin, HIGH);
//       if (e->airRenewalValve.prevState != digitalRead(e->airRenewalValve.pin)) {
//         Serial.print(F("O2 low: opening air renewal valve\n"));
//       }
//       circulateAir = true;
//     } else if (m.airO2 >= AIR_O2_NORM) {
//       digitalWrite(e->airRenewalValve.pin, LOW);
//       if (e->airRenewalValve.prevState != digitalRead(e->airRenewalValve.pin)) {
//         Serial.print(F("O2 normal: closing air renewal valve\n"));
//       }
//     }
//   } else {
//     // Renew air on a set schedule
//     if (millis() - e->airRenewalValve.prevMillis >= e->airRenewalValve.offInterval + e->airRenewalValve.onInterval && 
//         digitalRead(e->airRenewalValve.pin) == LOW){

//       Serial.print(F("Opening air renewal valve on set schedule\n"));
//       e->airRenewalValve.prevMillis = millis();
//       digitalWrite(e->airRenewalValve.pin,  HIGH);    
//       circulateAir = true;
//     } else if (millis() - e->airRenewalValve.prevMillis >= e->airRenewalValve.onInterval && 
//         digitalRead(e->airRenewalValve.pin) == HIGH) {

//       Serial.print(F("Closing air renewal valve on set schedule\n"));
//       digitalWrite(e->airRenewalValve.pin, LOW);  
//     }
//   }

//   // --------- Radiator -------------
//   // Temperature should NEVER go above maximum.
//   e->radiatorValve.prevState = digitalRead(e->radiatorValve.pin);
//   if (m.airTemp >= MAX_TEMP_C && digitalRead(e->radiatorValve.pin) == LOW) {
//     // Open radiatior path and close direct path.
//     digitalWrite(e->radiatorValve.pin, HIGH);
//     circulateAir = true;
//     if (e->radiatorValve.prevState != digitalRead(e->radiatorValve.pin)) {
//       Serial.print(F("Temperature high: opening radiator valve and closing shortest path valve\n"));
//     }
//   } else if (m.airTemp < MAX_TEMP_C && digitalRead(e->radiatorValve.pin) == HIGH) {
//     digitalWrite(e->radiatorValve.pin, LOW);
//     if (e->radiatorValve.prevState != digitalRead(e->radiatorValve.pin)) {
//       Serial.print(F("Temperature in range: closing radiator valve and opening shortest path valve\n"));
//     }
//   }

//   // --------- Humidity -------------
//   e->waterPump.prevState = digitalRead(e->waterPump.pin);
//   if (m.soilHumidity >= SOIL_H2O_MAX) {
//     digitalWrite(e->waterPump.pin, LOW);
//     circulateAir = true;
//     if (e->waterPump.prevState != digitalRead(e->waterPump.pin)) {
//       Serial.print(F("Soil humidity high: aerating for evaporation\n"));
//     }
//   } else if (m.soilHumidity < SOIL_H2O_MIN) {
//     digitalWrite(e->waterPump.pin, HIGH);
//     circulateAir = true;
//     if (e->waterPump.prevState != digitalRead(e->waterPump.pin)) {
//       Serial.print(F("Soil humidity low: adding water\n"));
//     }
//   } else if (m.soilHumidity >= SOIL_H2O_NORM && digitalRead(e->waterPump.pin) == HIGH) {
//     digitalWrite(e->waterPump.pin, LOW);
//     if (e->waterPump.prevState != digitalRead(e->waterPump.pin)) {
//       Serial.print(F("Soil humidity in range: stop water pump\n"));
//     }
//   }

//   // --------- Circulate air -------------
//   e->blower.prevState = digitalRead(e->blower.pin);
//   if (circulateAir) {
//     e->blower.prevMillis = millis();
//     digitalWrite(e->blower.pin, HIGH);
//     if (e->blower.prevState != digitalRead(e->blower.pin)) {
//       Serial.print(F("Turning blower on to adjust parameters\n"));
//     }
//   } else if (millis() - e->blower.prevMillis >= BLOWER_ON_INTERVAL + BLOWER_OFF_INTERVAL) {
//     // Force blower on at a set interval no matter what parameter updates happened beforehand.
//     e->blower.prevMillis = millis();
//     digitalWrite(e->blower.pin, HIGH);
//   } else if (millis() - e->blower.prevMillis >= BLOWER_ON_INTERVAL && digitalRead(e->blower.pin) == HIGH) {
//     e->blower.prevMillis = millis();
//     digitalWrite(e->blower.pin,  LOW);
//   }

//   // Log state of effectors if an update occured
//   if (e->waterPump.prevState != digitalRead(e->waterPump.pin) ||
//       e->blower.prevState != digitalRead(e->blower.pin) ||
//       e->radiatorValve.prevState != digitalRead(e->radiatorValve.pin) ||
//       e->airRenewalValve.prevState != digitalRead(e->airRenewalValve.pin)) {
//     logEffectorStateOnConsole(*e);
//   }
// }

// void logEffectorStateOnConsole(Effectors e) {
//   Serial.print(F("Water Pump: \t\t  "));
//   Serial.print(digitalRead(e.waterPump.pin));
//   Serial.print(F("\n"));

//   Serial.print(F("Blower: \t\t  "));
//   Serial.print(digitalRead(e.blower.pin));
//   Serial.print(F("\n"));

//   Serial.print(F("Radiator valve: \t  "));
//   Serial.print(digitalRead(e.radiatorValve.pin));
//   Serial.print(F("\n"));

//   Serial.print(F("Air renewal valve: \t  "));
//   Serial.print(digitalRead(e.airRenewalValve.pin));
//   Serial.print(F("\n\n"));
// }

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

void testAllEffectorOnStartup(struct Effectors effectors) {
  unsigned long runtime = 1000;
  unsigned long breakTime = 1000;
  digitalWrite(effectors.blower.pin, HIGH);
  delay(runtime);
  digitalWrite(effectors.blower.pin, LOW);

  delay(breakTime);

  digitalWrite(effectors.airRenewalValve.pin, HIGH);
  delay(runtime);
  digitalWrite(effectors.airRenewalValve.pin, LOW);

  delay(breakTime);

  digitalWrite(effectors.radiatorValve.pin, HIGH);
  delay(runtime);
  digitalWrite(effectors.radiatorValve.pin, LOW);

  delay(breakTime);

  digitalWrite(effectors.waterPump.pin, HIGH);
  delay(runtime);
  digitalWrite(effectors.waterPump.pin, LOW);
}