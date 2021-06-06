#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <stdbool.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define DHTTYPE DHT11

// TODO: hook up O2 sensor and enable.

// --------CONSTANTS (won't change)---------------
// Digital pin connected to the DHT sensor
// Pin 15 can work but DHT must be disconnected during program upload.
const int DEBUG_TIME_ACCEL = 1; // divide all times by this to accelerate investigations during dev. Set to 1 when not needed.

const int DHT_PIN = A0; // DHT hum/temp sensor pin
const int ONE_WIRE_PIN = 16; // DS18B20 temp sensor pin
const int SOIL_HUM_PIN = 8;
const int WATER_PUMP_PIN = 3;
const int BLOWER_PIN = 4;
const int RADIATOR_VALVE_PIN = 6;
// const int SHORTEST_PATH_VALVE_PIN 
const int AIR_RENEWAL_VALVE_PIN = 5;


const int SOIL_MOISTURE_SIGNAL_PIN = A1;
const int SOIL_MOISTURE_POWER_PIN = 52;

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

const float CAP_HUM_SENSOR_AIR = 798;   // Output of capacitive moisture sensor in air and water. Values normalized in % in this range.
const float CAP_HUM_SENSOR_WATER = 395; // Water level to above the "v" in "v.1.2". May need to be retested once waterproofed.

// --- Enabled/Disabled sensors and effectors
const bool IS_O2_SENSOR_ENABLED = false;
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
  //byte state;
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
  // struct Effector shortestPathValve; // Opens air system to shortest path
  struct Effector airRenewalValve; // Opens air circulation to ambient atmosphere
};

// Sensors unites all sensors for easy bookkeeping
struct Sensors {
  DHT dht; // Air humidity and temp
  DallasTemperature soilTemp;
  Sensor airO2; // TODO
  Sensor methane; // TODO
};

//------------ HEADERS---------------------
void changeState(unsigned long currentMillis, struct Measurements measurements, struct Effectors *effectors);
void logEffectorStateOnConsole(struct Effectors effectors);
void display(struct Measurements measurements);
void logInfoOnConsole(struct Measurements measurements);
// void turnOnEffectorFor5Sec(struct Effector effector, unsigned long currentMillis);

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
    // .shortestPathValve = {
    //   .pin = SHORTEST_PATH_VALVE_PIN,
    //   .prevState = LOW,
    //   .prevMillis = 0,
    //   .onInterval = BLOWER_ON_INTERVAL + VALVE_BUFFER_INTERVAL,
    //   .offInterval = 0,
    // },
    .airRenewalValve = {
      .pin = AIR_RENEWAL_VALVE_PIN,
      .prevState = LOW,
      .prevMillis = 0,
      .onInterval = BLOWER_ON_INTERVAL,
      .offInterval = BLOWER_OFF_INTERVAL * 4
    }
};
//------------ HEADERS---------------------
void readSensors(unsigned long currentMillis, Sensors sensors, Measurements measurements);

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
  // pinMode(effectors.shortestPathValve.pin, OUTPUT);
  pinMode(effectors.airRenewalValve.pin, OUTPUT);

  // Start all effectors in the off state
  digitalWrite(effectors.waterPump.pin, LOW);
  digitalWrite(effectors.blower.pin, LOW);
  digitalWrite(effectors.radiatorValve.pin, LOW);
  // digitalWrite(effectors.shortestPathValve.pin, LOW);
  digitalWrite(effectors.airRenewalValve.pin, LOW);
}

// Take measurements every 1 min when system is at rest
// Take measurements every 3s when air is circulating
// Show measurements on LCD

bool firstRun = true;
void loop() {
  if (firstRun) {
    digitalWrite(effectors.waterPump.pin, HIGH);
    delay(5000);
  }
  digitalWrite(effectors.waterPump.pin, LOW);
  firstRun = false;

  // put your main code here, to run repeatedly:
  unsigned long currentMillis = millis();
  readSensors(currentMillis, sensors, measurements);
  changeState(currentMillis, measurements, &effectors);
}

void readSensors(unsigned long currentMillis, Sensors s, Measurements m) {
  bool refreshDisplay = false;

  // if (s.soilSensor.isReady(currentMillis)) {
  //   // TODO: calibrate soil humidity sensor
  //   m.soilHumidity = s.soilSensor.getSoilHumidity();
  //   refreshDisplay = true;
  // }

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
      Serial.println(F("Failed to read from DHT sensor!"));
      return;
    }

    // Soil moisture sensor
    digitalWrite(SOIL_MOISTURE_POWER_PIN, HIGH); // Turn the sensor ON

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

void changeState(unsigned long currentMillis, Measurements m, Effectors *e) {
  bool circulateAir = false;

  // --------- Air renewal -------------
  // O2 should NEVER go below minimum.
  e->airRenewalValve.prevState = digitalRead(e->airRenewalValve.pin);
  if (IS_O2_SENSOR_ENABLED) {
    if (m.airO2 < AIR_O2_MIN) {
      // If O2 is low, continuously renew air until the normal level is reached.
      digitalWrite(e->airRenewalValve.pin, HIGH);
      if (e->airRenewalValve.prevState != digitalRead(e->airRenewalValve.pin)) {
        Serial.print(F("O2 low: opening air renewal valve"));
      }
      circulateAir = true;
    } else if (m.airO2 >= AIR_O2_NORM) {
      digitalWrite(e->airRenewalValve.pin, LOW);
      if (e->airRenewalValve.prevState != digitalRead(e->airRenewalValve.pin)) {
        Serial.print(F("O2 normal: closing air renewal valve"));
      }
    }
  } else {
    // Renew air on a set schedule
    if (millis() - e->airRenewalValve.prevMillis >= e->airRenewalValve.offInterval){
      Serial.print(F("Opening air renewal valve on set schedule"));
      e->airRenewalValve.prevMillis = millis();
      digitalWrite(e->airRenewalValve.pin,  HIGH);    
      circulateAir = true;
    } else if (millis() - e->airRenewalValve.prevMillis >= e->airRenewalValve.onInterval && 
        digitalRead(e->airRenewalValve.pin) == HIGH) {
      Serial.print(F("Closing air renewal valve on set schedule"));
      e->airRenewalValve.prevMillis = millis();
      digitalWrite(e->airRenewalValve.pin, HIGH);  
    }
  }

  // --------- Radiator -------------
  // Temperature should NEVER go above maximum.
  e->radiatorValve.prevState = digitalRead(e->radiatorValve.pin);
  if (m.airTemp >= MAX_TEMP_C && digitalRead(e->radiatorValve.pin) == LOW) {
    // Open radiatior path and close direct path.
    digitalWrite(e->radiatorValve.pin, HIGH);
    // digitalWrite(e->shortestPathValve.pin, LOW);
    circulateAir = true;
    if (e->radiatorValve.prevState != digitalRead(e->radiatorValve.pin)) {
      Serial.print(F("Temperature high: opening radiator valve and closing shortest path valve"));
    }
  } else if (m.airTemp < MAX_TEMP_C && digitalRead(e->radiatorValve.pin) == HIGH) {
    digitalWrite(e->radiatorValve.pin, LOW);
    // digitalWrite(e->shortestPathValve.pin, HIGH);
    if (e->radiatorValve.prevState != digitalRead(e->radiatorValve.pin)) {
      Serial.print(F("Temperature in range: closing radiator valve and opening shortest path valve"));
    }
  }
  // --------- Humidity -------------
  e->waterPump.prevState = digitalRead(e->waterPump.pin);
  if (m.soilHumidity >= SOIL_H2O_MAX) {
    digitalWrite(e->waterPump.pin, LOW);
    circulateAir = true;
    if (e->waterPump.prevState != digitalRead(e->waterPump.pin)) {
      Serial.print(F("Soil humidity high: aerating for evaporation"));
    }
  } else if (m.soilHumidity < SOIL_H2O_MIN) {
    digitalWrite(e->waterPump.pin, HIGH);
    circulateAir = true;
    if (e->waterPump.prevState != digitalRead(e->waterPump.pin)) {
      Serial.print(F("Soil humidity low: adding water\n"));
    }
  } else if (m.soilHumidity >= SOIL_H2O_NORM && digitalRead(e->waterPump.pin) == HIGH) {
    digitalWrite(e->waterPump.pin, LOW);
    if (e->waterPump.prevState != digitalRead(e->waterPump.pin)) {
      Serial.print(F("Soil humidity in range: stop water pump"));
    }
  }
  // --------- Circulate air -------------
  e->blower.prevState = digitalRead(e->blower.pin);
  if (circulateAir) {
    // Blow air until above params are in check and do not toggle `circulateAir` to true anymore.
    e->blower.prevMillis = millis();
    digitalWrite(e->blower.pin, HIGH);
    if (e->blower.prevState != digitalRead(e->blower.pin)) {
      // Serial.print(F("Turning blower on to adjust parameters"));
    }
  } else if (millis() - e->blower.prevMillis >= BLOWER_OFF_INTERVAL) {
    e->blower.prevMillis = millis();
    digitalWrite(e->blower.pin, HIGH);
  } else if (millis() - e->blower.prevMillis >= BLOWER_ON_INTERVAL) {
    e->blower.prevMillis = millis();
    digitalWrite(e->blower.pin,  LOW);
  }

  // Log state of effectors if an update occured
  if (e->waterPump.prevState != digitalRead(e->waterPump.pin) ||
      e->blower.prevState != digitalRead(e->blower.pin) ||
      e->radiatorValve.prevState != digitalRead(e->radiatorValve.pin) ||
      // e->shortestPathValve.prevState != digitalRead(e->shortestPathValve.pin) ||
      e->airRenewalValve.prevState != digitalRead(e->airRenewalValve.pin)) {
    logEffectorStateOnConsole(*e);
  }
}

void logEffectorStateOnConsole(Effectors e) {
  Serial.print(F("Water Pump: \t\t  "));
  Serial.print(digitalRead(e.waterPump.pin));
  Serial.print(F("\n"));

  Serial.print(F("Blower: \t\t  "));
  Serial.print(digitalRead(e.blower.pin));
  Serial.print(F("\n"));

  Serial.print(F("Radiator valve: \t  "));
  Serial.print(digitalRead(e.radiatorValve.pin));
  Serial.print(F("\n"));

  // Serial.print(F("Shortest path valve: \t  "));
  // Serial.print(digitalRead(e.shortestPathValve.pin));
  // Serial.print(F("\n"));

  Serial.print(F("Air renewal valve: \t  "));
  Serial.print(digitalRead(e.airRenewalValve.pin));
  Serial.print(F("\n\n"));
}

void display(Measurements measurements) {
  logInfoOnConsole(measurements);
}

void logInfoOnConsole(Measurements m) {

  Serial.print(F("Soil Hum.: "));
  Serial.print(m.soilHumidity);
  Serial.print(F("% "));
  Serial.print(F("Soil Temp.: "));
  Serial.print(m.soilTemp);
  Serial.print(("°C "));
  Serial.print(F("Air Hum.: "));
  Serial.print(m.airHumidity);
  Serial.print(F("% "));
  Serial.print(F("Air Temp.: "));
  Serial.print(m.airTemp);
  Serial.print(F("°C "));
  Serial.print(F(" Heat index: "));
  Serial.print(m.airHeatIndex);
  Serial.println(F("°C "));
}

// void turnOnEffectorFor5Sec(struct Effector e, unsigned long currentMillis) {
//   unsigned long interval = 500000;
//   if (e.state == HIGH && currentMillis - e.prevMillis >= interval) {
//     e.state = LOW;
//   } else
// }