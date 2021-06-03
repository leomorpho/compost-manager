#ifndef MAIN_H
#define MAIN_H

#include <stdbool.h>

enum state {high, low};
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
  enum state state;
  enum state prevState; // Used to log to console only if there's a change
  unsigned long prevMillis;
  unsigned long onInterval;
  unsigned long offInterval;
};

// Effectors unites all effectors for easy bookkeeping
struct Effectors {
  struct Effector waterPump;
  struct Effector blower;
  struct Effector radiatorValve; // Opens air system to radiator
  struct Effector shortestPathValve; // Opens air system to shortest path
  struct Effector airRenewalValve; // Opens air circulation to ambient atmosphere
};

//------------ HEADERS---------------------
void changeState(unsigned long currentMillis, struct Measurements measurements, struct Effectors *effectors);
void logEffectorStateOnConsole(struct Effectors effectors);
void display(struct Measurements measurements);
void logInfoOnConsole(struct Measurements measurements);

#endif // MAIN_H