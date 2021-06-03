#include <unity.h>
#include <main.h>

Measurements create_normal_measurements_state() {
  struct Measurements m;
  m = (Measurements) {
    .airHumidity = 30,
      .airTemp = 50,
      .airO2 = 18,
      .soilHumidity = 50,
      .soilTemp = 50
  };
  return m;
}

Effectors create_normal_effector_state() {
  struct Effectors e;
  e = (Effectors) {
    .waterPump = {
        .state = low,
        .prevState = low,
        .prevMillis = 0,
        .onInterval = 1000,
        .offInterval = 0
      },
      .blower = {
        .state = low,
        .prevState = low,
        .prevMillis = 0,
        .onInterval = 0,
        .offInterval = 0
      },
      .radiatorValve = {
        .state = low,
        .prevState = low,
        .prevMillis = 0,
        .onInterval = 0,
        .offInterval = 0
      },
      .shortestPathValve = {
        .state = low,
        .prevState = low,
        .prevMillis = 0,
        .onInterval = 0,
        .offInterval = 0,
      },
      .airRenewalValve = {
        .state = low,
        .prevState = low,
        .prevMillis = 0,
        .onInterval = 0,
        .offInterval = 0
      }
  }; 
  return e;
}

void test_high_temperature() {
  struct Measurements m = create_normal_measurements_state();
  struct Effectors e = create_normal_effector_state();

  // changeState(0, m, &e);
  // logEffectorStateOnConsole(e);
}

int main(int argc, char ** argv) {
  UNITY_BEGIN();

  RUN_TEST(test_high_temperature);

  UNITY_END();
}