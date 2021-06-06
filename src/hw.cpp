#include <Arduino.h>
#include <hw.h>
#include <stdbool.h>
#include <stdint.h>

SoilSensor::SoilSensor(uint8_t sPin, uint8_t pPin, unsigned long interval) {
    sensorPin = sPin;
    powerPin = pPin;
    isPowerOn = false;
    justTookReading = false;
    prevReadMillis = 0;
    intervalWarmupMillis = 10000;
    intervalReadMillis = interval - intervalWarmupMillis;
    startWarmupMillis = 0;
}

// isReady takes care of the sensor state 
bool SoilSensor::isReady(unsigned long currentMillis) {
    if (startWarmupMillis == 0) {
        startWarmupMillis = currentMillis; 
        return false;
    }
    if (justTookReading) {
        justTookReading = false;
        isPowerOn = false;
        return false;
    }
    // Warmup period is done, sensor read can now be done
    if (isPowerOn && currentMillis - startWarmupMillis >= intervalWarmupMillis) {
        prevReadMillis = currentMillis; 
        justTookReading = true;
        return true;  
    }
    // Warmup period is not done, keep waiting
    if (isPowerOn && currentMillis - startWarmupMillis < intervalWarmupMillis) {
        return false; 
    }
    // Wait interval is done, start warmup
    if (currentMillis - prevReadMillis >= intervalReadMillis) {
        digitalWrite(sensorPin, HIGH);
        isPowerOn = true;
        return false;
    }
    // Wait interval is not done, keep waiting
    else {
        isPowerOn = false;
        return false;
    }
}

// getSoilHumidity reads the sensor value and returns it
float SoilSensor::getSoilHumidity() {
    int val = analogRead(sensorPin);
    digitalWrite(sensorPin, LOW);
    return val;
}