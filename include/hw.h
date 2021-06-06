#include <stdbool.h>
#include <stdint.h>
#include <DHT.h>

class SoilSensor {
    public:
        SoilSensor(uint8_t sensorPin, uint8_t powerPin, unsigned long intervalMillis);
        bool isReady(unsigned long currentMillis);
        float getSoilHumidity();
    private:
        uint8_t sensorPin;
        uint8_t powerPin;
        bool isPowerOn;
        bool justTookReading;
        unsigned long prevReadMillis;   // Last time it was turned on
        unsigned long intervalReadMillis;  // Interval between reads
        unsigned long intervalWarmupMillis;     // How long to warmup before read
        unsigned long startWarmupMillis;   // If set, when it was turned on to warm up
};