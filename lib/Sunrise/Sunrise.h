//
// Created by Andis on 14.08.2020.
//

#ifndef ESP32_PLATFORMIO_SUNRISE_H
#define ESP32_PLATFORMIO_SUNRISE_H

#include <ModbusSerial.h>

extern const int MEASUREMENT_MODE_SINGLE;
extern const int MEASUREMENT_MODE_CONTINUOUS;

class Sunrise {
public:
    Sunrise(byte rxpin, byte txpin, bool debug);

    int requestReading();

    int getCurrentMeasurementMode();

    int setSingleMeasurementMode();

    int setContinuousMeasurementMode();

    float getLastTemp();

    void printDeviceIdentification();

    int startMeasurement();

    int startMeasurementWithStateData(const byte *stateData);

    int readStateData(byte *stateData);

private:
    bool _debug = false;
    ModbusSerial mbs;
    int _lastTemp = 0;
};


#endif //ESP32_PLATFORMIO_SUNRISE_H
