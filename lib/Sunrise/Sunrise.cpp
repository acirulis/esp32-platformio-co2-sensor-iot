//
// Created by Andis on 14.08.2020.
//

#include "Sunrise.h"

const int MEASUREMENT_MODE_SINGLE = 2;
const int MEASUREMENT_MODE_CONTINUOUS = 1;

Sunrise::Sunrise(byte rxpin, byte txpin, bool debug) : mbs(rxpin, txpin, 9600, 0x68) {
    mbs.setDebug(debug);
}

int Sunrise::requestReading() {
    byte response[12];
    int r = mbs.readInputRegisters(0x0000, 0x0005, response);
    if (r == STATUS_OK) {
        int co2 = _JOIN(response[8], response[9]);
        _lastTemp = _JOIN(response[10], response[11]);
        return co2;
    } else {
        return r;
    }
}

int Sunrise::getCurrentMeasurementMode() {
    byte response[7];
    int r = mbs.readHoldingRegisters(0x000A, 0x0001, response);
    if (r == STATUS_OK) {
        int mm = _JOIN(response[3], response[4]);
        if (mm == 0) {
            return MEASUREMENT_MODE_CONTINUOUS;
        } else {
            return MEASUREMENT_MODE_SINGLE;
        }
    } else {
        return r;
    }
}

int Sunrise::setSingleMeasurementMode() {
    byte response[8];
    const byte startingAddress[] = {0x00, 0x0A};
    const byte numberOfRegister[] = {0x00, 0x01};
    const byte numberOfDataBytes = 0x02;
    const byte registerValue[] = {0x00, 0x01};
    int r = mbs.writeMultipleRegisters(startingAddress, numberOfRegister, numberOfDataBytes, registerValue, response);
    return r;
}

int Sunrise::setContinuousMeasurementMode() {
    byte response[8];
    const byte startingAddress[] = {0x00, 0x0A};
    const byte numberOfRegister[] = {0x00, 0x01};
    const byte numberOfDataBytes = 0x02;
    const byte registerValue[] = {0x00, 0x00};
    int r = mbs.writeMultipleRegisters(startingAddress, numberOfRegister, numberOfDataBytes, registerValue, response);
    return r;
}

float Sunrise::getLastTemp() {
    return _lastTemp / 100.0;
}