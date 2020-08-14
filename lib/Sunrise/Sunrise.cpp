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
    byte response[15];
    const byte startingAddress[] = {0x00, 0x00};
    const byte quantityOfRegisters[] = {0x00, 0x05};
    int r = mbs.readInputRegisters(startingAddress, quantityOfRegisters, response);
    if (r == STATUS_OK) {
        int co2 = mbs.make_int(response[9], response[10]);
        _lastTemp = mbs.make_int(response[11], response[12]);
        return co2;
    } else {
        return r;
    }
}

int Sunrise::getCurrentMeasurementMode() {
    byte response[7];
    const byte startingAddress[] = {0x00, 0x0A};
    const byte quantityOfRegisters[] = {0x00, 0x01};
    int r = mbs.readHoldingRegisters(startingAddress, quantityOfRegisters, response);
    if (r == STATUS_OK) {
        int mm = mbs.make_int(response[3], response[4]);
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