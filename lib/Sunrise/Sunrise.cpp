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
        int mm = _JOIN(response[2], response[3]);
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
    const byte registerValue[] = {0x00, 0x01};
    int r = mbs.writeMultipleRegisters(0x000A, 0x0001, 0x02, registerValue, response);
    return r;
}

int Sunrise::setContinuousMeasurementMode() {
    byte response[8];
    const byte registerValue[] = {0x00, 0x00};
    int r = mbs.writeMultipleRegisters(0x000A, 0x0001, 0x02, registerValue, response);
    return r;
}

int Sunrise::startMeasurement() {
    byte response[8];
    const byte registerValue[] = {0x00, 0x01};
    int r = mbs.writeMultipleRegisters(0x0021, 0x0001, 0x02, registerValue, response);
    return r;
}

int Sunrise::startMeasurementWithStateData(const byte *stateData) {
    byte response[8];
    byte registerValue[26];
    registerValue[0] = 0x00;
    registerValue[1] = 0x01; //START WITH HR34 - set to 1, to initiate meeasurement
    memcpy(registerValue + 2, stateData, 24);
    int notNullState = 0; //it is very important that host do not write "0" to HR36-HR46
    for (int i = 2; i < 26; i++) {
        if (registerValue[i] != 0x00) notNullState++;
    }
    if (notNullState > 5) {
        int r = mbs.writeMultipleRegisters(0x0021, 0x000D, 0x1A, registerValue, response);
        return r;
    } else {
        Serial.println("State data probably missing!!!");
        return STATUS_STATE_DATA_MISSING;
    }
}

int Sunrise::readStateData(byte *stateData) {
    byte response_pdu[26];
    int r = mbs.readHoldingRegisters(0x0022, 0x000C, response_pdu);
    memcpy(stateData, response_pdu + 2, 24);
    return r;
}

void Sunrise::printDeviceIdentification() {
    byte response[16];
    mbs.readDeviceIdentification(0x01, 0x07, response);
    for (int c = 0; c < 7; c++) {
        Serial.printf("%c", response[9 + c]);
    }
    Serial.print(" ");
    mbs.readDeviceIdentification(0x02, 0x04, response);
    for (int c = 0; c < 4; c++) {
        Serial.printf("%c", response[9 + c]);
    }

}

float Sunrise::getLastTemp() {
    return _lastTemp / 100.0;
}