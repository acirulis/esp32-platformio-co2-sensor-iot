//
// Created by Andis on 14.08.2020.
//


#include "ModbusSerial.h"
#include <string.h>

const int STATUS_OK = 0;
const int STATUS_NO_RESPONSE = -2;
const int STATUS_CHECKSUM_MISMATCH = -3;
const int STATUS_SLAVE_ADDRESS_NOT_DEFINED = -1;

/**
 * Enables or disables the debug mode (more logging).
 */
void ModbusSerial::setDebug(bool enable) {
    _debug = enable;
    if (_debug) {
        Serial.println(F("ModbusSerial: debug mode ENABLED"));
    } else {
        Serial.println(F("ModbusSerial: debug mode DISABLED"));
    }
}

/*
 * Constructor
 */
ModbusSerial::ModbusSerial(byte rxpin, byte txpin, int baud, int slave) {
    Serial2.begin(baud, SERIAL_8N1, rxpin, txpin);
    _slave = slave;
}


int ModbusSerial::readHoldingRegisters(const byte *startingAddress, const byte *quantityOfRegisters,
                                       byte *response_pdu) {
    byte functionCode = 0x03;
    int responseLength = 2 * make_int(quantityOfRegisters[0], quantityOfRegisters[1]) + 5;
    const byte request_pdu[] = {functionCode, startingAddress[0], startingAddress[1], quantityOfRegisters[0],
                                quantityOfRegisters[1]};
    return _request(request_pdu, 5, response_pdu, responseLength);
}

int ModbusSerial::readInputRegisters(const byte *startingAddress, const byte *quantityOfRegisters,
                                     byte *response_pdu) {
    byte functionCode = 0x04;
    int responseLength = 2 * make_int(quantityOfRegisters[0], quantityOfRegisters[1]) + 5;
    const byte request_pdu[] = {functionCode, startingAddress[0], startingAddress[1], quantityOfRegisters[0],
                                quantityOfRegisters[1]};
    return _request(request_pdu, 5, response_pdu, responseLength);
}

int ModbusSerial::writeMultipleRegisters(const byte *startingAddress, const byte *numberOfRegister, const byte numberOfDataBytes,
                                         const byte *registerValue, byte *response_pdu) {
    byte functionCode = 0x10; //decimal 16
    int responseLength = 8;
    const byte request_pdu[] = {functionCode, startingAddress[0], startingAddress[1], numberOfRegister[0],
                                numberOfRegister[1], numberOfDataBytes};
    byte full_request_pdu[numberOfDataBytes+6];
    memcpy(full_request_pdu, request_pdu,6);
    memcpy(full_request_pdu + 6, registerValue, numberOfDataBytes);
    return _request(full_request_pdu, numberOfDataBytes+6, response_pdu, responseLength);
}

int ModbusSerial::readDeviceIdentification(byte *request, byte *response) {
//    byte functionCode = 0x2B;
//    byte functionCod2 = 0x0E;
    return STATUS_OK;
}

int ModbusSerial::_request(const byte *request, const int req_size, byte *response, const int resp_size) {
    if (_slave == 0) {
        return STATUS_SLAVE_ADDRESS_NOT_DEFINED;
    }
    //REQUEST
    if (_debug) Serial.print("ModbusSerial::_request REQUEST: ");
    byte *full_request = new byte[1 + req_size];
    memcpy(full_request, &_slave, 1);
    memcpy(full_request + 1, request, req_size);
    uint16_t crc = calculateCRC(full_request, req_size + 1);
    byte crc_lo = crc / 256;
    byte crc_hi = crc % 256;
    for (int i = 0; i < req_size + 1; i++) {
        Serial2.write(full_request[i]);
        if (_debug) Serial.printf("%.2X ", full_request[i]);
    }
    Serial2.write(crc_hi);
    if (_debug) Serial.printf("%.2X ", crc_hi);
    Serial2.write(crc_lo);
    if (_debug) Serial.printf("%.2X ", crc_lo);
    if (_debug) Serial.println("");
    delete[] full_request;

    //RESPONSE
    long int waited = 0L;
    while (Serial2.available() < resp_size) {
        waited++;
        if (waited > 100000L) {
            return STATUS_NO_RESPONSE;
        }
    }; // wait for response
    if (_debug) Serial.print("ModbusSerial::_request RESPONSE: ");
    for (int i = 0; i < resp_size; i++) {
        response[i] = Serial2.read();
        if (_debug) Serial.printf("%.2X ", response[i]);
    }
    if (_debug) Serial.println("");
    crc = calculateCRC(response, resp_size - 2);
    crc_hi = response[resp_size - 2];
    crc_lo = response[resp_size - 1];
    if (crc != crc_lo * 256 + crc_hi) {
        return STATUS_CHECKSUM_MISMATCH;
    }
    return STATUS_OK;
}

// Compute the MODBUS RTU CRC
uint16_t ModbusSerial::calculateCRC(const byte *buf, const int len) {
    uint16_t crc = 0xFFFF;

    for (int pos = 0; pos < len; pos++) {
        crc ^= (uint16_t) buf[pos];          // XOR byte into least sig. byte of crc

        for (int i = 8; i != 0; i--) {    // Loop over each bit
            if ((crc & 0x0001) != 0) {      // If the LSB is set
                crc >>= 1;                    // Shift right and XOR 0xA001
                crc ^= 0xA001;
            } else                            // Else LSB is not set
                crc >>= 1;                    // Just shift right
        }
    }
    // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
    return crc;
}

int ModbusSerial::make_int(const byte byte_hi, const byte byte_lo) {
    return ((byte_hi << 8) | (byte_lo & 0x00FF));
}

