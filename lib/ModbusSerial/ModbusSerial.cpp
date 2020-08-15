//
// Created by Andis on 14.08.2020.
//


#include "ModbusSerial.h"

const int STATUS_OK = 0;
const int STATUS_NO_RESPONSE = -2;
const int STATUS_CHECKSUM_MISMATCH = -3;
const int STATUS_SLAVE_ADDRESS_NOT_DEFINED = -1;
const int STATUS_STATE_DATA_MISSING = -4;


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
    modbusAdu.address = slave;
}


int ModbusSerial::readHoldingRegisters(const uint16_t startingAddress, const uint16_t quantityOfRegisters,
                                       byte *response_pdu) {
    byte functionCode = 0x03;
    int response_pdu_len = 2 * quantityOfRegisters + 2;
    const byte request_pdu[] = {functionCode, _HI(startingAddress), _LO(startingAddress), _HI(quantityOfRegisters),
                                _LO(quantityOfRegisters)};
    return _request(request_pdu, sizeof(request_pdu), response_pdu, response_pdu_len);
}

int ModbusSerial::readInputRegisters(const uint16_t startingAddress, const uint16_t quantityOfRegisters,
                                     byte *response_pdu) {
    byte functionCode = 0x04;
    int response_pdu_len = 2 * quantityOfRegisters + 2;
    const byte request_pdu[] = {functionCode, _HI(startingAddress), _LO(startingAddress), _HI(quantityOfRegisters),
                                _LO(quantityOfRegisters)};
    return _request(request_pdu, sizeof(request_pdu), response_pdu, response_pdu_len);
}

int ModbusSerial::writeMultipleRegisters(const uint16_t startingAddress, const uint16_t numberOfRegisters,
                                         const byte numberOfDataBytes,
                                         const byte *registerValues, byte *response_pdu) {
    byte functionCode = 0x10; //decimal 16
    int response_pdu_len = 5;
    const byte request_pdu[] = {functionCode, _HI(startingAddress), _LO(startingAddress), _HI(numberOfRegisters),
                                _LO(numberOfRegisters), numberOfDataBytes};
    byte *full_request_pdu = new byte[numberOfDataBytes + 6];
    memcpy(full_request_pdu, request_pdu, 6);
    memcpy(full_request_pdu + 6, registerValues, numberOfDataBytes);
    int result = _request(full_request_pdu, numberOfDataBytes + 6, response_pdu, response_pdu_len);
    delete[] full_request_pdu;
    return result;
}

int ModbusSerial::readDeviceIdentification(const byte objectId, const byte objectLength, byte *response_pdu) {
    byte functionCode = 0x2B;
    int response_pdu_len = 9 + objectLength;
    const byte request_pdu[] = {functionCode, 0x0E, 0x04, objectId};
    bool debug_current = _debug;
    _debug = false; //disable debug for ID read
    int result = _request(request_pdu, sizeof(request_pdu), response_pdu, response_pdu_len);
    _debug = debug_current;
    return result;
}

int ModbusSerial::_request(const byte *request_pdu, const int req_pdu_len, byte *response, const int resp_pdu_len) {
    if (modbusAdu.address == 0) {
        return STATUS_SLAVE_ADDRESS_NOT_DEFINED;
    }
    //REQUEST
    modbusAdu.pdu = request_pdu;
    modbusAdu.pdu_size = req_pdu_len;
    ModbusSerial::setCrc();

    if (_debug) Serial.print("ModbusSerial::REQUEST: ");
    Serial2.write(modbusAdu.address);
    if (_debug) Serial.printf("%.2X ", modbusAdu.address);
    for (int i = 0; i < modbusAdu.pdu_size; i++) {
        Serial2.write(modbusAdu.pdu[i]);
        if (_debug) Serial.printf("%.2X ", modbusAdu.pdu[i]);
    }
    Serial2.write(_HI(modbusAdu.crc));
    if (_debug) Serial.printf("%.2X ", _HI(modbusAdu.crc));
    Serial2.write(_LO(modbusAdu.crc));
    if (_debug) Serial.printf("%.2X ", _LO(modbusAdu.crc));
    if (_debug) Serial.println("");

    //RESPONSE
    long int waited = 0L;
    int resp_adu_len = resp_pdu_len + 3; //add address & crc bytes
    while (Serial2.available() < resp_adu_len) {
        waited++;
        if (waited > 50000L) {
            if (_debug) {
                Serial.print("Last available() value: ");
                Serial.println(Serial2.available());
            }
            return STATUS_NO_RESPONSE;
        }
    }; // wait for response
    byte *buffer = new byte[resp_adu_len];
    if (_debug) Serial.print("ModbusSerial::RESPONSE: ");
    for (int i = 0; i < resp_adu_len; i++) {
        buffer[i] = Serial2.read();
        if (_debug) Serial.printf("%.2X ", buffer[i]);
    }
    if (_debug) Serial.println("");
    if (!ModbusSerial::validateCrc(buffer, resp_adu_len)) {
        delete[] buffer;
        return STATUS_CHECKSUM_MISMATCH;
    }
    memcpy(response, buffer + 1, resp_pdu_len);
    delete[] buffer;
    return STATUS_OK;
}

void ModbusSerial::setCrc() {
    byte *buffer = new byte[1 + modbusAdu.pdu_size];
    buffer[0] = modbusAdu.address;
    memcpy(buffer + 1, modbusAdu.pdu, modbusAdu.pdu_size);
    modbusAdu.crc = crc16(buffer, 1 + modbusAdu.pdu_size);
    delete[] buffer;
}

bool ModbusSerial::validateCrc(const byte *buffer, int adu_len) {
    uint16_t crc = crc16(buffer, adu_len - 2);
    return (crc == _JOIN(buffer[adu_len - 2], buffer[adu_len - 1]));
}

// Compute the MODBUS RTU CRC
uint16_t ModbusSerial::crc16(const byte *buf, const int len) {
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
    return (crc << 8 | crc >> 8);
}

byte _HI(const uint16_t a) { return (((a) >> 8) & 0xFF); }

byte _LO(const uint16_t a) { return ((a) & 0xFF); }

uint16_t _JOIN(const byte byte_hi, const byte byte_lo) { return ((byte_hi << 8) | (byte_lo & 0x00FF)); }

