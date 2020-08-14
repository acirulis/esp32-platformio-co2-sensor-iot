//
// Basic Modbus serial implemenation for use by Specific IOT sensor (SenseAir Sunraise)
// Using HardwareSerial (Serial2 UART pins)
// By andis.cirulis@gmail.com (c)2020
//

#ifndef ESP32_PLATFORMIO_MODBUSSERIAL_H
#define ESP32_PLATFORMIO_MODBUSSERIAL_H

#include <HardwareSerial.h>

typedef uint8_t byte;

extern const int STATUS_OK;
extern const int STATUS_NO_RESPONSE;
extern const int STATUS_CHECKSUM_MISMATCH;
extern const int STATUS_SLAVE_ADDRESS_NOT_DEFINED;

class ModbusSerial {
public:
    ModbusSerial(byte rxpin, byte txpin, int baud, int slave);

    void setDebug(bool enable);

    int readHoldingRegisters(const byte *startingAddress, const byte *quantityOfRegisters,
                             byte *response_pdu);

    int readInputRegisters(const byte *startingAddress, const byte *quantityOfRegisters,
                           byte *response_pdu);

    int writeMultipleRegisters(const byte *startingAddress, const byte *numberOfRegister, const byte numberOfDataBytes,
                               const byte *registerValue, byte *response_pdu);

    int readDeviceIdentification(byte *request, byte *response);

    int make_int(const byte byte_hi, const byte byte_lo);

private:
    bool _debug = false;
    byte _slave = 0x00; //not set
    int _request(const byte *request, const int req_size, byte *response, const int resp_size);

    uint16_t calculateCRC(const byte *buf, const int len);
};

#endif //ESP32_PLATFORMIO_MODBUSSERIAL_H


