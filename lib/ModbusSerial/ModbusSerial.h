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

struct ModbusAdu {
    byte address;
    const byte *pdu;
    int pdu_size;
    uint16_t crc;
};


byte _HI(const uint16_t a);
byte _LO(const uint16_t a);
uint16_t _JOIN(const byte byte_hi, const byte byte_lo);

class ModbusSerial {
public:
    ModbusSerial(byte rxpin, byte txpin, int baud, int slave);

    void setDebug(bool enable);

    int readHoldingRegisters(uint16_t startingAddress, uint16_t quantityOfRegisters, byte *response_pdu);

    int readInputRegisters(uint16_t startingAddress, uint16_t quantityOfRegisters, byte *response_pdu);

    int writeMultipleRegisters(const byte *startingAddress, const byte *numberOfRegister, const byte numberOfDataBytes,
                               const byte *registerValue, byte *response_pdu);

    int readDeviceIdentification(byte *request, byte *response);

private:
    bool _debug = false;
    ModbusAdu modbusAdu;

    int _request(const byte *request, const int req_pdu_len, byte *response, const int resp_pdu_len);

    uint16_t crc16(const byte *buf, const int len);

    void setCrc();

    bool validateCrc(const byte *buffer, int adu_len);

};

#endif //ESP32_PLATFORMIO_MODBUSSERIAL_H


