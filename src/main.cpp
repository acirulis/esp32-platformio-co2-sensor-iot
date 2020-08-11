#include <HardwareSerial.h>
#include <WiFiClient.h>
#include <WiFi.h>
#include <PubSubClient.h>
//#include <MHZ.h>

#include <soc/soc.h>
#include <soc/rtc_cntl_reg.h>
#include <esp32-hal-cpu.h>
#include "driver/adc.h"
#include <esp_wifi.h>
#include <esp_bt.h>


#ifndef STASSID
//#define STASSID "WhiteDigital"
//#define STAPSK  "WhiteDigital2015"
//#define STASSID "ALHN-84DB"
//#define STAPSK  "4914040374"
#define STASSID "MikroTik-Andis2"
#define STAPSK  "viensdivitris"
#endif

const char *ssid = STASSID;
const char *password = STAPSK;
//const char *mqtt_server = "192.168.88.22";
const char *mqtt_server = "192.168.88.242";
IPAddress ip(192, 168, 88, 177);
IPAddress gateway(192, 168, 88, 1);
IPAddress subnet(255, 255, 255, 0);

char msg[10];
char msg_debug[10];
char msg_tmp[10];

WiFiClient espClient;
PubSubClient client(espClient);

#define RXD2 16 //RXX2 pin
#define TXD2 17 //TX2 pin
#define SENS_ENABLE 23
#define SENS_RDY 22
#define byte uint8_t

//MHZ co2(RXD2, TXD2, MHZ14A);

void goToDeepSleep(int minutes) {
    // DEEP SLEEP ROUTINE START
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    btStop();
    adc_power_off();
    esp_wifi_stop();
    esp_bt_controller_disable();
    esp_sleep_enable_timer_wakeup(minutes * 60L * 1000000L);
    Serial.print("Going to sleep now for minutes: ");
    Serial.println(minutes);
    Serial.flush();
    esp_deep_sleep_start();
}

void setup_wifi() {
    if (WiFi.status() == WL_CONNECTED) return;
    Serial.print("Starting Wifi connection to ");
    Serial.println(ssid);
    WiFi.mode(WIFI_STA);
    WiFi.config(ip, gateway, subnet);
    WiFi.begin(ssid, password);
    int failed_counter = 0;
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
        failed_counter++;
        if (failed_counter > 5) {
            Serial.println();
            Serial.println("Wifi connection failed");
            goToDeepSleep(2);
        }
    }
    Serial.println();
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}

void reconnect() {
    setup_wifi();
    Serial.print("MQTT connect...");
    client.setServer(mqtt_server, 1883);
    client.setKeepAlive(70);
    int failed_counter = 0;
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        if (client.connect("Client_ESP32")) {
            Serial.println("connected");
        } else {
            Serial.print("failed, rc=");
            Serial.println(client.state());
            Serial.println("Try again in 500ms");
            delay(500);
            failed_counter++;
            if (failed_counter > 4) {
                Serial.println("MQTT connection failed");
                goToDeepSleep(2);
            }
        }
    }
}


// Compute the MODBUS RTU CRC
uint16_t ModRTU_CRC(byte *buf, int len) {
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


void getCurrentMeasurementMode() {
    char received;
    const byte getMMode[] = {0x68, 0x03, 0x00, 0x0A, 0x00, 0x01, 0xAD, 0x31};
    int len = sizeof(getMMode) / sizeof(getMMode[0]);
    Serial.println("Enabling sensor and sending getCurrentMeasurementMode command");
    digitalWrite(SENS_ENABLE, HIGH);
    delay(50);
    for (int i = 0; i < len; i++) {
        Serial2.write(getMMode[i]);
    }
    delay(50);
    Serial.println("Response: ");
    while (Serial2.available()) {
        received = Serial2.read();
        Serial.printf("%.2X", received);
        Serial.print(" ");
    }
    Serial.println("Done");
    digitalWrite(SENS_ENABLE, LOW);
}

void setSingleMeasurementMode() {
    char received;
    const byte setSingleMode[] = {0x68, 0x10, 0x00, 0x0A, 0x00, 0x01, 0x02, 0x00, 0x01, 0xA5, 0x68};
//  const byte setContinuosMode[] = {0x68, 0x10, 0x00, 0x0A, 0x00, 0x01, 0x02, 0x00, 0x00, 0x64, 0xA8};
    int len = sizeof(setSingleMode) / sizeof(setSingleMode[0]);
    Serial.println("Enabling sensor and sending Reading Mode command");
    digitalWrite(SENS_ENABLE, HIGH);
    delay(50);
    for (int i = 0; i < len; i++) {
        Serial2.write(setSingleMode[i]);
    }
    delay(50);
    Serial.println("Response: ");
    while (Serial2.available()) {
        received = Serial2.read();
        Serial.printf("%.2X", received);
        Serial.print(" ");
    }
    Serial.println("Done");
    digitalWrite(SENS_ENABLE, LOW);
}

int readPPMSerialContinuous() {
//    const byte requestReading[] = {0x68, 0x04, 0x00, 0x00, 0x00, 0x04, 0xF8, 0xF0};
    const byte requestReading[] = {0x68, 0x04, 0x00, 0x00, 0x00, 0x05, 0x39, 0x30}; //with temp
    int len = sizeof(requestReading) / sizeof(requestReading[0]);
    byte result[15];
    for (int i = 0; i < len; i++) {
        Serial2.write(requestReading[i]);
    }
    long int waited = 0L;
    while (Serial2.available() < 15) {
        waited++;
        if (waited > 100000L) {
            sprintf(msg_debug, "ESP32 status: Could not read UART serial");
            client.publish("DEBUG", msg_debug);
//            ESP.restart();
            Serial.println("Timout while reading serial");
            return -2;
        }
    }; // wait for response
    Serial.print("Resp: ");
    for (int i = 0; i < 15; i++) {
        result[i] = Serial2.read();
        Serial.printf("%.2X", result[i]);
        Serial.print(" ");
    }
    uint16_t crc = ModRTU_CRC(result, 11);
    int crc_high = result[13];
    int crc_low = result[14];
    if (crc == crc_low * 256 + crc_high) {
        int low = result[10];
        int high = result[9];
        //TODO temp is in 12,11 (0x091A == 2330 == 23.30 C)
        return high * 256 + low;
    } else {
        Serial.print("CRC incorrect. Got: ");
        Serial.print(crc);
        Serial.print(" expected: ");
        Serial.println(crc_low * 256 + crc_high);
        return -1;
    }
}

int readPPMSerialSingle() {
//    const byte requestReading[] = {0x68, 0x04, 0x00, 0x00, 0x00, 0x04, 0xF8, 0xF0};
    const byte requestReading[] = {0x68, 0x04, 0x00, 0x00, 0x00, 0x05, 0x39, 0x30}; //with temp
    int len = sizeof(requestReading) / sizeof(requestReading[0]);
    const byte startMeasurement[] = {0x68, 0x10, 0x00, 0x21, 0x00, 0x01, 0x02, 0x00, 0x01, 0xA3, 0x73};
    int len_sm = sizeof(startMeasurement) / sizeof(startMeasurement[0]);
    byte result[15];
    char received;
//    int measurement_result;

    // STEP 1 - DRIVE EN HIGH
    digitalWrite(SENS_ENABLE, HIGH);

    // STEP 2 - WAIT FOR 35ms MIN
    delay(35);

    // STEP 3.2 - SENSOR DATA DOES NOT EXIST, START SINGLE MEASUREMENT
    Serial.println("Sending StartMeasurement command");
    for (int i = 0; i < len_sm; i++) {
        Serial2.write(startMeasurement[i]);
    }
    Serial.print("Response: ");
    while (Serial2.available() < 8) {}
    for (int i = 0; i < 8; i++) {
        received = Serial2.read();
        Serial.printf("%.2X", received);
        Serial.print(" ");
    }
    Serial.println("");

    // STEP 4 - WAIT RDY LOW or 2 sec
    while(digitalRead(SENS_RDY));

    // STEP 5 - Read IR1-IR5
    Serial.println("Step5: requestReading");
    for (int i = 0; i < len; i++) {
        Serial2.write(requestReading[i]);
    }
    long int waited = 0L;
    while (Serial2.available() < 15) {
        waited++;
        if (waited > 100000L) {
            Serial.println("Timout while reading serial");
            digitalWrite(SENS_ENABLE, LOW);
            return -2;
        }
    }; // wait for response
    Serial.print("Response : ");
    for (int i = 0; i < 15; i++) {
        result[i] = Serial2.read();
        Serial.printf("%.2X", result[i]);
        Serial.print(" ");
    }
    Serial.println("");
    uint16_t crc = ModRTU_CRC(result, 13);
    int crc_high = result[13];
    int crc_low = result[14];
    if (crc == crc_low * 256 + crc_high) {
        int low = result[10];
        int high = result[9];
        //TODO temp is in 12,11 (0x091A == 2330 == 23.30 C)
        digitalWrite(SENS_ENABLE, LOW);
        return high * 256 + low;
        //TODO Step 6. read sensor state data and store in RTC
        //TODO Step 7. Drive EN pin low
    } else {
        Serial.print("CRC incorrect. Got: ");
        Serial.print(crc);
        Serial.print(" expected: ");
        Serial.println(crc_low * 256 + crc_high);
        digitalWrite(SENS_ENABLE, LOW);
        return -1;
    }
}

void setup() {
    //disable brownout reset (low power reset)
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
    setCpuFrequencyMhz(80); //Set CPU clock to 80MHz fo example
//    int cs = getCpuFrequencyMhz(); //Get CPU clock

    Serial.begin(115200);
//    setup_wifi();
//    client.setServer(mqtt_server, 1883);
//    if (!client.connected()) {
//        reconnect();
//    }
//    sprintf(msg_debug, "ESP32 status: Connected");
//    client.publish("DEBUG", msg_debug);
    Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
    Serial.println("Setup finished");
    pinMode(SENS_RDY, INPUT);
    pinMode(SENS_ENABLE, OUTPUT);
    digitalWrite(SENS_ENABLE, LOW);


}

void loop() {
    long int m = millis() / 1000L;
    Serial.print("Starting to read (ms) - ");
    Serial.println(m);
    int reading = readPPMSerialSingle();
    if (reading > 0) {
        Serial.print("CO2: ");
        Serial.println(reading);
        if (!client.connected()) {
            reconnect();
        }
        sprintf(msg, "%i", reading);
        client.publish("CO2", msg);
        delay(50);
        sprintf(msg_debug, "Reading OK [%ld]", m);
        client.publish("DEBUG", msg_debug);
    } else {
        if (!client.connected()) {
            reconnect();
        }
        Serial.print("Reading result: ");
        Serial.println(reading);
        sprintf(msg_debug, "UART error: %i", reading);
        client.publish("DEBUG", msg_debug);
    }

    goToDeepSleep(2);
//    Serial.println("10 sec delay");
//    delay(10000);

}