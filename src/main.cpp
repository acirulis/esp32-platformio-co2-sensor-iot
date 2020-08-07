#include <HardwareSerial.h>
#include <WiFiClient.h>
#include <WiFi.h>
#include <PubSubClient.h>
//#include <MHZ.h>

#include <soc/soc.h>
#include <soc/rtc_cntl_reg.h>


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
#define byte uint8_t

//MHZ co2(RXD2, TXD2, MHZ14A);



void setup_wifi() {
    if (WiFi.status() == WL_CONNECTED) return;
    Serial.print("Starting Wifi connection to ");
    Serial.println(ssid);
    WiFi.mode(WIFI_STA);
    WiFi.config(ip, gateway, subnet);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}

void reconnect() {
    setup_wifi();
    Serial.println("In MQTT reconnection...");
    client.setKeepAlive(70);
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        if (client.connect("Client_ESP32")) {
            Serial.println("connected");
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 3 seconds");
            delay(3000);
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

const byte requestReading[] = {0x68, 0x04, 0x00, 0x00, 0x00, 0x04, 0xF8, 0xF0};
byte result[13];

int readPPMSerial() {
    for (int i = 0; i < 8; i++) {
        Serial2.write(requestReading[i]);
    }
//    delay(100);
//    Serial.print("Sent request. Response: ");
    long int waited = 0L;
    while (Serial2.available() < 13) {
        waited++;
        if (waited > 100000L) {
            sprintf(msg_debug, "ESP32 status: Could not read UART serial");
            client.publish("DEBUG", msg_debug);
            ESP.restart();
        }
    }; // wait for response
    for (int i = 0; i < 13; i++) {
        result[i] = Serial2.read();
//        Serial.printf("%X", result[i]);
//        Serial.print(" ");
    }
//    Serial.println();
    uint16_t crc = ModRTU_CRC(result, 11);
    int crc_high = result[11];
    int crc_low = result[12];
    if (crc == crc_low * 256 + crc_high) {
        int high = result[9];
        int low = result[10];
        return high * 256 + low;
    } else {
        Serial.print("CRC incorrect. Got: ");
        Serial.print(crc);
        Serial.print(" expected: ");
        Serial.println(crc_low * 256 + crc_high);
        return -1;
    }

}

void setup() {
    //disable brownout reset (low power reset)
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

    Serial.begin(115200);
    setup_wifi();
    client.setServer(mqtt_server, 1883);
    if (!client.connected()) {
        reconnect();
    }
    sprintf(msg_debug, "ESP32 status: Connected");
    client.publish("DEBUG", msg_debug);

    Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
    delay(60);
}

void loop() {

    if (!client.connected()) {
        reconnect();
    }
//    delay(60);
    long int m = millis() / 1000L;
    Serial.print("Starting to read - ");
    Serial.println(m);
    int reading = readPPMSerial();
    if (reading > 0) {
        Serial.print("CO2: ");
        Serial.println(reading);
        sprintf(msg, "%i", reading);
        client.publish("CO2", msg);
        delay(50);
        sprintf(msg_debug, "Reading OK [%ld]", m);
        client.publish("DEBUG", msg_debug);
    } else {
        sprintf(msg_debug, "UART error: %i", reading);
        client.publish("DEBUG", msg_debug);
    }

    esp_sleep_enable_timer_wakeup(60000000L);
    Serial.println("Going to sleep now");
//    delay(1000);
    Serial.flush();
    esp_deep_sleep_start();

}