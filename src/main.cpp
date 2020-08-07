#include <HardwareSerial.h>
#include <WiFiClient.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <MHZ.h>

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

MHZ co2(RXD2, TXD2, MHZ14A);

const byte requestReading[] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
byte result[9];

void setup_wifi() {
    Serial.print("Starting Wifi connection to ");
    Serial.println(ssid);
    if (WiFi.status() == WL_CONNECTED) return;
    WiFi.mode(WIFI_STA);
//    WiFi.config(ip, gateway, subnet);
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



void setup() {
    //disable brownout reset (low power reset)
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

    Serial.begin(115200);
    setup_wifi();
    client.setServer(mqtt_server, 1883);
    if (!client.connected()) {
        reconnect();
    }
    delay(60);
    sprintf(msg_debug, "ESP32 status: preheating");
    client.publish("DEBUG", msg_debug);

    co2.setDebug(false);
    if (co2.isPreHeating()) {
        Serial.print("Preheating");
        while (co2.isPreHeating()) {
            Serial.print(".");
            delay(5000);
        }
        Serial.println();
    }
//    Serial2.begin(9600,SERIAL_8N1, RXD2, TXD2);
}

int readPPMSerial() {
    for (int i = 0; i < 9; i++) {
        Serial2.write(requestReading[i]);
    }
    //Serial.println("sent request");
    while (Serial2.available() < 9) {}; // wait for response
    for (int i = 0; i < 9; i++) {
        result[i] = Serial2.read();
    }
    int high = result[2];
    int low = result[3];
    //Serial.print(high); Serial.print(" ");Serial.println(low);
    return high * 256 + low;
}


void loop() {


    if (!client.connected()) {
        reconnect();
    }
    delay(60);

    int reading = co2.readCO2UART();
    Serial.print("CO2 ppm: ");
    if (reading > 0) {
        Serial.print(reading);
        Serial.print(", T: ");
        Serial.println(co2.getLastTemperature());
        sprintf(msg, "%i", reading);
        client.publish("CO2", msg);
        sprintf(msg_tmp, "%i", co2.getLastTemperature());
        client.publish("TEMP", msg_tmp);
        long int m = millis() / 1000L;
        sprintf(msg_debug, "Reading OK [%ld]", m);
        client.publish("DEBUG", msg_debug);
    } else {
        Serial.print("UART n/a: ");
        Serial.println(reading);
        //debug message
        sprintf(msg_debug, "UART error: %i", reading);
        client.publish("DEBUG", msg_debug);
    }

    delay(60100L);
}