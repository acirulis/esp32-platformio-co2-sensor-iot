#include <HardwareSerial.h>
#include <WiFiClient.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <soc/soc.h>
#include <soc/rtc_cntl_reg.h>
#include <esp32-hal-cpu.h>
#include "driver/adc.h"
#include <Sunrise.h>
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

typedef uint8_t byte;

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

RTC_DATA_ATTR float minutesSinceFirstBoot = 0.0;
RTC_DATA_ATTR float sunriseABCtimeoutMins = 0.0; //must increment ABC calibration value every hour
RTC_DATA_ATTR bool stateDataUpdated = false;
RTC_DATA_ATTR byte sunriseStateData[24];

WiFiClient espClient;
PubSubClient client(espClient);

#define RXD2 16 //RX2 pin
#define TXD2 17 //TX2 pin
#define SENS_ENABLE 23
#define SENS_RDY 22

Sunrise sunrise(RXD2, TXD2, false);

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
    minutesSinceFirstBoot += minutes + (millis() / 1000 / 60.0);
    sunriseABCtimeoutMins += minutes + (millis() / 1000 / 60.0);
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


int readPPMSerialSingle() {

    // STEP 1 - DRIVE EN HIGH
    digitalWrite(SENS_ENABLE, HIGH);

    // STEP 2 - WAIT FOR 35ms MIN
    delay(35);

    // STEP 3.2 - SENSOR DATA DOES NOT EXIST, START SINGLE MEASUREMENT
    if (stateDataUpdated) {
        //check if ABC timer needs to be increased
        if (sunriseABCtimeoutMins > 60) {
            uint16_t currentABC = _JOIN(sunriseStateData[0], sunriseStateData[1]);
            currentABC++; //increase by one
            sunriseStateData[0] = _HI(currentABC); //and write back
            sunriseStateData[1] = _LO(currentABC);
            sunriseABCtimeoutMins = 0.0; //and reset to 0;
            Serial.print("ABC timer increased to: ");
            Serial.printf("%.2X\n", currentABC);
        }
        //start measurement
        sunrise.startMeasurementWithStateData(sunriseStateData);
    } else {
        //TODO Something wrong!!!!
        sunrise.startMeasurement();
    }
    stateDataUpdated = false;


    // STEP 4 - WAIT RDY LOW or 2 sec
    while (digitalRead(SENS_RDY));

    // STEP 5 - Read IR1-IR5
    int reading = sunrise.requestReading();

    // STEP 6 - read sensor state data;
    if (sunrise.readStateData(sunriseStateData) == STATUS_OK) {
        stateDataUpdated = true;
    } else {
        stateDataUpdated = false;
        //TODO Something wrong!!!
    }

    // STEP 7 - Drive EN pin low
    digitalWrite(SENS_ENABLE, LOW);

    return reading;
}


void setup() {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout reset (low power reset)
    setCpuFrequencyMhz(80); //Set CPU clock to 80MHz fo example
//    int cs = getCpuFrequencyMhz(); //Get CPU clock

    Serial.begin(115200);

    pinMode(SENS_RDY, INPUT);
    pinMode(SENS_ENABLE, OUTPUT);
    digitalWrite(SENS_ENABLE, LOW);

//    Serial.println("Setting mode to single after 10 sec");
//    delay(10000);
//    sunrise.setSingleMeasurementMode();
//    Serial.println("Waiting for 10 sec");
//    delay(10000);
    if (minutesSinceFirstBoot < 1) {
        digitalWrite(SENS_ENABLE, HIGH);
        delay(35);
        int mm = sunrise.getCurrentMeasurementMode();
        Serial.print("Measurement mode: ");
        if (mm == MEASUREMENT_MODE_SINGLE) {
            Serial.println("SINGLE");
        } else {
            Serial.println("CONTINUOUS");
        }
        Serial.print("Sensor registered: ");
        sunrise.printDeviceIdentification();
        Serial.println("");
        // READ STATE DATA FOR FUTURE USE!
        if (sunrise.readStateData(sunriseStateData) == STATUS_OK) {
            stateDataUpdated = true;
        }
        digitalWrite(SENS_ENABLE, LOW);
    }
    Serial.println("Setup function finished");
}


void loop() {


    Serial.print("Starting to read (min_since_start): ");
    Serial.println(minutesSinceFirstBoot);
    Serial.print("State: ");
    for (int i = 0; i < 24; i++) {
        Serial.printf("%.2X ", sunriseStateData[i]);
    }
    Serial.println("");
    int reading = readPPMSerialSingle();
    if (reading > 0) {
        Serial.print("CO2: ");
        Serial.println(reading);
        Serial.print("Temp: ");
        Serial.println(sunrise.getLastTemp());
        if (!client.connected()) {
            reconnect();
        }
        sprintf(msg, "%i", reading);
        client.publish("CO2", msg);
        sprintf(msg_debug, "Reading OK [%4.2f]", minutesSinceFirstBoot);
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