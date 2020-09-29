
/****************************************************************************

 ****************************************************************************/
//#include <Arduino.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <HX711_ADC.h>
#include <Wire.h>

/******************** Define includes based on platfrom  ********************/
#ifdef ESP8266
#include <ESP8266WiFi.h>
#endif
#ifdef ESP32
#include <Wifi.h>
#define ENC_TYPE_NONE 7
#endif

#define SERIAL_BAUD 115200

/************************* WiFi Access Point *********************************/
#ifdef REMOTE_CLIENT
#define WLAN_SSID "pipBoyA6"
#define WLAN_PASS "jeanlucpicard"
#endif
#ifdef HOME_CLIENT
#define WLAN_SSID ".:[0.o]:."
#define WLAN_PASS "H3l4j3d3b3l4guz1c4"
#endif

#define WIFI_DELAY 500
/* Max SSID octets. */
#define MAX_SSID_LEN 32
/* Wait this much until device gets IP. */
#define MAX_CONNECT_TIME 10000
#define MAX_RECONNECT_TIME 10000
/* SSID that to be stored to connect. */
char ssid[MAX_SSID_LEN] = "";
char passwd[32] = "";

/************************* Adafruit.io Setup *********************************/
#define AIO_SERVER "io.adafruit.com"
#define AIO_SERVERPORT 1883
#define AIO_USERNAME "ketukil"
#define AIO_KEY "08c3bfd47520403b82bbb4f6ba4847db"
#ifdef ESP8266
#ifdef REMOTE_CLIENT
#define AIO_CID "ESP8266_REMOTE_CLEINT"
#endif

#ifdef HOME_CLIENT
#define AIO_CID "ESP8266_HOME_CLIENT"
#endif
#endif
#ifdef ESP32
#define AIO_CID "ESP32_CLIENT"
#endif

/******************************** Global State  ******************************/
#define BMP280_GND_PIN 13
#define BMP280_VCC_PIN 16
#define BMP280_ADDRESS 0x76
Adafruit_BMP280 bmp; // Create an BMP280 class to connect over I2C. Note: Modify Wire.begin(SDA, SCK);

#define HX711_DATA 12
#define HX711_SCK 13
#define LOADCELL_GAIN 128
HX711_ADC LoadCell(HX711_DATA, HX711_SCK); // Create an HX711 class

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_CID, AIO_USERNAME, AIO_KEY);

/****************************** Feeds ***************************************/
// Setup a feed called 'time' for subscribing to current time
Adafruit_MQTT_Subscribe timefeed = Adafruit_MQTT_Subscribe(&mqtt, "time/seconds", MQTT_QOS_1);
#ifdef ESP8266
#ifdef REMOTE_CLIENT
Adafruit_MQTT_Publish battery = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/esp.battery", MQTT_QOS_1);
Adafruit_MQTT_Publish pressure = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/esp.pressure", MQTT_QOS_1);
Adafruit_MQTT_Publish temperature = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/esp.temperature", MQTT_QOS_1);
#endif
#ifdef HOME_CLIENT
Adafruit_MQTT_Publish battery = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/home.battery", MQTT_QOS_1);
Adafruit_MQTT_Publish pressure = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/home.pressure", MQTT_QOS_1);
Adafruit_MQTT_Publish temperature = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/home.temperature", MQTT_QOS_1);
#endif
#endif

#ifdef ESP32
Adafruit_MQTT_Publish beer = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/esp32.beer", MQTT_QOS_1);
Adafruit_MQTT_Publish msg = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/esp32.msg", MQTT_QOS_1);
Adafruit_MQTT_Subscribe button = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/esp32.switch", MQTT_QOS_1);
#endif

/*************************** Sketch Code ************************************/
// Function prototypes
void MQTT_connect(void);
void timeCallback(uint32_t);
void buttonCallback(char *, uint16_t);
void publishData(uint32_t, uint16_t);
void publishBeer(uint32_t, uint16_t);
void scanWifiAndSort(void);

// Software entry pont
void setup()
{
#ifdef ESP8266
    // define pins for PSU to BMP280
    pinMode(BMP280_GND_PIN, OUTPUT);
    pinMode(BMP280_VCC_PIN, OUTPUT);

    // Set coresponding pins to right values
    digitalWrite(BMP280_GND_PIN, LOW);  // GROUND PIN
    digitalWrite(BMP280_VCC_PIN, HIGH); // VCC PIN
#endif
    pinMode(BUILTIN_LED, OUTPUT);
    // Startup delay
    delay(2000);

    Serial.begin(SERIAL_BAUD);

    // Display welcome message
    Serial.println(F("Karlovac Developer Meetup ESP8266 MQTT and BMP280 on I2C example"));

// Start communication with BMP280 sensor over I2C on address 0x76
#ifdef ESP8266
    bmp.begin(0x76);
#endif

#ifdef ESP32
    LoadCell.begin(LOADCELL_GAIN);
    LoadCell.start(2500);          // tare preciscion can be improved by adding a few seconds of stabilising time
    LoadCell.setCalFactor(21.375); // user set calibration factor (float)
#endif
    // Connect to WiFi access point.
    Serial.println("Scanning for networks...");
    while (WiFi.status() != WL_CONNECTED)
    {
        // Clear previous modes.
        WiFi.softAPdisconnect();
        WiFi.disconnect();
        WiFi.mode(WIFI_STA);
        delay(WIFI_DELAY);

        // Scan for networks to find open guy
        scanWifiAndSort();

        // Global ssid param need to be filled to connect.
        if (strlen(ssid) > 0)
        {
            Serial.printf("Connecting to: %s", ssid);

            // No pass for WiFi. We are looking for non-encrypteds.
            if (strlen(passwd) > 0)
            {
                Serial.printf(" [%s]\n", passwd);
                WiFi.begin(ssid, passwd);
            }
            else
                WiFi.begin(ssid);

            /* Wait until WiFi connection but do not exceed MAX_CONNECT_TIME */
            unsigned short try_cnt = 0;
            while (WiFi.status() != WL_CONNECTED && try_cnt < MAX_CONNECT_TIME / WIFI_DELAY)
            {
                Serial.print("-\r");
                delay(WIFI_DELAY / 4); // Do a cute animation
                Serial.print("\\\r");
                delay(WIFI_DELAY / 4);
                Serial.print("|\r");
                delay(WIFI_DELAY / 4);
                Serial.print("/\r");
                delay(WIFI_DELAY / 4);
                try_cnt++;
            }
            if (WiFi.status() == WL_CONNECTED)
            {
                Serial.println("");
                Serial.println("Connection Successful!");
                Serial.println("Your device IP address is: ");
                Serial.println(WiFi.localIP());
            }
            else
            {
                Serial.println("Connection FAILED");
                scanWifiAndSort();
            }
        }
        else
        {
            Serial.println("No open networks available.");
            scanWifiAndSort();
        }
    }

    // Set callback function
    timefeed.setCallback(timeCallback);
    mqtt.subscribe(&timefeed);

#ifdef ESP32
    button.setCallback(buttonCallback);
    mqtt.subscribe(&button);
#endif
}

void loop()
{
    // Ensure the connection to the MQTT server is alive (this will make the first
    // connection and automatically reconnect when disconnected).  See the MQTT_connect
    // function definition further below.
    MQTT_connect();

    // this is our 'wait for incoming subscription packets and callback em' busy subloop
    // try to spend your time here:
    mqtt.processPackets(10000);

    // ping the server to keep the mqtt connection alive
    // NOT required if you are publishing once every KEEPALIVE seconds
    if (!mqtt.ping())
    {
        mqtt.disconnect();
    }
}

void buttonCallback(char *data, uint16_t len)
{
    Serial.printf("State: %c\tlen: %d", &data, len);

    if (strcmp(data, "OFF") == 0)
        digitalWrite(2, LOW);
    if (strcmp(data, "ON") == 0)
        digitalWrite(2, HIGH);
}

void timeCallback(uint32_t current)
{
    const int timeZone = 1; // utc+4 local Croatian time
    int hour, minute, sec;
    uint32_t unixTime = current;

    // Adjust to local time zone
    current += (timeZone * 60 * 60);

    // Calculate current time
    sec = current % 60;
    current /= 60;
    minute = current % 60;
    current /= 60;
    hour = current % 24;

    // Print hours
    Serial.print("[");
    if (hour < 10)
        Serial.print("0");
    Serial.print(hour);

    // Print minutes
    Serial.print(":");
    if (minute < 10)
        Serial.print("0");
    Serial.print(minute);

    // Print seconds
    Serial.print(":");
    if (sec < 10)
        Serial.print("0");
    Serial.print(sec);
    Serial.print("]\t");

#ifdef ESP32

    if (strcmp((char *)button.lastread, "ON") == 0)
        digitalWrite(2, HIGH);
    if (strcmp((char *)button.lastread, "OFF") == 0)
        digitalWrite(2, LOW);
    publishBeer(sec, 12);

#endif

#ifdef ESP8266
    publishData(sec, 20);
#endif
}

#ifdef ESP8266
void publishData(uint32_t unixTime, uint16_t pushInterval)
{
    int bStatus, pStatus, tStatus;
    float voltage, pressure_hPa, temperature_C;

    voltage = (float)analogRead(A0) * (4.892 / 1.1005) / 1024.0; // (3.839/0.8623);
    pressure_hPa = bmp.readPressure() * 0.01;
    temperature_C = bmp.readTemperature();

    // Push values to serial port
    Serial.print("Battery: ");
    Serial.print(voltage, 3);
    Serial.print(" V");

    Serial.print("\tPressure: ");
    Serial.print(pressure_hPa, 2);
    Serial.print(" hPa");

    Serial.print("\tTemperature: ");
    Serial.print(temperature_C, 2);
    Serial.println(" *C");

    if (unixTime % pushInterval == 0)
    {
        bStatus = battery.publish(voltage, 3);
        pStatus = pressure.publish(pressure_hPa, 2);
        tStatus = temperature.publish(temperature_C, 2);
        Serial.print("Push status: ");
        Serial.print(bStatus, HEX);
        Serial.print(", ");
        Serial.print(pStatus, HEX);
        Serial.print(", ");
        Serial.println(tStatus, HEX);
    }
}
#endif

#ifdef ESP32
void publishBeer(uint32_t unixTime, uint16_t pushInterval)
{
    int bStatus, pStatus, tStatus;
    int beerNumber;
    float weight = 0.0;

    for (int i = 0; i < 32; i++)
    {
        LoadCell.update();
        if (i % 8 == 0)
            weight = LoadCell.getData();
        delay(10);
        delay(10);
    }

    beerNumber = (int)(weight / 500.0);

    Serial.print("\tScale: ");
    Serial.println(beerNumber);

    if (unixTime % pushInterval == 0)
    {

        bStatus = beer.publish(weight);
        if (beerNumber <= 0)
            pStatus = msg.publish("WE ARE OUT OF BEER!");
        else
            pStatus = msg.publish("");

        Serial.print("Push status: 0x");
        Serial.print(bStatus, HEX);
        Serial.print(", 0x");
        Serial.println(pStatus, HEX);
    }
}
#endif

void scanWifiAndSort()
{
    memset(ssid, 0, MAX_SSID_LEN);
    memset(passwd, 0, 32);
    int n = WiFi.scanNetworks();
    Serial.println("Scan complete!");
    if (n == 0)
    {
        Serial.println("No networks available.");
    }
    else
    {
        Serial.print(n);
        Serial.println(" networks discovered.");
        int indices[n];
        for (int i = 0; i < n; i++)
        {
            indices[i] = i;
        }
        for (int i = 0; i < n; i++)
        {
            for (int j = i + 1; j < n; j++)
            {
                if (WiFi.RSSI(indices[j]) > WiFi.RSSI(indices[i]))
                {
                    std::swap(indices[i], indices[j]);
                }
            }
        }
        for (int i = 0; i < n; ++i)
        {
            Serial.println("The strongest open network is:");
            Serial.print(WiFi.SSID(indices[i]));
            Serial.print("\tRSSI: ");
            Serial.print(WiFi.RSSI(indices[i]));
            Serial.print("dB\tEnc: ");
            Serial.print(WiFi.encryptionType(indices[i]));
            Serial.println();
            if (WiFi.encryptionType(indices[i]) == ENC_TYPE_NONE)
            {
                memset(ssid, 0, MAX_SSID_LEN);
                strncpy(ssid, WiFi.SSID(indices[i]).c_str(), MAX_SSID_LEN);
                break;
            }
            if (strcmp(WLAN_SSID, WiFi.SSID(indices[i]).c_str()) == 0)
            {
                memset(ssid, 0, MAX_SSID_LEN);
                strncpy(ssid, WiFi.SSID(indices[i]).c_str(), MAX_SSID_LEN);
                memset(passwd, 0, 32);
                strncpy(passwd, WLAN_PASS, 32);
                break;
            }
        }
    }
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect()
{
    int8_t ret;

    // Stop if already connected.
    if (mqtt.connected())
    {
        return;
    }

    Serial.print("Connecting to MQTT... ");

    uint8_t retries = 3;
    while ((ret = mqtt.connect()) != 0)
    {
        // connect will return 0 for connected
        Serial.println(mqtt.connectErrorString(ret));
        Serial.println("Retrying MQTT connection in 10 seconds...");
        mqtt.disconnect();
        delay(10000); // wait 10 seconds
        retries--;
        if (retries == 0)
        {
            // basically die and wait for WDT to reset me
            Serial.println("----- RESTARTING CHIP -----");
            delay(1000);
            ESP.restart();
        }
    }
    Serial.println("MQTT Connected!");
}
