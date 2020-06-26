/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!

 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <stdlib.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define DHTPIN A0
#define DHTTYPE DHT21 // DHT 21 (AM2301)
DHT dht(DHTPIN, DHTTYPE);

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET 4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = {0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C};
void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 50;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 5,
    .dio = {2, 3, LMIC_UNUSED_PIN},
};

// Disables all channels, except for the one defined above, and sets the
// data rate (SF). This only affects uplinks; for downlinks the default
// channels or the configuration from the OTAA Join Accept are used.
//
// Not LoRaWAN compliant; FOR TESTING ONLY!
//
void forceTxSingleChannelDr()
{
    for (int i = 0; i < 9; i++)
    { // For EU; for US use i<71
        if (i != channel)
        {
            LMIC_disableChannel(i);
        }
    }
    // Set data rate (SF) and transmit power for uplink
    LMIC_setDrTxpow(dr, 14);
}

void do_send(osjob_t *j)
{
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    String myString;
    if (isnan(h) || isnan(t))
    {
        myString = "{\"error\":\"No DHT Found\"}";
    }
    else
    {
        myString = "{\"temp\":" + String(t) + ", \"hum\":" + String(h) + "}";
    }
    Serial.print("Sending: " + myString);
    displayText("Sending: \n" + myString);
    String a = myString;                           // string a
    unsigned char *b = (unsigned char *)a.c_str(); // cast from string to unsigned char*

    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND)
    {
        displayText("OP_TXRXPEND, not sending");
        Serial.println(F("OP_TXRXPEND, not sending"));
    }
    else
    {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, b, strlen(b), 0);
        displayText("Packet queued");
        Serial.println(F("Packet queued"));
    }
}

void onEvent(ev_t ev)
{
    Serial.print(os_getTime());
    Serial.print(": ");
    switch (ev)
    {
    case EV_SCAN_TIMEOUT:
        displayText("EV_SCAN_TIMEOUT");
        Serial.println(F("EV_SCAN_TIMEOUT"));
        break;
    case EV_BEACON_FOUND:
        displayText("EV_BEACON_FOUND");
        Serial.println(F("EV_BEACON_FOUND"));
        break;
    case EV_BEACON_MISSED:
        displayText("EV_BEACON_MISSED");
        Serial.println(F("EV_BEACON_MISSED"));
        break;
    case EV_BEACON_TRACKED:
        displayText("EV_BEACON_TRACKED");
        Serial.println(F("EV_BEACON_TRACKED"));
        break;
    case EV_JOINING:
        displayText("EV_JOINING");
        Serial.println(F("EV_JOINING"));
        break;
    case EV_JOINED:
        displayText("EV_JOINED");
        Serial.println(F("EV_JOINED"));
        forceTxSingleChannelDr();
        // Disable link check validation (automatically enabled
        // during join, but not supported by TTN at this time).

        LMIC_setLinkCheckMode(0);
        break;
    case EV_RFU1:
        displayText("EV_RFU1");
        Serial.println(F("EV_RFU1"));
        break;
    case EV_JOIN_FAILED:
        displayText("EV_JOIN_FAILED");
        Serial.println(F("EV_JOIN_FAILED"));
        break;
    case EV_REJOIN_FAILED:
        displayText("EV_REJOIN_FAILED");
        Serial.println(F("EV_REJOIN_FAILED"));
        break;
    case EV_TXCOMPLETE:
        displayText("EV_TXCOMPLETE \n(includes waiting for RX windows)");
        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
        if (LMIC.txrxFlags & TXRX_ACK)
        {
            Serial.println(F("Received ack"));
            displayText("Received ack");
        }
        if (LMIC.dataLen)
        {
            Serial.println(F("Received "));
            Serial.println(LMIC.dataLen);
            Serial.println(F(" bytes of payload"));
            String message = "Received ";
            message += LMIC.dataLen;
            message += "\nbytes of payload";
            displayText(message);
        }
        else
        {
            displayText("Seemed to not\nreceive anything\nback? ");
            Serial.println("Seemed to not receive anything back? ");
        }
        // Schedule next transmission
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
        break;
    case EV_LOST_TSYNC:
        displayText("EV_LOST_TSYNC");
        Serial.println(F("EV_LOST_TSYNC"));
        break;
    case EV_RESET:
        displayText("EV_RESET");
        Serial.println(F("EV_RESET"));
        break;
    case EV_RXCOMPLETE:
        // data received in ping slot
        displayText("EV_RXCOMPLETE");
        Serial.println(F("EV_RXCOMPLETE"));
        break;
    case EV_LINK_DEAD:
        displayText("EV_LINK_DEAD");
        Serial.println(F("EV_LINK_DEAD"));
        break;
    case EV_LINK_ALIVE:
        displayText("EV_LINK_ALIVE");
        Serial.println(F("EV_LINK_ALIVE"));
        break;
    default:
        displayText("Unknown event");
        Serial.println(F("Unknown event"));
        break;
    }
}

void displayText(String text)
{
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    // Display static text
    display.println(text);
    display.display();
    delay(2000);
}

void setup()
{
    Serial.begin(9600);
    Serial.println(F("Starting"));
#ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
#endif

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
    { // Address 0x3C for 128x32
        Serial.println(F("SSD1306 allocation failed"));
        for (;;)
            ; // Don't proceed, loop forever
    }
    displayText("OLed display\nworking");

    dht.begin();
    Serial.println(F("Starting DHT"));
    displayText("Starting DHT");

    // LMIC init
    os_init();
    displayText("LoRa library\nworking");

    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);
    LMIC_setAdrMode(1);
    LMIC_startJoining();

    // LMiC will already have decided to send on one of the 3 default
    // channels; ensure it uses the one we want
    LMIC.txChnl = channel;

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}

void loop()
{
    os_runloop_once();
}
