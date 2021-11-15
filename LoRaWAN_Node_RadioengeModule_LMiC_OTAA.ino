/* 
 *   
 *  Project:          IoT Energy Meter with C/C++, Java/Spring, TypeScript/Angular and Dart/Flutter;
 *  About:            End-to-end implementation of a LoRaWAN network for monitoring electrical quantities;
 *  Version:          1.0;
 *  Backend Mote:     ATmega328P/ESP32/ESP8266/ESP8285/STM32;
 *  Radios:           RFM95w and LoRaWAN EndDevice Radioenge Module: RD49C;
 *  Sensors:          Peacefair PZEM-004T 3.0 Version TTL-RTU kWh Meter;
 *  Backend API:      Java with Framework: Spring Boot;
 *  LoRaWAN Stack - Customized for the Frequency Plan adopted in Brazil for LoRaWAN --> [AU915]:
 *  MCCI Arduino LoRaWAN Library (LMiC: LoRaWAN-MAC-in-C) version 3.0.99;
 *  
 *  Or:
 *  LoRaWAN Stack - Implementation approved by the National Telecommunications Agency of Brazil
 *  and inserted by the company Radioenge:
 *  LoRaMac-node Library (LoRaWAN L2 1.0.3 - Released / API via AT commands);
 *  
 *  Activation mode:  Activation by Personalization (ABP) or Over-the-Air Activation (OTAA);
 *  Author:           Adail dos Santos Silva
 *  E-mail:           adail101@hotmail.com
 *  WhatsApp:         +55 89 9 9433-7661
 *  
 *  This project was conceived using the MCCI Arduino LoRaWAN Library (LMiC: LoRaWAN-MAC-in-C) version 3.0.99,
 *  This RD49C module in turn has the copyright belonging to the manufacturer, the company Radioenge do Brasil.
 *  All remaining implementation is authored by the creator of the LoRaWAN Electricity Meter project.
 *  
 *  WARNINGS:
 *  Permission is hereby granted, free of charge, to any person obtaining a copy of
 *  this software and associated documentation files (the “Software”), to deal in
 *  the Software without restriction, including without limitation the rights to
 *  use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 *  the Software, and to permit persons to whom the Software is furnished to do so,
 *  subject to the following conditions:
 *  
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *  
 *  THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 *  FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 *  COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 *  IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 *  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *  
 */


/*
 *  Radioenge Module - LMiC (LoRaWAN-MAC-in-C) Example
 */
 
/********************************************************************
 _____              __ _                       _   _             
/  __ \            / _(_)                     | | (_)            
| /  \/ ___  _ __ | |_ _  __ _ _   _ _ __ __ _| |_ _  ___  _ __  
| |    / _ \| '_ \|  _| |/ _` | | | | '__/ _` | __| |/ _ \| '_ \ 
| \__/\ (_) | | | | | | | (_| | |_| | | | (_| | |_| | (_) | | | |
 \____/\___/|_| |_|_| |_|\__, |\__,_|_|  \__,_|\__|_|\___/|_| |_|
                          __/ |                                  
                         |___/                                   
********************************************************************/

/* Includes. */
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

/* Definitions. */
//#define

// Pinout - IOs:
#define RX_1                    PB7
#define TX_1                    PB6

// Pinout - GPIOs:
#define GPIO0                   PA3
#define GPIO1                   PA2
#define GPIO2                   PA15
#define GPIO3                   PA10
#define GPIO4                   PA9
#define GPIO5                   PA8
#define GPIO6                   PB15
#define GPIO7                   PA0
#define GPIO8                   PA1
#define GPIO9                   PB11

// Pinout - LEDs:
#define LED_RED                 PB5
#define LED_GREEN               PB8
#define LED_YELLOW              PB9

// Pinout - Radio LoRa:
#define RADIO_RESET_PORT        PB14
#define RADIO_MOSI_PORT         PA7
#define RADIO_MISO_PORT         PA6
#define RADIO_SCLK_PORT         PA5
#define RADIO_NSS_PORT          PA4

#define RADIO_DIO_0_PORT        PB10
#define RADIO_DIO_1_PORT        PB2
#define RADIO_DIO_2_PORT        PB1
#define RADIO_DIO_3_PORT        PB0

#define RADIO_RXTX1_PORT        PB13
#define RADIO_RXTX2_PORT        PB12


/* Credentials */
// CHIRPSTACK - CS (8 at 15 + 65 channels):
/* little-endian - LSB */ // 00 00 00 00 00 00 00 00
/* big-endian - MSB */    // 00 00 00 00 00 00 00 00
static const u1_t PROGMEM APPEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/* little-endian - LSB */ // f3 8d f5 e5 a3 1e 63 de (Usar esta chave no cadastro)
/* big-endian - MSB */    // de 63 1e a3 e5 f5 8d f3
static const u1_t PROGMEM DEVEUI[8] = {0xde, 0x63, 0x1e, 0xa3, 0xe5, 0xf5, 0x8d, 0xf3};

/* little-endian - LSB */ // ae 94 4e 5e 85 b7 bc 02 c5 dd 46 da 4c 7c c5 6f
/* big-endian - MSB */    // 6f c5 7c 4c da 46 dd c5 02 bc b7 85 5e 4e 94 ae (Usar esta chave no Cadastro)
static const u1_t PROGMEM APPKEY[16] = {0x6f, 0xc5, 0x7c, 0x4c, 0xda, 0x46, 0xdd, 0xc5, 0x02, 0xbc, 0xb7, 0x85, 0x5e, 0x4e, 0x94, 0xae};

void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }
void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }
void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }


/* Data to Send. */
static uint8_t mydata[] = "AdailSilva";

/* Instances. */
/* Jobs: */
static osjob_t sendjob;

/* Schedule TX every this many seconds (might become longer due to duty cycle limitations). */
const unsigned TX_INTERVAL = 10;

/* Pin mapping. */
const lmic_pinmap lmic_pins = {
  .nss = RADIO_NSS_PORT,
  .rxtx = RADIO_RXTX1_PORT,
  .rst = RADIO_RESET_PORT,
  .dio = {RADIO_DIO_0_PORT, RADIO_DIO_1_PORT, RADIO_DIO_2_PORT},
  .rxtx_rx_active = 1,
};

/*****************************
 _____           _      
/  __ \         | |     
| /  \/ ___   __| | ___ 
| |    / _ \ / _` |/ _ \
| \__/\ (_) | (_| |  __/
 \____/\___/ \__,_|\___|
*****************************/

/* Functions. */

/* Log Functions */
void printHex2(unsigned key)
{
    key &= 0xff;

    if (key < 16)
    {
        Serial.print('0');
    }
    
    Serial.print(key, HEX);
}

void showNetworkInformations()
{
    u4_t netid = 0;
    devaddr_t devaddr = 0;
    u1_t nwkKey[16];
    u1_t artKey[16];
    LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);

//    Serial.println(F("_____________________________________________________________________________"));
    Serial.println();
    Serial.println(" [INFO] NetID   (MSB)       : " + String(netid, HEX));
    Serial.println(" [INFO] DevAddr (MSB)       : " + String(devaddr, HEX));

    Serial.print(" [INFO] NwkSKey (MSB)       : "); // Not used (F());
    
    for (size_t i = 0; i < sizeof(nwkKey); ++i)
    {
        if (i != 0)
            Serial.print("-");
        printHex2(nwkKey[i]);
    }

    Serial.println("");        
    Serial.print(" [INFO] AppSKey (MSB)       : ");

    for (size_t i = 0; i < sizeof(artKey); ++i)
    {
        if (i != 0)
            Serial.print("-");
        printHex2(artKey[i]);
    }
    
    Serial.println();
//    Serial.println(F("_____________________________________________________________________________"));
    Serial.println();
}

void onEvent (ev_t ev) {
//    Serial.print(os_getTime());
//    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            
            showNetworkInformations();

//            LMIC_selectSubBand(1);

            /* Channels Control to AU915 (8 at 15 + 65 channels): */
            for (u1_t b = 0; b < 8; ++b)
            {
                LMIC_disableSubBand(b);
            }
        
            for (u1_t channel = 0; channel < 72; ++channel)
            {
                LMIC_disableChannel(channel);
            }
        
            /* ChirpStack AU915 */
            LMIC_enableChannel(8);
            LMIC_enableChannel(9);
            LMIC_enableChannel(10);
            LMIC_enableChannel(11);
            LMIC_enableChannel(12);
            LMIC_enableChannel(13);
            LMIC_enableChannel(14);
            LMIC_enableChannel(15);
//            LMIC_enableChannel(65); /* Test */
            
            /* Disable Adaptive Data Rate. */
            LMIC_setAdrMode(0);
            
            /* Disable link check validation. */
            LMIC_setLinkCheckMode(0);

            /* TTN uses SF9 for its RX2 window. */
            LMIC.dn2Dr = DR_SF9;

            /* Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library). */
            LMIC_setDrTxpow(DR_SF7,14);

            /* 
             *  Use with Arduino Pro Mini ATmega328P 3.3V 8MHz;
             *  Let LMIC compensate for +/- 1% clock error.
             */
            LMIC_setClockError (MAX_CLOCK_ERROR * 10 / 100);
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack;"));
            if (LMIC.dataLen) {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" byte(s) of payload."));
            }
            /* Schedule next transmission. */
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            /* Data received in ping slot. */
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j){
    /* Check if there is not a current TX/RX job running. */
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        /* Prepare upstream data transmission at the next possible time. */
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        Serial.println(F("Packet queued."));
    }
    /* Next TX is scheduled after TX_COMPLETE event. */
}

/*****************************
 _____      _               
/  ___|    | |              
\ `--.  ___| |_ _   _ _ __  
 `--. \/ _ \ __| | | | '_ \ 
/\__/ /  __/ |_| |_| | |_) |
\____/ \___|\__|\__,_| .__/ 
                     | |    
                     |_|    
******************************/

void setup() {
    Serial.begin(9600);
    delay(100);
    Serial.println(F("Starting..."));

    /* For Pinoccio Scout boards. */
    #ifdef VCC_ENABLE
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    /* All LED pins as Output. */
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_YELLOW, OUTPUT);

    /* Turn leds off. */
    digitalWrite(LED_RED, HIGH); 
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_YELLOW, HIGH);

    pinMode(RADIO_RXTX2_PORT, OUTPUT);
    digitalWrite(RADIO_RXTX2_PORT, LOW);

    /* LMIC init. */
    os_init();
    
    /* Reset the MAC state. Session and pending data transfers will be discarded. */
    LMIC_reset();

//    LMIC_selectSubBand(1);

    /* Channels Control to AU915 (8 at 15 + 65 channels): */
    for (u1_t b = 0; b < 8; ++b)
    {
        LMIC_disableSubBand(b);
    }

    for (u1_t channel = 0; channel < 72; ++channel)
    {
        LMIC_disableChannel(channel);
    }

    /* ChirpStack AU915 */
    LMIC_enableChannel(8);
    LMIC_enableChannel(9);
    LMIC_enableChannel(10);
    LMIC_enableChannel(11);
    LMIC_enableChannel(12);
    LMIC_enableChannel(13);
    LMIC_enableChannel(14);
    LMIC_enableChannel(15);
//    LMIC_enableChannel(65); /* Test */

    /* Disable Adaptive Data Rate. */
    LMIC_setAdrMode(0);

    /* Disable link check validation. */
    LMIC_setLinkCheckMode(0);

    /* TTN uses SF9 for its RX2 window. */
    LMIC.dn2Dr = DR_SF9;

    /* Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library). */
    LMIC_setDrTxpow(DR_SF7,14);

    /* 
     *  Use with Arduino Pro Mini ATmega328P 3.3V 8MHz;
     *  Let LMIC compensate for +/- 1% clock error.
     */
    LMIC_setClockError (MAX_CLOCK_ERROR * 10 / 100);

    /* Start job (sending automatically starts OTAA too). */
    do_send(&sendjob);
}

/*****************************
 _                       
| |                      
| |     ___   ___  _ __  
| |    / _ \ / _ \| '_ \ 
| |___| (_) | (_) | |_) |
\_____/\___/ \___/| .__/ 
                  | |    
                  |_|    
*****************************/

void loop() {
    unsigned long now;
    now = millis();
    if ((now & 512) != 0) {
      digitalWrite(LED_GREEN, HIGH);
    }
    else {
      digitalWrite(LED_GREEN, LOW);
    }
    os_runloop_once();
}
