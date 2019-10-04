/*
 * Copyright (C) 2017 Robert Bosch. All Rights Reserved.
 *
 * Disclaimer
 *
 * Common:
 * Bosch Sensortec products are developed for the consumer goods industry. They may only be used
 * within the parameters of the respective valid product data sheet.  Bosch Sensortec products are
 * provided with the express understanding that there is no warranty of fitness for a particular purpose.
 * They are not fit for use in life-sustaining, safety or security sensitive systems or any system or device
 * that may lead to bodily harm or property damage if the system or device malfunctions. In addition,
 * Bosch Sensortec products are not fit for use in products which interact with motor vehicle systems.
 * The resale and/or use of products are at the purchasers own risk and his own responsibility. The
 * examination of fitness for the intended use is the sole responsibility of the Purchaser.
 *
 * The purchaser shall indemnify Bosch Sensortec from all third party claims, including any claims for
 * incidental, or consequential damages, arising from any product use not covered by the parameters of
 * the respective valid product data sheet or not approved by Bosch Sensortec and reimburse Bosch
 * Sensortec for all costs in connection with such claims.
 *
 * The purchaser must monitor the market for the purchased products, particularly with regard to
 * product safety and inform Bosch Sensortec without delay of all security relevant incidents.
 *
 * Engineering Samples are marked with an asterisk (*) or (e). Samples may vary from the valid
 * technical specifications of the product series. They are therefore not intended or fit for resale to third
 * parties or for use in end products. Their sole purpose is internal client testing. The testing of an
 * engineering sample may in no way replace the testing of a product series. Bosch Sensortec
 * assumes no liability for the use of engineering samples. By accepting the engineering samples, the
 * Purchaser agrees to indemnify Bosch Sensortec from all claims arising from the use of engineering
 * samples.
 *
 * Special:
 * This software module (hereinafter called "Software") and any information on application-sheets
 * (hereinafter called "Information") is provided free of charge for the sole purpose to support your
 * application work. The Software and Information is subject to the following terms and conditions:
 *
 * The Software is specifically designed for the exclusive use for Bosch Sensortec products by
 * personnel who have special experience and training. Do not use this Software if you do not have the
 * proper experience or training.
 *
 * This Software package is provided `` as is `` and without any expressed or implied warranties,
 * including without limitation, the implied warranties of merchantability and fitness for a particular
 * purpose.
 *
 * Bosch Sensortec and their representatives and agents deny any liability for the functional impairment
 * of this Software in terms of fitness, performance and safety. Bosch Sensortec and their
 * representatives and agents shall not be liable for any direct or indirect damages or injury, except as
 * otherwise stipulated in mandatory applicable law.
 *
 * The Information provided is believed to be accurate and reliable. Bosch Sensortec assumes no
 * responsibility for the consequences of use of such Information nor for any infringement of patents or
 * other rights of third parties which may result from its use. No license is granted by implication or
 * otherwise under any patent or patent rights of Bosch. Specifications mentioned in the Information are
 * subject to change without notice.
 *
 * It is not allowed to deliver the source code of the Software to any third party without permission of
 * Bosch Sensortec.
 *
 ***********************************************************************************************************************
 *
 * Wireless sensor for measuring temperature, humidity, airpressure, lightlevel and airquality.
 * Hardware: NodeMCU V1.0, WEMOS D1 mini, Generic STM32F103C8(e.g.BluePill) or Maple mini + BME680
 * Optional: RFM69CW, BH1750 and 128x64 OLED Display(SH1106 or SSD1306)
 * If you use a RFM69CW, protokoll is UniversalSensor, frequency is 868.3 MHz
 * New since V3.3: 
 * MQTT is possible on ESP8266 boards, therefore we use WiFi. Configuration for this is in file secrets.h .
 * CO2e and bVOC works now !
 * 
 * V1.3  10.11.2017, T.Hirte
 * V1.4  include BH1750 light sensor, 17.11.2017, T.Hirte
 * V1.5  - changed to Bosch Sensortec Software from 17.11.2017, 01.12.2017, T.Hirte
 *       - rounding of values for OLED Display is done by "display.print()", 05.12.2017, T.Hirte
 *       - changed display of gas resistance on OLED to 123.45kOhm, 18.12.2017, T.Hirte
 *       - you can now deactivate vcc measuring, BSEC version is shown at startup,
 *         crc-errors at receiving end: SPI clock reduced to 400kHz, now it's ok,
 *         correct hardware wiring (IRQ of RFM69 is not needed), 28.12.2017, T.Hirte
 * V1.6  UniversalSensor
 *       Added Altitude configuration
 *       Get voltage from ESP.getVcc() - does not really work, so used the old way ;o)
 *       Changed VERSION to float to be able to transmit it
 * V1.7  updated to latest changes of the CC-Sensor version, anything (except the BME680)
 *       can be enabled/disabled, changed to internal vcc measuring, works on NodeMCU and D1 mini...
 * V1.8  changed crc calculation to crc16, by HCS
 * V1.9  BME680 I2C adress is now autodetected ! NodeID is showing as hex value.
 *       Definition of not needed irq for RFMxx removed.
 *       Compiled with esp8266 V2.4.0 -> changed the vcc correction factor for exakt voltage display.
 * V2.0  Lightsensor and OLED display are now autodetected.
 *       BH1750 Lib's changed for this to work !
 *       RFMxx Lib changed, Soft-SPI selectable in RFMxx.h, 13.01.2018, T.Hirte
 * V2.1  RFM69 is now autodetected, if not present, output only on serial port.
 *       onboard led is flashing 3 times if setup completes successfully, code improvements, 15.01.2018, T.Hirte
 * V2.2  Debug mode and Soft-SPI is now selectable from the define's, no longer need to change RFMxx.h ;o)
 *       Debug messages added, code improvements, 20.01.2018, T.Hirte

 * V3.0  First version for ESP8266 (NodeMCU or Wemos D1 mini) and STM32F1 (Maple mini, BluePill)!
 *       In BSEC routines: WireEndTransmission for STM32xx fixed,
 *       in RFMxx.cpp: SPI speed for STM32xx fixed, 19.01.2018, T.Hirte
 *       Input of NodeID, Altitude and Temperature Offset at first start (serial terminal), values stored in EEPROM.
 *       Delete settings in EEPROM at startup possible, you have 5 seconds to say "Y", otherwise the stored 
 *       settings will be used. 12.02.2018, T.Hirte
 *       Autodetect of OLED I2C address moved to Adafruit lib (Adafruit_SSD1306.cpp + Adafruit_SSD1306.h),
 *       code improvements, 14.02.2018, T.Hirte
 *
 * V3.1  Update to BSEC-software V1.4.7.1, integrate changes from juergs (error if HAS_OLED = false, switch off wifi).
 *       Compiled with Arduino V1.8.7 (used ESP8266 package: V2.5.0 beta2) ;o) 27.12.2018, T.Hirte
 *
 * V3.2  Non public version for McArthur, iaq display with WS2812 NeoPixel and data transmission to Thingspeak.  
 *       New sensor outputs: bVOC and CO2e in bsec_integration.c activated.
 *       31.08.2019
 *
 * V3.3  Update to BSEC-software V1.4.7.4, compiled with Arduino V1.8.9, ESP8266 package: V2.5.2,
 *       New sensor outputs: bVOC and CO2e in bsec_integration.c activated.
 *       MQTT over WiFi on ESP8266 based clients, sends messages every 30 seconds, 
 *       All login data (WiFi, MQTT) stored in secrets.h !
 *       update hardware setup comments, in PubSubClient.h: MQTT_KEEPALIVE changed to 35,
 *       Adafruit_SSD1306.cpp code improvements - 15.09.2019, T.Hirte
 *       
 * V3.4  new option "homekit" - send iaq as value from 0 to 5, see config.h       
 *       24.09.2019, T.Hirte
 *       
 ***********************************************************************************************************************

 ******************
 * HARDWARE SETUP *
 ******************

   ESP8266:

                D1 mini                               NodeMCU
                +--\/--+                              +--\/--+
            RST |      | TX                        A0 |      | D0 int.LED
             A0 |      | RX                       RSV |      | D1 SCL -->
             D0 |      | D1 SCL -->               RSV |      | D2 SDA <->
    <--- SCK D5 |      | D2 SDA <->               SD3 |      | D3
    <-- MISO D6 |      | D3                       SD2 |      | D4
    --> MOSI D7 |      | D4 int.LED               SD1 |      | 3V3
    <--- NSS D8 |      | GND                      CMD |      | GND
       <-- 3.3V |      | 5V <-- 5V                SD0 |      | D5 SCK --->                  BME680              RFM69CW
                +------+                          CLK |      | D6 MISO <--                 +------+            +-------+
                                                  GND |      | D7 MOSI -->            3.3V | 0x76 |       3.3V |       | GND
                                                  3V3 |      | D8 NSS --->         <-> SDA | or   |   --> MOSI |       | NSS <--
                                                   EN |      | RX                  --> SCL | 0x77 |   <-- MISO |       |
                                                  RST |      | TX                      GND |      |   --> SCK  |       |
                                                  GND |      | GND -->                     +------+            +-------+
                                           5V --> Vin |      | 3V3 -->
                                                      +------+

   STM32F1:

                mapleMini                             BluePill                              
                +--\/--+                              +--\/--+
       <-- 3.3V |      | VCC 3.3V                PB12 |      | GND                          BH1750            SH1106 OLED
       <--- GND |      | GND                     PB13 |      | GND -->                     +------+            +-------+
          BOOT0 |      | VBAT                    PB14 |      | 3V3 -->                3.3V | 0x23 |       3.3V | 0x3C  |
   <-> SDA PB_7 |      | PC_13                   PB15 |      | NRST                    GND | or   |        GND | or    |
   <-- SCL PB_6 |      | PC_14                   PA8  |      | PB11                --> SCL | 0x5C |    --> SCL | 0x3D  |        
           PB_5 |      | PC_15               Tx1 PA9  |      | PB10                <-> SDA |      |    <-> SDA |       |
           PB_4 |      | RESET               Rx1 PA10 |      | PB1                         +------+            +-------+
           PB_3 |      | PA_0               USB- PA11 |      | PB0  A0                
          PA_15 |      | PA_1               USB+ PA12 |      | PA7  A1/MOSI1 -->           
          PA_14 |      | PA_2                    PA15 |      | PA6  A2/MISO1 <--      
          PA_13 |      | PA_3                    PB3  |      | PA5  A3/SCK1  -->       
          PA_12 |      | PA_4 NSS  -->           PB4  |      | PA4  A4/NSS1  -->   
          PA_11 |      | PA_5 SCK  -->           PB5  |      | PA3  A5             
          PA_10 |      | PA_6 MISO <--   <-- SCL PB6  |      | PA2  A6                     
           PA_9 |      | PA_7 MOSI -->   <-> SDA PB7  |      | PA1  A7
           PA_8 |      | PB_0                    PB8  |      | PA0  A8
          PB_15 |      | PB_2 int.LED            PB9  |      | PC15
          PB_14 |      | PB_10                     5V |      | PC14
          PB_13 |      | PB_11                    GND |      | PC13 int.LED
          PB_12 |      | + 5V                     3V3 |      | VBAT
                +------+                              +------+

  BluePill-Board
  ==============
  http://wiki.stm32duino.com/index.php?title=Blue_Pill

  Maple_Mini-Board
  ================
  https://wolfgangklenk.wordpress.com/2017/11/05/indoor-air-quality-iaq-measurement-with-bosch-bme680-and-stm32f103c8t6/

************************************************************************************************************************/
#include "config.h"
#include <Wire.h>
#include "bsec_integration.h"
#include "UniversalSensor.h"
#include "HandleEeprom.h"
#include "string"

#if USE_MQTT
  #ifndef ESP8266
    #error "MQTT client only for ESP8266 !"
  #endif
#endif

#if !USE_MQTT
  #if USE_HOMEKIT
    #error "For homekit option you must enable MQTT protocol !"
  #endif
#endif

#ifdef ESP8266
  #include "secrets.h"
  #include <ESP8266WiFi.h>
  byte my_WiFi_Mode = 0;
  WiFiServer server(80);
  WiFiClient espClient;
  #define MAX_PACKAGE_SIZE 2048
  char HTML_String[5000];
  char HTTP_Header[150];
#endif

#if USE_MQTT
  #include "PubSubClient.h"
  PubSubClient client(espClient);
  uint8_t mqtt_counter;
  uint8_t iaq_expr;
  char temp_topic[50];
  char pres_topic[50];
  char hum_topic[50];
  char iaq_topic[50];
  char iaq_accuracy_topic[50];
  char co2_topic[50];
  char voc_topic[50];
  char light_topic[50];
  char rssi_topic[50];
#endif

#if HAS_RFM69
  #include "RFMxx.h"
  #include "SensorBase.h"
#endif

#if HAS_OLED
  #include <Adafruit_GFX.h>
  #include "Adafruit_SSD1306.h"
#endif

#if HAS_LIGHTSENSOR
  #include "AS_BH1750.h"
#endif

uint8_t NODEID;
float TEMPOFFSET;
float ALTITUDE;
HandleEeprom                   eeprom;

//RFM69
#if HAS_RFM69
  unsigned long DATA_RATE    = 17241ul;    //default data rate (for transmit on RFM69)
  unsigned long INITIAL_FREQ = 868300;     //default frequency in kHz (5 kHz steps, 860480 ... 879515)
  bool RFM69                 = false;      //if RFM69 is not detected
  uint8_t rfm_interval       = 10;       //time to wait between data transmissions (10: 10 * 3sec = 30sec)
  uint8_t rfm_counter;
  //RFM69, PIN-config for SPI:
  #ifdef ESP8266
    #define RFM_SS             D8        //  SS pin -> RFM69 (NSS) for both Soft- and HW-SPI
    #define RFM_MISO           D6        //MISO pin <- RFM69 (MOSI) only used by soft spi
    #define RFM_MOSI           D7        //MOSI pin -> RFM69 (MISO) only used by soft spi
    #define RFM_SCK            D5        // SCK pin -> RFM69 (SCK) only used by soft spi
  #elif defined(__STM32F1__)
    #define RFM_SS             PA4       //  SS pin -> RFM69 (NSS) for both Soft- and HW-SPI
    #define RFM_SCK            PA5       // SCK pin -> RFM69 (SCK) only used by soft spi
    #define RFM_MISO           PA6       //MISO pin <- RFM69 (MOSI) only used by soft spi
    #define RFM_MOSI           PA7       //MOSI pin -> RFM69 (MISO) only used by soft spi
  #endif
  RFMxx rfm(RFM_MOSI, RFM_MISO, RFM_SCK, RFM_SS, SOFT_SPI);
#endif

//OLED display
#if HAS_OLED
  bool OLED                  = false;      //if display not detected
  Adafruit_SSD1306             display;
#endif

//Lightsensor
#if HAS_LIGHTSENSOR
  bool BH1750                = false;      //if BH1750 not detected
  AS_BH1750                    bh1750;
#endif

#ifdef ESP8266
  #if VCC_MEASURE
    ADC_MODE(ADC_VCC);                     //esp internal vcc measuring
  #endif
#endif

//Webserver
#ifdef ESP8266
  #define ACTION_OK 1
  #define ACTION_NOTOK 2
  #define ACTION_SET_MQTT 3
  #define ACTION_SET_WIFI 4
  #define ACTION_LIES_AUSWAHL 5
  int action;

  // checkboxen
  char devices_tab[6][7] = {"OLED  ", "RFM69 ", "BH1750", "MQTT  ", "VCC   ", "DEBUG "};
  byte devices = 0;
  char tmp_string[20];
#endif

/***********************************************************************************************************************/

static void blink (byte pin, byte n = 3, int del = 50)
{
  for (byte i = 0; i < n; ++i)
  {
    digitalWrite(pin, LOW);
    delay(del);
    digitalWrite(pin, HIGH);
    delay(del);
  }
}

/*****************************************************/
void dim_display (byte brightness)
{
   #if HAS_OLED
     if (OLED)
     {
       display.ssd1306_command(SSD1306_SETCONTRAST);
       display.ssd1306_command(brightness);
     }
   #endif
}

/*****************************************************/
#if USE_MQTT
  void mqtt_reconnect() {
    while (!client.connected()) {
      Serial.print("Connecting to MQTT server ... ");
      if (client.connect(mqtt_clientid,mqtt_user,mqtt_password)) {
        Serial.println("connected");
        blink(LEDpin,1,50);
      } else {
        Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.println(" try again in 2 seconds");
        blink(LEDpin,4,250);
      }
    }
  }
#endif

/*****************************************************/
#ifdef ESP8266
  void WiFi_Start_STA() {
    unsigned long timeout;
  
    WiFi.mode(WIFI_STA);   //  Workstation
  
    Serial.println();
    Serial.print("Connecting to ");
    Serial.print(wifi_ssid);
    Serial.print(" ");
  
    WiFi.begin(wifi_ssid, wifi_password);
    timeout = millis() + 10000L;
    while (WiFi.status() != WL_CONNECTED && millis() < timeout) {
      Serial.print(".");
      blink(LEDpin,2,50);
      delay(500);
    }
  
    if (WiFi.status() == WL_CONNECTED) {
      server.begin();
      my_WiFi_Mode = WIFI_STA;
      blink(LEDpin,1,100);
      Serial.println(" connected");
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
      uint8_t macAddr[6];
      WiFi.macAddress(macAddr);
      Serial.printf("MAC address: %02x:%02x:%02x:%02x:%02x:%02x\n", macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);
    }
    else {
      WiFi.mode(WIFI_OFF);
      Serial.println(" failed");
    }
  Serial.println();
  }

/*****************************************************/
  void WiFi_Start_AP() {
    WiFi.mode(WIFI_AP);   // Accesspoint
    WiFi.softAP(ssid_ap, password_ap);
    server.begin();
    IPAddress myIP = WiFi.softAPIP();
    my_WiFi_Mode = WIFI_AP;

    Serial.print("Accesspoint started, Name: ");
    Serial.print(ssid_ap);
    Serial.print( ", IP address: ");
    Serial.println(myIP);
  }

/*****************************************************/
void WiFi_Traffic() {

  char my_char;
  int htmlPtr = 0;
  int myIndex;
  unsigned long my_timeout;

  // Check if a client has connected
  espClient = server.available();
  if (!espClient)  {
    return;
  }

  my_timeout = millis() + 250L;
  while (!espClient.available() && (millis() < my_timeout) ) delay(10);
  delay(10);
  if (millis() > my_timeout)  {
    return;
  }
  //---------------------------------------------------
  htmlPtr = 0;
  my_char = 0;
  while (espClient.available() && my_char != '\r') {
    my_char = espClient.read();
    HTML_String[htmlPtr++] = my_char;
  }
  espClient.flush();
  HTML_String[htmlPtr] = 0;
#if DEBUG
  exhibit ("Request : ", HTML_String);
#endif

  if (Find_Start ("/?", HTML_String) < 0 && Find_Start ("GET / HTTP", HTML_String) < 0 ) {
    send_not_found();
    return;
  }

  //---------------------------------------------------
  // Benutzereingaben einlesen und verarbeiten
  //---------------------------------------------------
  action = Pick_Parameter_Zahl("ACTION=", HTML_String);

  // wifi-ssid und wifi-password
  if ( action == ACTION_SET_WIFI) {

    myIndex = Find_End("wifi_ssid=", HTML_String);
    if (myIndex >= 0) {
      Pick_Text(wifi_ssid, &HTML_String[myIndex], 32);
#if DEBUG
      exhibit ("wifi_ssid  : ", wifi_ssid);
#endif
    }

    myIndex = Find_End("wifi_password=", HTML_String);
    if (myIndex >= 0) {
      Pick_Text(wifi_password, &HTML_String[myIndex], 32);
#if DEBUG
      exhibit ("wifi_password  : ", wifi_password);
#endif
    }
  }
  
  // MQTT Settings
  if ( action == ACTION_SET_MQTT) {

    myIndex = Find_End("mqtt_server=", HTML_String);
    if (myIndex >= 0) {
      Pick_Text(mqtt_server, &HTML_String[myIndex], 15);
#if DEBUG
      Serial.print("MQTT Server: ");
      Serial.println(mqtt_server);
#endif
    }

    myIndex = Find_End("mqtt_port=", HTML_String);
    if (myIndex >= 0) {
      Pick_Text(mqtt_port, &HTML_String[myIndex], 4);
#if DEBUG
      Serial.print("MQTT Port: ");
      Serial.println(mqtt_port);
#endif
    }

    myIndex = Find_End("mqtt_user=", HTML_String);
    if (myIndex >= 0) {
      Pick_Text(mqtt_user, &HTML_String[myIndex], 32);
#if DEBUG
      Serial.print("MQTT User: ");
      Serial.println(mqtt_user);
#endif
    }
    
    myIndex = Find_End("mqtt_password=", HTML_String);
    if (myIndex >= 0) {
      Pick_Text(mqtt_password, &HTML_String[myIndex], 32);
#if DEBUG
      Serial.print("MQTT Password: ");
      Serial.println(mqtt_password);
#endif
    }
  }
  
  //Options select
  if ( action == ACTION_LIES_AUSWAHL) {
    devices = 0;
    for (int i = 0; i < 6; i++) {
      strcpy( tmp_string, "devices");
      strcati( tmp_string, i);
      strcat( tmp_string, "=");
      if (Pick_Parameter_Zahl(tmp_string, HTML_String) == 1) {
        devices |= 1 << i;
#if DEBUG
        Serial.print("Option");
        Serial.print(i);
        Serial.print(" = ");
        Serial.println(devices && i+1);
#endif    
      }
    }
    /*
    //set bits
    if (devices & 1) {
      HAS_OLED = true;
    }
    else {
      HAS_OLED = false;
    }
    if (devices & 2) {
      HAS_RFM69 = true;
    }
    else {
      HAS_RFM69 = false;
    }
    if (devices & 4) {
      HAS_LIGHTSENSOR = true;
    }
    else {
      HAS_LIGHTSENSOR = false;
    }
    if (devices & 8) {
      USE_MQTT = true;
    }
    else {
      USE_MQTT = false;
    }
    if (devices & 16) {
      VCC_MEASURE = true;
    }
    else {
      VCC_MEASURE = false;
    }
    if (devices & 32) {
      DEBUG = true;
    }
    else {
      DEBUG = false;
    }
    */
    
#if DEBUG    
    Serial.print("Options (BIN): ");
    Serial.println(devices,BIN);
    /*
    Serial.print("OLED: ");Serial.println(HAS_OLED);
    Serial.print("RFM69: ");Serial.println(HAS_RFM69);
    Serial.print("BH1750: ");Serial.println(HAS_LIGHTSENSOR);
    Serial.print("MQTT: ");Serial.println(USE_MQTT);
    Serial.print("VCC measure: ");Serial.println(VCC_MEASURE);
    Serial.print("DEBUG: ");Serial.println(DEBUG);
    */
#endif
  }

  //---------------------------------------------------
  //Antwortseite aufbauen

  make_HTML01();

  //---------------------------------------------------
  // Header aufbauen
  strcpy(HTTP_Header , "HTTP/1.1 200 OK\r\n");
  strcat(HTTP_Header, "Content-Length: ");
  strcati(HTTP_Header, strlen(HTML_String));
  strcat(HTTP_Header, "\r\n");
  strcat(HTTP_Header, "Content-Type: text/html\r\n");
  strcat(HTTP_Header, "Connection: close\r\n");
  strcat(HTTP_Header, "\r\n");

#if DEBUG
  exhibit("Header : ", HTTP_Header);
  exhibit("Laenge Header : ", strlen(HTTP_Header));
  exhibit("Laenge HTML   : ", strlen(HTML_String));
#endif

  espClient.print(HTTP_Header);
  delay(20);

  send_HTML();

}

//---------------------------------------------------
// HTML Seite 01 aufbauen
//---------------------------------------------------
void make_HTML01() {

  strcpy( HTML_String, "<!DOCTYPE html>");
  strcat( HTML_String, "<html>");
  strcat( HTML_String, "<head>");
  strcat( HTML_String, "<title>Universal Sensor Setup</title>");
  strcat( HTML_String, "</head>");
  strcat( HTML_String, "<body bgcolor=\"#adcede\">");
  strcat( HTML_String, "<font color=\"#000000\" face=\"VERDANA,ARIAL,HELVETICA\">");
  strcat( HTML_String, "<h1>Universal Sensor Setup</h1>");

  //---------------------------------------------------
  // Textfelder wifi-ssid und wifi-password
  strcat( HTML_String, "<h2>WiFi Settings</h2>");
  strcat( HTML_String, "<form>");
  strcat( HTML_String, "<table>");
  set_colgroup(150, 270, 150, 0, 0);

  strcat( HTML_String, "<tr>");
  strcat( HTML_String, "<td><b>SSID</b></td>");
  strcat( HTML_String, "<td>");
  strcat( HTML_String, "<input type=\"text\" style= \"width:200px\" name=\"wifi_ssid\" maxlength=\"20\" Value =\"");
  strcat( HTML_String, wifi_ssid);
  strcat( HTML_String, "\"></td>");
  strcat( HTML_String, "<td><button style= \"width:100px\" name=\"ACTION\" value=\"");
  strcati(HTML_String, ACTION_SET_WIFI);
  strcat( HTML_String, "\">set</button></td>");
  strcat( HTML_String, "</tr>");

  strcat( HTML_String, "<tr>");
  strcat( HTML_String, "<td><b>Password</b></td>");
  strcat( HTML_String, "<td>");
  strcat( HTML_String, "<input type=\"password\" style= \"width:200px\" name=\"wifi_password\" maxlength=\"20\" Value =\"");
  strcat( HTML_String, wifi_password);
  strcat( HTML_String, "\"></td>");
  strcat( HTML_String, "</tr>");

  strcat( HTML_String, "</table>");
  strcat( HTML_String, "</form>");
  strcat( HTML_String, "<br>");

  //---------------------------------------------------
  // MQTT Settings
  strcat( HTML_String, "<h2>MQTT Settings</h2>");
  strcat( HTML_String, "<form>");
  strcat( HTML_String, "<table>");
  set_colgroup(150, 270, 150, 0, 0);

  strcat( HTML_String, "<tr>");
  strcat( HTML_String, "<td><b>Server</b></td>");
  strcat( HTML_String, "<td><input type=\"text\" style= \"width:100px\" name=\"mqtt_server\" value=\"");
  strcat( HTML_String, mqtt_server);

  strcat( HTML_String, "\"></td>");
  strcat( HTML_String, "<td><button style= \"width:100px\" name=\"ACTION\" value=\"");
  strcati(HTML_String, ACTION_SET_MQTT);
  strcat( HTML_String, "\">set</button></td>");
  strcat( HTML_String, "</tr>");

  strcat( HTML_String, "<tr>");
  strcat( HTML_String, "<td><b>Port</b></td>");
  strcat( HTML_String, "<td><input type=\"text\"  style= \"width:100px\" name=\"mqtt_port\" value=\"");
  strcat( HTML_String, mqtt_port);
  strcat( HTML_String, "\"></td></tr>");

  strcat( HTML_String, "<tr>");
  strcat( HTML_String, "<td><b>User</b></td>");
  strcat( HTML_String, "<td><input type=\"text\"  style= \"width:200px\" name=\"mqtt_user\" value=\"");
  strcat( HTML_String, mqtt_user);
  strcat( HTML_String, "\"></td></tr>");
  
  strcat( HTML_String, "<tr>");
  strcat( HTML_String, "<td><b>Password</b></td>");
  strcat( HTML_String, "<td><input type=\"password\"  style= \"width:200px\" name=\"mqtt_password\" value=\"");
  strcat( HTML_String, mqtt_password);
  strcat( HTML_String, "\"></td></tr>");
  
  strcat( HTML_String, "</table>");
  strcat( HTML_String, "</form>");
  strcat( HTML_String, "<br>");

  //---------------------------------------------------
  // Options
  strcat( HTML_String, "<h2>Options</h2>");
  strcat( HTML_String, "<form>");
  strcat( HTML_String, "<table>");
  set_colgroup(150, 270, 150, 0, 0);
  strcat( HTML_String, "<tr>");
  strcat( HTML_String, "<td>");

  for (int i = 0; i < 6; i++) {
    if (!i == 0)strcat( HTML_String, "<br>");
    strcat( HTML_String, "<input type=\"checkbox\" name=\"devices");
    strcati( HTML_String, i);
    strcat( HTML_String, "\" id = \"DEV");
    strcati( HTML_String, i);
    strcat( HTML_String, "\" value = \"1\" ");
    if (devices & 1 << i) strcat( HTML_String, "checked ");
    strcat( HTML_String, "> ");
    strcat( HTML_String, "<label for =\"DEV");
    strcati( HTML_String, i);
    strcat( HTML_String, "\">");
    strcat( HTML_String, devices_tab[i]);
    strcat( HTML_String, "</label>");
  }
  strcat( HTML_String, "</td>");

  strcat( HTML_String, "<td><button style= \"width:100px\" name=\"ACTION\" value=\"");
  strcati(HTML_String, ACTION_LIES_AUSWAHL);
  strcat( HTML_String, "\">set</button></td>");
  strcat( HTML_String, "</tr>");
  strcat( HTML_String, "</font>");
  strcat( HTML_String, "</font>");
  strcat( HTML_String, "</body>");
  strcat( HTML_String, "</html>");
}

//--------------------------------------------------------------------------
void send_not_found() {
#if DEBUG
  Serial.println("Sende Not Found");
#endif
  espClient.print("HTTP/1.1 404 Not Found\r\n\r\n");
  delay(20);
  espClient.stop();
}

//--------------------------------------------------------------------------
void send_HTML() {
  char my_char;
  int  my_len = strlen(HTML_String);
  int  my_ptr = 0;
  int  my_send = 0;

  //--------------------------------------------------------------------------
  // in Portionen senden
  while ((my_len - my_send) > 0) {
    my_send = my_ptr + MAX_PACKAGE_SIZE;
    if (my_send > my_len) {
      espClient.print(&HTML_String[my_ptr]);
      delay(20);
#if DEBUG
      Serial.println(&HTML_String[my_ptr]);
#endif
      my_send = my_len;
    } else {
      my_char = HTML_String[my_send];
      // Auf Anfang eines Tags positionieren
      while ( my_char != '<') my_char = HTML_String[--my_send];
      HTML_String[my_send] = 0;
      espClient.print(&HTML_String[my_ptr]);
      delay(20);
#if DEBUG
      Serial.println(&HTML_String[my_ptr]);
#endif
      HTML_String[my_send] =  my_char;
      my_ptr = my_send;
    }
  }
  espClient.stop();
}

//----------------------------------------------------------------------------------------------
void set_colgroup(int w1, int w2, int w3, int w4, int w5) {
  strcat( HTML_String, "<colgroup>");
  set_colgroup1(w1);
  set_colgroup1(w2);
  set_colgroup1(w3);
  set_colgroup1(w4);
  set_colgroup1(w5);
  strcat( HTML_String, "</colgroup>");
}

//------------------------------------------------------------------------------------------
void set_colgroup1(int ww) {
  if (ww == 0) return;
  strcat( HTML_String, "<col width=\"");
  strcati( HTML_String, ww);
  strcat( HTML_String, "\">");
}

//---------------------------------------------------------------------
void strcati(char* tx, int i) {
  char tmp[8];

  itoa(i, tmp, 10);
  strcat (tx, tmp);
}

//---------------------------------------------------------------------
void strcati2(char* tx, int i) {
  char tmp[8];

  itoa(i, tmp, 10);
  if (strlen(tmp) < 2) strcat (tx, "0");
  strcat (tx, tmp);
}

//---------------------------------------------------------------------
int Pick_Parameter_Zahl(const char * par, char * str) {
  int myIdx = Find_End(par, str);

  if (myIdx >= 0) return  Pick_Dec(str, myIdx);
  else return -1;
}

//---------------------------------------------------------------------
int Find_End(const char * such, const char * str) {
  int tmp = Find_Start(such, str);
  if (tmp >= 0)tmp += strlen(such);
  return tmp;
}

//---------------------------------------------------------------------
int Find_Start(const char * such, const char * str) {
  int tmp = -1;
  int ww = strlen(str) - strlen(such);
  int ll = strlen(such);

  for (int i = 0; i <= ww && tmp == -1; i++) {
    if (strncmp(such, &str[i], ll) == 0) tmp = i;
  }
  return tmp;
}

//---------------------------------------------------------------------
int Pick_Dec(const char * tx, int idx ) {
  int tmp = 0;

  for (int p = idx; p < idx + 5 && (tx[p] >= '0' && tx[p] <= '9') ; p++) {
    tmp = 10 * tmp + tx[p] - '0';
  }
  return tmp;
}

//----------------------------------------------------------------------------
int Pick_N_Zahl(const char * tx, char separator, byte n) {

  int ll = strlen(tx);
  
  //int tmp = -1;
  byte anz = 1;
  byte i = 0;
  while (i < ll && anz < n) {
    if (tx[i] == separator)anz++;
    i++;
  }
  if (i < ll) return Pick_Dec(tx, i);
  else return -1;
}

//---------------------------------------------------------------------
int Pick_Hex(const char * tx, int idx ) {
  int tmp = 0;

  for (int p = idx; p < idx + 5 && ( (tx[p] >= '0' && tx[p] <= '9') || (tx[p] >= 'A' && tx[p] <= 'F')) ; p++) {
    if (tx[p] <= '9')tmp = 16 * tmp + tx[p] - '0';
    else tmp = 16 * tmp + tx[p] - 55;
  }

  return tmp;
}

//---------------------------------------------------------------------
void Pick_Text(char * tx_ziel, char  * tx_quelle, int max_ziel) {

  int p_ziel = 0;
  int p_quelle = 0;
  int len_quelle = strlen(tx_quelle);

  while (p_ziel < max_ziel && p_quelle < len_quelle && tx_quelle[p_quelle] && tx_quelle[p_quelle] != ' ' && tx_quelle[p_quelle] !=  '&') {
    if (tx_quelle[p_quelle] == '%') {
      tx_ziel[p_ziel] = (HexChar_to_NumChar( tx_quelle[p_quelle + 1]) << 4) + HexChar_to_NumChar(tx_quelle[p_quelle + 2]);
      p_quelle += 2;
    } else if (tx_quelle[p_quelle] == '+') {
      tx_ziel[p_ziel] = ' ';
    }
    else {
      tx_ziel[p_ziel] = tx_quelle[p_quelle];
    }
    p_ziel++;
    p_quelle++;
  }

  tx_ziel[p_ziel] = 0;
}

//---------------------------------------------------------------------
char HexChar_to_NumChar( char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 55;
  return 0;
}

#if DEBUG
//---------------------------------------------------------------------
void exhibit(const char * tx, int v) {
  Serial.print(tx);
  Serial.println(v);
}
//---------------------------------------------------------------------
void exhibit(const char * tx, unsigned int v) {
  Serial.print(tx);
  Serial.println(v);
}
//---------------------------------------------------------------------
void exhibit(const char * tx, unsigned long v) {
  Serial.print(tx);
  Serial.println(v);
}
//---------------------------------------------------------------------
void exhibit(const char * tx, const char * v) {
  Serial.print(tx);
  Serial.println(v);
}
#endif

#endif //ifdef ESP8266

/*********************************************************************
 * Write operation in either Wire or SPI
 *
 * param[in]        dev_addr        Wire or SPI device address
 * param[in]        reg_addr        register address
 * param[in]        reg_data_ptr    pointer to the data to be written
 * param[in]        data_len        number of bytes to be written
 *
 * return           result of the bus communication function
 */
int8_t bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr, uint16_t data_len)
{
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);    /* Set register address to start writing to */

    /* Write the data */
    for (int index = 0; index < data_len; index++)
    {
        Wire.write(reg_data_ptr[index]);
    }
    #ifdef __STM32F1__
      Wire.endTransmission();
      return 0;
    #else
      return (int8_t)Wire.endTransmission();
    #endif
}
/*********************************************************************
 * Read operation in either Wire or SPI
 *
 * param[in]        dev_addr        Wire or SPI device address
 * param[in]        reg_addr        register address
 * param[out]       reg_data_ptr    pointer to the memory to be used to store the read data
 * param[in]        data_len        number of bytes to be read
 *
 * return           result of the bus communication function
 */
int8_t bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr, uint16_t data_len)
{
    int8_t comResult = 0;
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);                    /* Set register address to start reading from */
    comResult = Wire.endTransmission();

    delayMicroseconds(150);                 /* Precautionary response delay */
    Wire.requestFrom(dev_addr, (uint8_t)data_len);    /* Request data */

    int index = 0;
    while (Wire.available())  /* The slave device may send less than requested (burst read) */
    {
        reg_data_ptr[index] = Wire.read();
        index++;
    }
    #ifdef __STM32F1__
      return 0;
    #else
      return comResult;
    #endif
}
/*********************************************************************
 * System specific implementation of sleep function
 *
 * param[in]       t_ms    time in milliseconds
 *
 * return          none
 */
void sleep(uint32_t t_ms)
{
    delay(t_ms);
}

/*********************************************************************
 * Capture the system time in microseconds
 *
 * return           system_current_time    current system timestamp in microseconds
 */
int64_t get_timestamp_us()
{
    return (int64_t) millis() * 1000;
}

/*********************************************************************
 * Load previous library state from non-volatile memory
 *
 * param[in,out]    state_buffer    buffer to hold the loaded state string
 * param[in]        n_buffer        size of the allocated state buffer
 *
 * return           number of bytes copied to state_buffer
 */
uint32_t state_load(uint8_t *state_buffer, uint32_t n_buffer)
{
    // ...
    // Load a previous library state from non-volatile memory, if available.
    //
    // Return zero if loading was unsuccessful or no state was available,
    // otherwise return length of loaded state string.
    // ...
    return 0;
}

/*********************************************************************
 * Save library state to non-volatile memory
 *
 * param[in]        state_buffer    buffer holding the state to be stored
 * param[in]        length          length of the state string to be stored
 *
 * return           none
 */
void state_save(const uint8_t *state_buffer, uint32_t length)
{
    // ...
    // Save the string some form of non-volatile memory, if possible.
    // ...
}

/*********************************************************************
 * Load library config from non-volatile memory
 *
 * param[in,out]    config_buffer    buffer to hold the loaded state string
 * param[in]        n_buffer        size of the allocated state buffer
 *
 * return           number of bytes copied to config_buffer
 */
uint32_t config_load(uint8_t *config_buffer, uint32_t n_buffer)
{
    // ...
    // Load a library config from non-volatile memory, if available.
    //
    // Return zero if loading was unsuccessful or no config was available,
    // otherwise return length of loaded config string.
    // ...
    return 0;
}

/***********************************************************************************************************************
 * This is our main loop, it runs every 3 seconds                                                                      *
 ***********************************************************************************************************************/
void output_ready(int64_t timestamp, float iaq, uint8_t iaq_accuracy, float temperature, float humidity, float pressure, float raw_temperature, float raw_humidity, float gas, bsec_library_return_t bsec_status, float static_iaq, float co2_equivalent, float breath_voc_equivalent)
{
  #if DEBUG
    unsigned long cycle_start = millis();
  #endif

  #ifdef ESP8266
   if (my_WiFi_Mode == WIFI_AP) {
     WiFi_Traffic();
     Serial.println("*** Accesspoint activ ! ***");
   }
  #endif
      
  //get battery voltage
  #if VCC_MEASURE
    #ifdef ESP8266
      float vcc = ESP.getVcc() / 930.9; //correction for 220k/100k voltage divider to A0 on NodeMCU and D1 mini...
    #elif defined(__STM32F1__)
      //int vcc = analogRead(A0);
      float vcc = 3.3; //TODO: for STM32
    #else
      float vcc = 0xFF;
    #endif
  #else
    float vcc = 0xFF;
  #endif

  //get light level
  #if HAS_LIGHTSENSOR
    float lux;
    if (BH1750)
    {
      lux = bh1750.readLightLevel();
      #if HAS_OLED
        if (OLED)
        {
          //dimming the display to save OLED
          if (lux < 10) {
            dim_display(0);
          }
          else if (lux > 9 && lux < 50) {
            dim_display(0x20);
          }
          else if (lux > 49 && lux < 100) {
            dim_display(0x40);
          }
          else if (lux > 99 && lux < 500) {
            dim_display(0x60);
          }
          else {
            dim_display(0x80);
          }
        }
      #endif
    }
    else
    {
      lux = 0xFFFFFF;
    }
  #else
    float lux = 0xFFFFFF;
  #endif

  pressure /= pow(((float) 1.0 - ((float)ALTITUDE / 44330.0)), (float) 5.255);
  pressure /= 100;

  #if HAS_OLED
    if (OLED)
    {
      display.setCursor(0, 0);
      display.clearDisplay();
    }
  #endif

  //Output measurings
  Serial.print("[");
  Serial.print(timestamp / 1e9);
  Serial.print("] T: ");
  Serial.print(temperature);
  Serial.print("| P: ");
  Serial.print(pressure, 1);
  Serial.print("| rH: ");
  Serial.print(humidity);
  Serial.print("| IAQ: ");
  Serial.print(iaq);
  Serial.print(" (");
  Serial.print(iaq_accuracy);
  Serial.print(")");
  /***** new since BSEC V1.4.7.1: *****/
  Serial.print("| Static IAQ: ");
  Serial.print(static_iaq);
  Serial.print("| CO2: ");
  Serial.print(co2_equivalent);
  Serial.print("| VOC: ");
  Serial.print(breath_voc_equivalent);
  /************************************/
  Serial.print("| Gas: ");
  Serial.print(gas);
  #if VCC_MEASURE
    Serial.print("| UBat: ");
    Serial.print(vcc, 1);
    Serial.print("V");
  #endif
  #if HAS_LIGHTSENSOR
    if (BH1750)
    {
      Serial.print("| Light: ");
      Serial.print(lux);
      Serial.print("lx");
    }
  #endif
  Serial.println("");

  //on OLED Display
  #if HAS_OLED
    if (OLED)
    {
      display.setTextSize(2);
      display.print(temperature, 1);
      display.setTextSize(1);
      display.setCursor(48, 7);
      display.write(247);
      display.print("C ");

      display.setTextSize(2);
      display.setCursor(74, 0);
      display.print(humidity, 0);
      display.setTextSize(1);
      display.setCursor(98, 7);
      display.println("%");
      display.println();

      display.print("Luftdruck : ");
      display.print(pressure, 0);
      display.println(" hPa");

      display.print("Luftguete : ");
      if(iaq_accuracy > 0)
      {
        display.print(iaq, 0);
        display.print(" (");
        display.print(iaq_accuracy);
        display.print(")");
      }
      else
      {
        display.print("berechne");
      }
      display.println();

      display.print("CO2 Gehalt: ");
      display.print(co2_equivalent, 0);
      display.println(" ppm");
      
      display.print("VOC       : ");
      display.print(breath_voc_equivalent, 0);
      display.println("");

      #if HAS_LIGHTSENSOR
        if (BH1750)
        {
          display.print("Licht    : ");
          display.print(lux, 0);
          display.println(" lx");
        }
      #endif
      display.display();
    }
  #endif

  #if HAS_RFM69
    if (RFM69)
    {
      rfm_counter += 1;
      if(rfm_counter >= rfm_interval) //send every 30 seconds (rfm_interval(10) * 3)
      {
        UniversalSensor::Frame data;
        data.ID = NODEID;
        data.Flags = 0;
        data.Temperature = temperature;
        data.Humidity = humidity;
        data.Pressure = pressure;
        data.Gas1 = iaq;
        data.Gas2 = co2_equivalent; //gas
        data.Lux = lux;
        data.Voltage = vcc;
        data.Version = VERSION * 10;
        data.Rain = 0xFFFF;
        data.WindDirection = 0xFFFF;
        data.WindGust = 0xFFFF;
        data.WindSpeed = 0xFFFF;
        data.Debug = 0xFFFFFF;

        byte bytes[UniversalSensor::FRAME_LENGTH];
        UniversalSensor::EncodeFrame(&data, bytes);
        rfm.SendArray(bytes, UniversalSensor::FRAME_LENGTH);
        rfm.PowerDown();

        #if DEBUG
          Serial.print(sizeof(bytes));
          Serial.print(" bytes");
          Serial.println(" sent.");
          Serial.println();
        #endif

        rfm_counter = 0; 
        blink(LEDpin, 1, 25);
      }
    }
  #endif

  #if USE_MQTT
    if (my_WiFi_Mode == WIFI_STA) {
      
      mqtt_counter += 1;
      if(mqtt_counter >= mqtt_interval) {  //if mqtt_interval = 10, send every 30 seconds (mqtt_interval * 3)
        
        if (!client.connected()) {
          mqtt_reconnect();
        }
      
        Serial.print("[");
        Serial.print(timestamp / 1e9);
        Serial.print("] ");
        Serial.print("MQTT data >>> ");
     
        Serial.print("temperature: ");
        Serial.print(temperature);
        sprintf(temp_topic,"%s/temperature",mqtt_room);
        client.publish(temp_topic, String(temperature).c_str());
      
        Serial.print(", pressure: ");
        Serial.print(pressure);
        sprintf(pres_topic,"%s/pressure",mqtt_room);
        client.publish(pres_topic, String(pressure).c_str());
      
        Serial.print(", humidity: ");
        Serial.print(humidity);
        sprintf(hum_topic,"%s/humidity",mqtt_room);
        client.publish(hum_topic, String(humidity).c_str());

        sprintf(iaq_topic,"%s/iaq",mqtt_room);
        Serial.print(", iaq: ");
        #if USE_HOMEKIT
          if (iaq < 51) iaq_expr = 1; //"excellent";
          if (iaq >= 51 && iaq < 150) iaq_expr = 2; //"good";
          if (iaq >= 150 && iaq < 200) iaq_expr = 3; //"fair";
          if (iaq >= 201 && iaq < 350) iaq_expr = 4; //"inferior";
          if (iaq > 350) iaq_expr = 5; //"poor";
          Serial.print(iaq_expr);
          client.publish(iaq_topic, String(iaq_expr).c_str());
        #else
          Serial.print(iaq);
          client.publish(iaq_topic, String(iaq).c_str());
        #endif
        
        Serial.print(", iaq_accuracy: ");
        Serial.print(iaq_accuracy);
        sprintf(iaq_accuracy_topic,"%s/iaq_accuracy",mqtt_room);
        client.publish(iaq_accuracy_topic, String(iaq_accuracy).c_str());
      
        Serial.print(", co2: ");
        Serial.print(co2_equivalent);
        sprintf(co2_topic,"%s/co2",mqtt_room);
        client.publish(co2_topic, String(co2_equivalent).c_str());

        Serial.print(", voc: ");
        Serial.print(breath_voc_equivalent);
        sprintf(voc_topic,"%s/voc",mqtt_room);
        client.publish(voc_topic, String(breath_voc_equivalent).c_str());

        Serial.print(", light: ");
        Serial.print(lux);
        sprintf(light_topic,"%s/light",mqtt_room);
        client.publish(light_topic, String(lux).c_str());

        Serial.print(", rssi: ");
        Serial.println(WiFi.RSSI());
        sprintf(rssi_topic,"%s/rssi",mqtt_room);
        client.publish(rssi_topic, String(WiFi.RSSI()).c_str());
      
        mqtt_counter = 0;
        blink(LEDpin, 2, 25);
      }
    }
  #endif

#if DEBUG
  unsigned long cycle = millis() - cycle_start;
  Serial.print("Zykluszeit (ms): "); Serial.println(cycle);
#endif
}

/***********************************************************************************************************************/
void setup()
{

    pinMode(LEDpin, OUTPUT);
    digitalWrite(LEDpin, HIGH);

    Serial.begin(115200);
    
    Serial.println();
    //waiting 6 seconds to open terminal
    for (int i = 6; i > 0; i--) {
      Serial.print(".");
      delay(1000);
    }
    Serial.println("");
    
    #if DEBUG
      eeprom.SetDebugMode(true);
      #if HAS_RFM69
        rfm.SetDebugMode(true);
      #endif
    #endif

    return_values_init ret;

    //get bsec version
    bsec_version_t  version;
    bsec_get_version(&version);

    Serial.println();
    Serial.print("BME680 wireless sensor V");
    Serial.println(VERSION, 1);
    Serial.print("Compiled: ");
    Serial.println(__TIMESTAMP__);
    #if HAS_RFM69
      Serial.println("868MHz radio protocol: UniversalSensor");
    #endif
    #ifdef ESP8266    
      #if USE_MQTT
        Serial.println("MQTT over WiFi enabled");
        #if USE_HOMEKIT
          Serial.println("Homekit option selected (iaq value format)");
        #endif
      #endif
      Serial.printf("BSEC version: %d.%d.%d.%d\n",version.major, version.minor, version.major_bugfix, version.minor_bugfix);
      Serial.print("ESP Core Version: ");
      Serial.println(ESP.getCoreVersion());
      Serial.print("ESP SDK Version: ");
      Serial.println(ESP.getSdkVersion());
    #elif defined(__STM32F1__)
      Serial.print("BSEC Version: ");
      Serial.print(version.major);
      Serial.print(".");
      Serial.print(version.minor);
      Serial.print(".");
      Serial.print(version.major_bugfix);
      Serial.print(".");
      Serial.println(version.minor_bugfix);
    #endif
      Serial.println();
      
    //init I2C
    Wire.begin();
    Wire.setClock(400000);

    #if HAS_OLED
      //init OLED
      if (display.begin(SSD1306_SWITCHCAPVCC)) {

        OLED = true;
        display.setTextSize(2);
        display.setTextColor(WHITE, BLACK);
        display.setCursor(5,1);
        display.ssd1306_command(SSD1306_SETCONTRAST);
        display.ssd1306_command(0x80);
        display.clearDisplay();

        //startscreen
        display.println("  BME680");
        display.display();
        display.setTextSize(1);
        display.println();
        display.print("Wireless Sensor V");
        display.println(VERSION, 1);
        display.println("UniversalSensor Prot.");
        display.print("BSEC Ver.: ");
        display.print(version.major);
        display.print(".");
        display.print(version.minor);
        display.print(".");
        display.print(version.major_bugfix);
        display.print(".");
        display.println(version.minor_bugfix);
        display.display();
        sleep(5000);
      }
    #endif
    
    #ifdef ESP8266
      EEPROM.begin(6); //we need only 6 bytes
    #endif
    
    //restore defaults
    eeprom.RestoreDefaults();
    
    //read settings from EEPROM
    NODEID = eeprom.ReadNodeID();
    ALTITUDE = eeprom.ReadAltitude();
    TEMPOFFSET = eeprom.ReadTempOffset();
    
    //if no or not all values stored in EEPROM, edit settings
    if ( NODEID == 0xFF || ALTITUDE == 65560.5 || (TEMPOFFSET > -1.2 && TEMPOFFSET < -1.0) )
    {
      #if HAS_OLED
        if (OLED)
          {
            display.setCursor(0,0);
            display.clearDisplay();
            display.println("   *** SETUP ***");
            display.println();
            display.display();
          }
      #endif
      
      //edit nodeID
      if (NODEID == 0xFF)
        NODEID = eeprom.EditNodeID(NODEID);
      
      //edit altitude
      if (ALTITUDE == 65560.5)
        ALTITUDE = eeprom.EditAltitude(ALTITUDE);
      
      //edit temperature offset
      if (TEMPOFFSET > -1.2 && TEMPOFFSET < -1.0)
        TEMPOFFSET = eeprom.EditTempOffset(TEMPOFFSET);
        
      Serial.println("Save settings ...");
      eeprom.SaveSettings(NODEID, ALTITUDE, TEMPOFFSET);
    }
    else
    {
      Serial.println("Sensor Settings");

      #if HAS_OLED
        if (OLED)
          {
            display.setCursor(0,0);
            display.clearDisplay();
            display.println("   *** KONFIG ***   ");
            display.println();
            display.display();
          }
      #endif
    }
    
    Serial.print("Node-ID           : ");
    Serial.print(NODEID, DEC);
    Serial.print(" (0x");
    if (NODEID < 16)
      Serial.print("0");
    Serial.print(NODEID, HEX);
    Serial.println(")");

    Serial.print("Altitude          : ");
    Serial.print(ALTITUDE, 1);
    Serial.println("m");

    Serial.print("Temperature offset: ");
    Serial.print(TEMPOFFSET, 1);
    Serial.println(" degrees celsius");
    Serial.println();

    #if HAS_OLED
      if (OLED)
      {
        display.print("Node-ID  : ");
        display.print(NODEID, DEC);
        display.print(" (0x");
        if (NODEID < 16)
          display.print("0");
        display.print(NODEID, HEX);
        display.println(")");
        display.print("Hoehe    : ");
        display.print(ALTITUDE, 1);
        display.println("m");
        display.print("TempOffs.: ");
        display.print(TEMPOFFSET, 1);
        display.write(247);
        display.println("C");
        display.display();
        sleep(5000);
        display.setCursor(0,0);
        display.clearDisplay();
      }
    #endif
    
    #if DEBUG
      Serial.println("enabled components:");
      #if USE_MQTT
        Serial.println("MQTT over WiFi");
        #if USE_HOMEKIT
          Serial.println("Homekit option for IAQ value format enabled");
        #endif
      #endif
      #if HAS_RFM69
        Serial.println("RFM69 - 433/868/915MHz transmitter");
      #endif
      #if HAS_LIGHTSENSOR
        Serial.println("BH1750 - lightsensor");
      #endif
      #if HAS_OLED
        Serial.println("OLED - 128x64 OLED display (SH1106 or SSD1306)");
      #endif
      #if VCC_MEASURE
        Serial.println("measuring of battery/supply voltage");
      #endif
      Serial.println("");
    #endif

    /********************** BME680 init **********************/
    Serial.print("BME680 init ... ");
    #if HAS_OLED
      if (OLED)
      {
        display.print("BME680 init...");
        display.display();
      }
    #endif

    /* Call to the function which initializes the BSEC library
     * Switch on low-power mode and provide no temperature offset
     * for UltraLowPower Mode change following line to:
     * ret = bsec_iot_init(BSEC_SAMPLE_RATE_ULP, ... */

    ret = bsec_iot_init(BSEC_SAMPLE_RATE_LP, TEMPOFFSET, bus_write, bus_read, sleep, state_load, config_load);
    
    if (ret.bme680_status)
    {
      //Could not intialize BME680
      Serial.println("Error while initializing, BME680 connected ?");
      #if HAS_OLED
        if (OLED)
        {
          display.println();
          display.println("BME680 init. Fehler !");
          display.println("nicht angeschlossen ?");
          display.display();
        }
      #endif
      return;  //jump to main loop
    }
    else if (ret.bsec_status)
    {
      //Could not intialize BSEC library
      Serial.println("Error while initializing BSEC library !");
      #if HAS_OLED
        if (OLED)
        {
          display.println();
          display.println("BSEC init. Fehler !");
          display.display();
        }
      #endif
      return;  //jump to main loop
    }

    Serial.println("done");
    #if HAS_OLED
      if (OLED)
      {
        display.println("bereit");
        display.display();
      }
    #endif

    /********************** RFM69 init *************************/
    #if HAS_RFM69
      if (rfm.Begin())
      {
        RFM69 = true; //RFM69 is present
        rfm_counter = rfm_interval; //to send data at startup
        #if HAS_OLED
          if (OLED)
          {
            display.println("RFM69 gefunden.");
            display.display();
          }
        #endif
        Serial.print("RFM69  init ... ");

        rfm.InitializeLaCrosse();
        #if DEBUG
          Serial.println();
          Serial.println("Init LaCrosse done");
        #endif

        rfm.SetFrequency(INITIAL_FREQ);
        float init_freq = float(INITIAL_FREQ);
        #if HAS_OLED
          if (OLED)
          {
            display.print("Frequenz: ");
            display.print(init_freq/1000,3);
            display.println(" MHz");
          }
        #endif
        #if DEBUG
          Serial.print("Set frequency to ");
          Serial.print(init_freq/1000,3);
          Serial.println(" MHz");
        #endif

        rfm.SetDataRate(DATA_RATE);
        #if HAS_OLED
          if (OLED)
          {
            display.print("Baudrate: ");
            display.print(DATA_RATE);
            display.println(" Baud");
          }
        #endif
        #if DEBUG
          Serial.print("Set datarate to ");
          Serial.print(DATA_RATE);
          Serial.println(" bps");
        #endif

        rfm.PowerDown(); // sleep to save power
        Serial.println("done");
        #if HAS_OLED
          if (OLED)
          {
            display.println("RFM69 bereit.");
            display.display();
          }
        #endif
      }
      else
      {
        #if HAS_OLED
          if (OLED)
          {
            display.println("kein RFM69 gefunden!");
            display.display();
          }
        #endif
        Serial.println("RFM69 not found");
      }
    #endif

    /********************** BH1750 init **********************/
    #if HAS_LIGHTSENSOR
      Serial.print("BH1750 init ... ");
      #if HAS_OLED
        if (OLED)
        {
          display.print("BH1750 init...");
          display.display();
        }
      #endif
      /* for normal sensor resolution (1 lx resolution, 0-65535 lx, 120ms, no PowerDown),
      use: bh1750.begin(RESOLUTION_NORMAL, false); */
      if (bh1750.begin())
      {
        BH1750 = true; //sensor found on 0x23 or 0x5C
        Serial.println("done");
        #if HAS_OLED
          if (OLED)
          {
            display.println("bereit");
            display.display();
          }
        #endif
      }
      else
      {
        #if HAS_OLED
          if (OLED)
          {
            display.println("fehlt.");
            display.display();
          }
        #endif
        Serial.println("not present");
        BH1750 = false;
      }
    #endif

    /************** start WiFi / MQTT **************************/
    #ifdef ESP8266
      WiFi_Start_STA();                        //try to connect to an accesspoint
      if (my_WiFi_Mode == 0) WiFi_Start_AP();  //if no accesspoint available, starting our own accesspoint
    #endif    

    #if USE_MQTT
      mqtt_interval /= 3;  //time in seconds / 3 (iot_loop runs every 3 seconds once)
      
      //if RFM69 and MQTT in use, first transmission of MQTT data after 15 seconds, rfm sends data directly at startup
      if(RFM69) {
        mqtt_counter /= 2; //half of normal interval
      }
      else {
        mqtt_counter = mqtt_interval;
      }

      #if DEBUG
        Serial.print("first transmission of MQTT data in ");
        Serial.print((mqtt_interval - mqtt_counter) * 3);
        Serial.println(" seconds");
      #endif

      if (my_WiFi_Mode == WIFI_STA) {
        //connect to MQTT Server
        client.setServer(mqtt_server, atoi(mqtt_port));
        mqtt_reconnect();
      }

    #endif

    /************** Setup succesfull ***************/
    blink(LEDpin, 3, 250);
    Serial.println();
    Serial.println("Ready, start measuring ...");
    Serial.println();
    #if HAS_OLED
      if (OLED)
      {
        display.print("Starte Messungen ...");
        display.display();
        sleep(3000); //time to read display messages
        display.clearDisplay();
        display.display();
      }
    #endif

    /* todo ...
       Call to endless loop function which reads and processes data based on sensor settings
       State is saved every 10.000 samples, which means every 10.000 * 3 secs = 500 minutes  */
       bsec_iot_loop(sleep, get_timestamp_us, output_ready, state_save, 10000);
}

/*********************************************************************************************/

void loop()
{
  //BME680 had an error during setup
  blink(LEDpin, 3, 250);
  sleep(1000);
}
