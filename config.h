/************************************************************
 *            Configuration of UniversalSensor              *
 ************************************************************/

#define VERSION          3.4f
#define LEDpin           LED_BUILTIN

#define HAS_RFM69        true    //is auto detected, if not found, sensor works without RFM69
#define HAS_OLED         true    //is auto detected, if not found, sensor works without OLED (SH1106 or SSD1306)
#define HAS_LIGHTSENSOR  true    //is auto detected, if not found, sensor works without BH1750
#define VCC_MEASURE      true    //if not needed, you can disable it
#define USE_MQTT         true    //MQTT only for ESP8266 (WiFi)
#define USE_HOMEKIT      true    //for homekit IAQ as value from 0 to 5

#define DEBUG            false   //activate debug mode
#define SOFT_SPI         false   //if you need SOFT-SPI, set true

//************* MQTT settings *************
#if USE_MQTT
  char mqtt_server[16] = "000.000.000.000";    //IP of your MQTT server
  char mqtt_port[6]    = "1883";               //port you use

  char * mqtt_clientid = "UniversalSensor01";  //name of mqtt client
  char * mqtt_room     = "Wohnzimmer";         //sensor location

  uint16_t mqtt_interval = 30;                 //send every 30 seconds
#endif
