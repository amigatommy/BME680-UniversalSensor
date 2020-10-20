# BME680_UniversalSensor

This project is based on my cc_sensor, HCS has made some changes to upgrade it to the UniversalSensor, many thanks HCS !
The main component is an ESP8266 nodemcu, but i dont use the wifi section, for wireless transmission i use a RFM69CW module.
Since V3.0 you can use ESP8266 based NodeMCU/Wemos D1 mini OR STM32F103Cx based BluPill/Maple mini boards !
Hardware setup you can find in the sketch.
The RFM69CW transmit at 868.30 MHz with LaCrosse Weatherstation Protocoll. The signal is decoded by a LaCrosseGateway module (by HCS),
wich transmitt the data to a FHEM server. There is created a LaCrosse like weather sensor with all the sensor data as readings.
Since V3.3 MQTT over WiFi on ESP8266 based clients is possible. MQTT-messages are send every 30 seconds, 
login data (WiFi, MQTT) stored in secrets.h ! The sensor configuration (all other parameters) is stored in config.h.
This sensor gives you the following data:
temperature (degree Celsius), humidity (% rH), air pressure (hPa/mBar), air quality (IAQ, CO2 and VOC) , light intensity (Lux) and battery voltage (V).
You can use an optional 128x64 OLED Display with SH1106/SSD1306 Controller, to display the data. It is autodetected, simply connect it or not.
The Lightsensor (BH1750) is optional too.
If a RFM69CW is not connected, no wireless transmission is possible. The data where transmitted by the serial port only (115200 Baud).
