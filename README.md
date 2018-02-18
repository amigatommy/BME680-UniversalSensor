# BME680_UniversalSensor

This project is based on my cc_sensor, HCS has made some changes to upgrade it to the UniversalSensor, many thanks to HCS !
The main component is an ESP8266 nodemcu, but i dont use the wifi section, wireless transmission is done by an RFM69CW module.
Since V3.0 you can use ESP8266 based NodeMCU/Wemos D1 mini OR STM32F103Cx based BluPill/Maple mini boards !
Hardware setup you can find in the sketch.
The RFM69 transmit at 868.30 MHz with LaCrosse Weatherstation Protocoll. The signal is decoded by a LaCrosseGateway module (by HCS),
wich transmitt the data to a FHEM server. There is created a LaCrosse like weather sensor with all the sensor data as readings.
This sensor gives you the following data:
temerature (degree Celsius), humidity (% rH), air pressure (hPa/mBar), air quality, light intensity (Lux) and battery voltage (V).
You can use an optional 128x64 OLED Display with SH1106/SSD1306 Controller, to display the data. It is autodetected, simply connect it or not.
The Lightsensor (BH1750) is optional too, it not need to be connected.
The RFM69CW not need to be connected too, in this case, no wireless transmission is possible. The data where transmitted
by the serial port only (115200 Baud).
