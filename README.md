# BME680_UniversalSensor

This project is based on my cc_sensor, HCS has made some changes to upgrade it to the UniversalSensor, many thanks to HCS !
The main component is an ESP8266 nodemcu, but i dont use the wifi section, wireless transmission is done by an RFM69CW module.
I use 868.30 MHz. The signal is decoded by a LaCrosseGateway module (by HCS), wich transmitt the data to a FHEM server.
There is created a LaCrosse like weather sensor with all the sensor data as readings.
This sensor gives you the following data:
temerature (degree Celsius), humidity (% rH), air pressure (hPa/mBar), air quality, light intensity (Lux) and battery voltage (V).
You can use an optional 0.96" OLED Display with SH1106 Controller, to display the data. It is autodetected, simply connect it or not.
The Lightsensor (BH1750) is also optional, it can but not must be connected. If it is not present, the data for light are not used.
And at last, the RFM69CW must also not be connected, in this case, no wireless transmission is possible. The data where transmitted
by the serial port only (115200 Baud).
Instead of the nodemcu you can use similar boards, like WEMOS D1 mini.
Hardware setup you can find in the sketch.
