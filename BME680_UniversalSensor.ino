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
 */

/*!
 * @file bsec_iot_example.ino
 *
 * @brief
 * Example for using of BSEC library in a fixed configuration with the BME680 sensor.
 * This works by running an endless loop in the bsec_iot_loop() function.
 */

/*!
 * @addtogroup bsec_examples BSEC Examples
 * @brief BSEC usage examples
 * @{*/

/***********************************************************************************************************************
 * Wireless sensor for measuring temperature, humidity, pressure, lightlevel and airquality.
 * Hardware: NodeMCU V1.0, RFM69CW, BME680, BH1750 and 1.3 inch 128x64 LCD.
 * Protokoll is UniversalSensor.
 * Frequency is 868.3 MHz (or 433 MHz / 915 MHz based on RFM69CW model).
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
 ************************************************************************************************************************

  Hardware setup:                                                   BME680              RFM69CW
                         NodeMCU                                   +------+            +-------+
                         +--\/--+                            +3.3V | 0x76 |      +3,3V |       | GND
                VCC 3,3V |      | GND                      <-> SDA | or   |   --> MOSI |    NSS| RFM_NSS <--
    int.LED (D0) GPIO 16 |      | GPIO 1  (D10)            --> SCL | 0x77 |   <-- MISO |       |
    <-- SCL (D1)  GPIO 5 |      | GPIO 3  (D9)                 GND |      |   --> SCK  |       |
           RESET     RST |      | GPIO 15 (D8) RFM_NSS -->         +------+            +-------+
    <-> SDA (D2)  GPIO 4 |      | GPIO 13 (D7) MOSI -->             BH1750            SH1106 OLED
            (D3)  GPIO 0 |      | GPIO 12 (D6) MISO <--            +------+            +-------+
            (D4)  GPIO 2 |      | GPIO 14 (D5) SCK  -->      +3.3V | 0x23 |      +3.3V | 0x3C  |
                         +------+                              GND | or   |        GND | or    |
                                                           --> SCL | 0x5C |    --> SCL | 0x3D  |
                                                           <-> SDA |      |    <-> SDA |       |
                                                                   +------+            +-------+

************************************************************************************************************************/
/* defines                                                                                                             */
/***********************************************************************************************************************/
/* select your hardware and options */
#define HAS_RFM69         true   //is auto detected, if not found, sensor works without RFM69, data only on serial port
#define HAS_OLED          true   //is auto detected, if not found, sensor works without OLED
#define HAS_LIGHTSENSOR   true   //is auto detected, if not found, sensor works without BH1750
#define VCC_MEASURE       true   //if not needed, you can disable it here
#define SOFT_SPI         false   //if you need SOFT-SPI, set true
#define DEBUG            false   //activate debug mode
 
/***********************************************************************************************************************/
/* header files                                                                                                        */
/***********************************************************************************************************************/
#include <Wire.h>
#include "bsec_integration.h"
#include "UniversalSensor.h"

#if HAS_RFM69
  #include "RFMxx.h"
  #include "SensorBase.h"
#endif

#if HAS_OLED
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
#endif

#if HAS_LIGHTSENSOR
  #include <AS_BH1750.h>
#endif

/***********************************************************************************************************************/
/* variables                                                                                                           */
/***********************************************************************************************************************/
/* Sensor config */
#define VERSION                 2.2f
#define NODEID                  0x6F       // every node needs his own unique ID 
#define TEMP_CORR               2.3f       // adjust temperature, 2.4f => -2.4 degrees !
#define ALTITUDE               53.0f       // 53.0 meters above sea level
#define LEDpin                 LED_BUILTIN //auto or set pin of your choice

/* RFM69 */
#if HAS_RFM69
  unsigned long DATA_RATE    = 17241ul;    //default data rate (for transmit on RFM69)
  unsigned long INITIAL_FREQ = 868300;     //default frequency in kHz (5 kHz steps, 860480 ... 879515) 
  bool RFM69                 = false;      //if RFM69 is not detected 
  uint8_t loop_count_lim     = 4;          // time to wait between data transmissions (20: 20 * 3sec = 60sec)
  uint8_t loop_counter;
  /* RFM69, PIN-config SPI (GPIO XX or pin Dx): */
  #define RFM_SS               D8          //15   SS pin -> RFM69 (NSS)  //for both Soft- and HW-SPI
  #define RFM_MISO             D6          //12 MISO pin <- RFM69 (MOSI) //only used by soft spi 
  #define RFM_MOSI             D7          //13 MOSI pin -> RFM69 (MISO) //only used by soft spi
  #define RFM_SCK              D5          //14  SCK pin -> RFM69 (SCK)  //only used by soft spi
  RFMxx                        rfm(RFM_MOSI, RFM_MISO, RFM_SCK, RFM_SS, SOFT_SPI);
#endif

/* OLED display */
#if HAS_OLED
  #define OLED_ADR              0x3C       //set I2C adress 0x3C or 0x3D
  #define OLED_ADR_SEC          0x3D
  bool OLED                   = false;     //if display not detected
  Adafruit_SSD1306              display;
#endif

/* Lightsensor  */
#if HAS_LIGHTSENSOR
  bool BH1750                 = false;     //if BH1750 not detected
  AS_BH1750                     bh1750;
#endif

#if VCC_MEASURE
  ADC_MODE(ADC_VCC);                       //esp internal vcc measuring
#endif

/***********************************************************************************************************************/
/* functions */
/***********************************************************************************************************************/
/* blink led */
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

/***********************************************************************************************************************/
/*! 
 * @brief           Write operation in either Wire or SPI
 *
 * param[in]        dev_addr        Wire or SPI device address
 * param[in]        reg_addr        register address
 * param[in]        reg_data_ptr    pointer to the data to be written
 * param[in]        data_len        number of bytes to be written
 *
 * @return          result of the bus communication function
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
 
    return (int8_t)Wire.endTransmission();
}

/***********************************************************************************************************************/
/*!
 * @brief           Read operation in either Wire or SPI
 *
 * param[in]        dev_addr        Wire or SPI device address
 * param[in]        reg_addr        register address
 * param[out]       reg_data_ptr    pointer to the memory to be used to store the read data
 * param[in]        data_len        number of bytes to be read
 *
 * @return          result of the bus communication function
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
 
    return comResult;
}

/***********************************************************************************************************************/
/*!
 * @brief           System specific implementation of sleep function
 *
 * @param[in]       t_ms    time in milliseconds
 *
 * @return          none
 */
void sleep(uint32_t t_ms)
{
    delay(t_ms);
}

/***********************************************************************************************************************/
/*!
 * @brief           Capture the system time in microseconds
 *
 * @return          system_current_time    current system timestamp in microseconds
 */
int64_t get_timestamp_us()
{
    return (int64_t) millis() * 1000;
}

/***********************************************************************************************************************/
/*!
 * @brief           Load previous library state from non-volatile memory
 *
 * @param[in,out]   state_buffer    buffer to hold the loaded state string
 * @param[in]       n_buffer        size of the allocated state buffer
 *
 * @return          number of bytes copied to state_buffer
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

/***********************************************************************************************************************/
/*!
 * @brief           Save library state to non-volatile memory
 *
 * @param[in]       state_buffer    buffer holding the state to be stored
 * @param[in]       length          length of the state string to be stored
 *
 * @return          none
 */
void state_save(const uint8_t *state_buffer, uint32_t length)
{
    // ...
    // Save the string some form of non-volatile memory, if possible.
    // ...
}

/***********************************************************************************************************************/
/*!
 * @brief           Load library config from non-volatile memory
 *
 * @param[in,out]   config_buffer    buffer to hold the loaded state string
 * @param[in]       n_buffer        size of the allocated state buffer
 *
 * @return          number of bytes copied to config_buffer
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

/***********************************************************************************************************************/
/*!
 * @brief           Handling of the ready outputs
 *
 * @param[in]       timestamp       time in nanoseconds
 * @param[in]       iaq             IAQ signal
 * @param[in]       iaq_accuracy    accuracy of IAQ signal
 * @param[in]       temperature     temperature signal
 * @param[in]       humidity        humidity signal
 * @param[in]       pressure        pressure signal
 * @param[in]       raw_temperature raw temperature signal
 * @param[in]       raw_humidity    raw humidity signal
 * @param[in]       gas             raw gas sensor signal
 * @param[in]       bsec_status     value returned by the bsec_do_steps() call
 *
 * @return          none
 */
void output_ready(int64_t timestamp, float iaq, uint8_t iaq_accuracy, float temperature, float humidity, float pressure, float raw_temperature, float raw_humidity, float gas, bsec_library_return_t bsec_status)
{
  /* get battery voltage */
  #if VCC_MEASURE
    float vcc = ESP.getVcc() / 930.9; //correction for 220k/100k voltage divider to A0 on NodeMCU and D1 mini...
  #else  
    float vcc = 0xFF; //0.0;
  #endif

  /* get light level */
  #if HAS_LIGHTSENSOR
    float lux;
    if (BH1750) 
    {
      lux = bh1750.readLightLevel();
      #if HAS_OLED
        if (OLED)
        {
          /* display dimm, if lightlevel < 10lux */
          if (lux < 10)
          {
            display.dim(true);
          }
          else
          {
            display.ssd1306_command(SSD1306_SETCONTRAST);
            display.ssd1306_command(0x80);
          }
        }
      #endif
    }
    else
    {
      lux = 0xFFFFFF; //0.0;
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

  /* Output measurings */
  Serial.print("[");
  Serial.print(timestamp / 1e6);
  Serial.print("] P: ");
  Serial.print(pressure, 1);
  Serial.print("| T: ");
  Serial.print(temperature);
  Serial.print("| rH: ");
  Serial.print(humidity);
  Serial.print("| IAQ: ");
  Serial.print(iaq);
  Serial.print(" (");
  Serial.print(iaq_accuracy);
  Serial.print(")");
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
  
  /* on OLED Display */
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
      display.println("");
    
      display.print("Luftdruck: ");
      display.print(pressure, 0);
      display.println("hPa");

      display.print("Luftguete: ");
      if(iaq_accuracy > 0)
      {
        display.print(iaq, 0);
        display.print(" (");
        display.print(iaq_accuracy);
        display.print(")");
      }
      else
      {
        display.print("berechne..");
      }
      display.println("");

      #if HAS_LIGHTSENSOR
        if (BH1750)
        {
          display.print("Licht    : ");
          display.print(lux, 0);
          display.println("lx");
        }
      #endif
    
      #if VCC_MEASURE
        display.print("Batterie : ");
        display.print(vcc, 1);
        display.println("V");
      #endif

      display.print("Gas      : ");
      display.print(gas / 1000 + 0.005, 2);
      display.println("kOhm");
      display.display();
    }
  #endif

  #if HAS_RFM69
    if (RFM69)
    {
      loop_counter += 1;
      if(loop_counter >= loop_count_lim) //send every (loop_count_lim * 3) seconds 
      {
        loop_counter = 0;

        UniversalSensor::Frame data;
        data.ID = NODEID;
        data.Flags = 0;
        data.Temperature = temperature;
        data.Humidity = humidity;
        data.Pressure = pressure;
        data.Gas1 = iaq;
        data.Gas2 = gas;
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
        
        blink(LEDpin, 1, 25);
      }
    }
  #endif
}

/***********************************************************************************************************************/
/*!
 * @brief       Main function which configures BSEC library and then reads and processes the data from sensor based
 *              on timer ticks
 *
 * @return      result of the processing
 */
void setup()
{
    return_values_init ret;
    
    /* get bsec version */
    bsec_version_t  version;
    bsec_get_version(&version);
    
    pinMode(LEDpin, OUTPUT);
    digitalWrite(LEDpin, HIGH); //LED is low activ !

    #if DEBUG
      #if HAS_RFM69
        rfm.SetDebugMode(true);
      #endif
    #endif
    
    Serial.begin(115200);
    sleep(6000); //some time to open the terminalprogram ;o)
    
    Serial.println("");
    Serial.print("BME680 wireless sensor V");
    Serial.println(VERSION, 1);
    Serial.println("Protocol: UniversalSensor");
    Serial.printf("BSEC version: %d.%d.%d.%d\n",version.major, version.minor, version.major_bugfix, version.minor_bugfix);
    Serial.print("ESP Core Version: ");
    Serial.println(ESP.getCoreVersion());
    Serial.print("ESP SDK Version: ");
    Serial.println(ESP.getSdkVersion());
    Serial.print("NODE-ID : 0x");
    Serial.println(NODEID, HEX);

    #if DEBUG
      Serial.println("aktivierte Komponenten:");
      #if HAS_RFM69
        Serial.println("RFM69 - 433/868/915MHz Sender");
      #endif  
      #if HAS_LIGHTSENSOR
        Serial.println("BH1750 - Lichtsensor");
      #endif
      #if HAS_OLED
        Serial.println("OLED - 128x64 OLED Display mit SH1106 Chipsatz");
      #endif
      Serial.print("Messung der Versorgungsspannung ");
      #if VCC_MEASURE
        Serial.println("aktiv");
      #else
        Serial.println("deaktiviert");
      #endif
    #endif
    
    /* init I2C */
    Wire.begin();
    Wire.setClock(400000); 
    
    #if HAS_OLED
      /* init OLED */
      byte OLED_I2C_ADR;
      Wire.beginTransmission(OLED_ADR);  //first try primary adress
      if (Wire.endTransmission() == 0)
      {
        OLED_I2C_ADR = OLED_ADR;
        OLED = true;
        Serial.println("OLED found on 0x3C");
      } 
      if (!OLED)
      {
        Wire.beginTransmission(OLED_ADR_SEC);  //then try secondary adress
        if (Wire.endTransmission() == 0)
        {
          OLED_I2C_ADR = OLED_ADR_SEC;
          OLED = true;
          Serial.println("OLED found on 0x3D");
        } 
      }
      if (!OLED)
        Serial.println("OLED not found");
      
      if (OLED)
      {
        display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADR); // initialize VCC state, I2C addr 0x3C / 0x3D
        display.setTextSize(2);
        display.setTextColor(WHITE);
        display.setCursor(5,4);
        display.ssd1306_command(SSD1306_SETCONTRAST);
        display.ssd1306_command(0x80);
        display.clearDisplay();

        /* startscreen */
        display.println("  BME680");
        display.display();
        display.setTextSize(1);
        display.println("");
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
        display.print("Node-ID: 0x");
        display.println(NODEID, HEX);
        display.display();
        sleep(5000);
        display.setCursor(0,0);
        display.clearDisplay();
      }
    #endif
    
    /* --- RFM69CW init --- */
    #if HAS_RFM69
      if (rfm.Begin())
      {
        RFM69 = true; //RFM69 is present
        loop_counter = loop_count_lim; //to send data at startup
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
        Serial.println("RFM69 not found, no wireless transmission !");
      }
    #endif
    
    /* --- BME680 init --- */
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

    ret = bsec_iot_init(BSEC_SAMPLE_RATE_LP, TEMP_CORR, bus_write, bus_read, sleep, state_load, config_load);
    
    if (ret.bme680_status)
    {
      /* Could not intialize BME680 */
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
      return; //jump to main loop
    }
    else if (ret.bsec_status)
    {
      /* Could not intialize BSEC library */
      Serial.println("Error while initializing BSEC library !");
      #if HAS_OLED
        if (OLED)
        {
          display.println();
          display.println("BSEC init. Fehler !");
          display.display();
        }
      #endif
      return; //jump to main loop
    }
    
    Serial.println("done");
    #if HAS_OLED
      if (OLED)
      {
        display.println("bereit");
        display.display();
      }
    #endif
    
    /* --- BH1750 init --- */
    #if HAS_LIGHTSENSOR
      Serial.print("BH1750 init ... ");
      #if HAS_OLED
        if (OLED)
        {
          display.print("BH1750 init...");
          display.display();
        }
      #endif
      /* for normal sensor resolution (1 lx resolution, 0-65535 lx, 120ms, no PowerDown) use: bh1750.begin(RESOLUTION_NORMAL, false); */
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

    blink(LEDpin, 3, 250); //setup success
    
    Serial.println("Ready, start measuring ...");
    Serial.println("");
    #if HAS_OLED
      if (OLED)
      {
        display.print("Starte Messungen ...");
        display.display();
        sleep(5000); //time to read display messages
        display.clearDisplay();
        display.display();
      }
    #endif
    
    /* todo ... */
    /* Call to endless loop function which reads and processes data based on sensor settings */
    /* State is saved every 10.000 samples, which means every 10.000 * 3 secs = 500 minutes  */
    bsec_iot_loop(sleep, get_timestamp_us, output_ready, state_save, 10000);
}

/***********************************************************************************************************************/
void loop()
{
  //if an error occured in setup
  do
  {
    blink(LEDpin, 10, 250); //setup not successful
  } 
  while(0);
}

/*! @}*/
