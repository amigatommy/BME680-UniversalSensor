#include "HandleEeprom.h"

uint8_t HandleEeprom::ReadNodeID()
{
  int addr = startaddr_nodeid;
  uint8_t result = 0;  

  result = int(EEPROM.read(addr));
  if (m_debug)
    {
      Serial.println("Node-ID raw data from EEPROM:");
      Serial.print(addr);
      Serial.print(": 0x");
      if (result < 16)
        Serial.print("0");
      Serial.print(result, HEX);
      Serial.print(" (");
      Serial.print(result, DEC);
      Serial.println(")");
      Serial.println();
    }
  return result;
}
 
float HandleEeprom::ReadAltitude()
{
  float result = 0.0;
  
  if (m_debug)
    Serial.println("Altitude raw data from EEPROM:");

  for (int addr = startaddr_altitude; addr < startaddr_altitude + bytes_altitude; addr++)
  {
    byte data = EEPROM.read(addr);
    if (addr == startaddr_altitude)
      result = int(data) << 8;
    if (addr == startaddr_altitude + 1)
      result += int(data);
    if (addr == startaddr_altitude + 2)
      result += int(data) * 0.1;

    if (m_debug)
    {
      Serial.print(addr);
      Serial.print(": 0x");
      if (data < 16)
        Serial.print("0");
      Serial.print(data, HEX);
      Serial.print(" (");
      Serial.print(data, DEC);
      Serial.println(")");
    }
  }
  if (m_debug)
    Serial.println();
  return result;
}
 
float HandleEeprom::ReadTempOffset()
{
  float result = 0.0;

  if (m_debug)
    Serial.println("Temperature Offset raw data from EEPROM:");

  for (int addr = startaddr_tempoffset; addr < startaddr_tempoffset + bytes_tempoffset; addr++)
  {
    byte data = EEPROM.read(addr);
    if (addr == startaddr_tempoffset)
    {
      if (data > 9)   //if value minus
      {
        result = int(256 - data) * -1;
      }
      else
      {
        result = int(data);
      }
      if (m_debug)
      {
        Serial.print(addr);
        Serial.print(": 0x");
        if (data < 16)
          Serial.print("0");
        Serial.print(data, HEX);
        Serial.print(" (");
        Serial.print(data, DEC);
        Serial.println(")");
      }
    }
    if (addr == startaddr_tempoffset + 1)
    {
      if (data > 9) //if value minus
      {
        result += int(256 - data) * -0.1;
      }
      else
      {
        result += int(data) * 0.1;
      }
      if (m_debug)
      {
        Serial.print(addr);
        Serial.print(": 0x");
        if (data < 16)
          Serial.print("0");
        Serial.print(data, HEX);
        Serial.print(" (");
        Serial.print(data, DEC);
        Serial.println(")");
        Serial.println();
      }
    }
  }
  return result;
}
 
void HandleEeprom::SaveSettings(uint8_t m_nodeid, float m_altitude, float m_tempoffset)
{
  byte data;
  
  if (m_debug)
    Serial.println("Raw data saved in eeprom: ");
    
  for (int addr = startaddr_settings; addr < startaddr_settings + bytes_settings; addr++)
  {
    if (addr == startaddr_settings)
      data = m_nodeid;
    if (addr == startaddr_settings + 1)
      data = int(m_altitude) >> 8;           //highbyte
    if (addr == startaddr_settings + 2)
      data = int(m_altitude) & 0xFF;         //lowbyte
    if (addr == startaddr_settings + 3)
      data = int(m_altitude * 10.0) % 10;    //after komma
    if (addr == startaddr_settings + 4)
      data = int(m_tempoffset);              //max. 9
    if (addr == startaddr_settings + 5)
      data = int(m_tempoffset * 10.05) % 10; //after komma

    #ifdef ESP8266
      EEPROM.write(addr, data);
    #elif defined(__STM32F1__)
      EEPROM.update(addr, data);  //write only, if value different, max. 100.000 write cycles for EEPROM !
    #endif
            
    if (m_debug)
    {
      Serial.print(addr);
      Serial.print(": 0x");
      if (data < 16)
        Serial.print("0");
      Serial.print(data, HEX);
      Serial.print(" (");
      Serial.print(data, DEC);
      Serial.println(")");
    }
  }
  #ifdef ESP8266
    EEPROM.commit();
  #endif
  
  if (m_debug)
    Serial.println();
}

void HandleEeprom::RestoreDefaults()
{
  uint64_t Start = millis();
  Serial.println("Do you want to delete settings in EEPROM (Y/N)?");
  do {} while(!Serial.available() && millis() - Start < 5000);
  char input = Serial.read();
  if (input == 'Y' || input == 'y')
  {   
    for (int a = startaddr_settings; a < startaddr_settings + bytes_settings; a++)
    {
      #ifdef ESP8266
        EEPROM.write(a,0xFF);
      #elif defined(__STM32F1__)
        EEPROM.update(a,0xFF);
      #endif
    }
    #ifdef ESP8266
      EEPROM.commit();
    #endif
    Serial.println("Settings deleted.");
    Serial.println();
  }
  else
  {
    Serial.println("Not deleted.");
    Serial.println();
  }
}

uint8_t HandleEeprom::EditNodeID(uint8_t m_nodeid)
{
  int result = 0xFF; //0xFF is empty byte in EEPROM
  bool endinp = false;
  
  Serial.print("Node-ID is: ");
  Serial.print(m_nodeid, DEC);
  Serial.print(" (0x");
  if (m_nodeid < 16)
    Serial.print("0");
  Serial.print(m_nodeid, HEX);
  Serial.println(")");
  Serial.println("New value (1-254): ");

  /*clear serial input buffer*/
  if (Serial.available())
    Serial.read();

  do
  {
    if (Serial.available())
    {
      result = Serial.parseInt();
      if (!(result & 0x8000) && result > 0 && result <= 255) //not minus and in range 1-254
      {
        Serial.print("Node-ID set to: ");
        Serial.print(result, DEC);
        Serial.print(" (0x");
        if (result < 16)
          Serial.print("0");                            
        Serial.print(result, HEX);
        Serial.println(")");
        
        Serial.println("Save permanently (Y/N)?");
        do {} while(!Serial.available());
        char input = Serial.read();
        if (input == 'Y' || input == 'y')
        {   
          endinp = true;
          Serial.println("OK.");
          Serial.println();
        }
        else
        {
          Serial.println("new value (1-254): ");
        }
      }
      else
      {
        Serial.println("Error, try again !");
        Serial.print("received: ");
        Serial.println(result);
      }
    }
  }
  while (!endinp);
  return result;
}

float HandleEeprom::EditAltitude(float m_altitude)
{
  float result = 65560.5; //value for empty bytes in EEPROM
  bool endinp = false;
  
  Serial.print("Altitude is: ");
  Serial.print(m_altitude, 1);
  Serial.println("m");
  Serial.println("new value (0.0-8848.9): ");

  /*clear serial input buffer*/
  if (Serial.available())
    Serial.read();
  
  do
  {
    if (Serial.available())
    {
      result = Serial.parseFloat();
      if (result >= 0.0 && result < 8849.0) //if you live on mount everest ;o)
      {
        Serial.print("Altitude set to: ");
        Serial.print(result, 1);
        Serial.println("m");
        Serial.println("Save permanently (Y/N)?");
        do {} while(!Serial.available());
        char input = Serial.read();
        if (input == 'Y' || input == 'y')
        {   
          endinp = true;
          Serial.println("OK.");
          Serial.println();
        }
        else
        {
          Serial.println("new value (0.0-8848.9): ");
        }
      }
      else
      {
        Serial.println("Error, try again !");
        Serial.print("received: ");
        Serial.println(result);
      }
    }
  }
  while (!endinp);
  return result;
}

float HandleEeprom::EditTempOffset(float m_tempoffset)
{
  float result = -1.1; //value for empty bytes in EEPROM
  bool endinp = false;
  
  Serial.print("Temperature offset: ");
  Serial.print(m_tempoffset, 1);
  Serial.println(" degrees celsius");
  Serial.println("new value (+-10.0): ");
  
  /*clear serial input buffer*/
  if (Serial.available())
    Serial.read();
  
  do
  {
    if (Serial.available())
    {
      result = Serial.parseFloat();
      if (result >= -10.0 && result <= 10.0)
      {
        Serial.print("Temperature offset set to: ");
        Serial.print(result, 1);
        Serial.println(" degrees celsius");
        Serial.println("Save permanently (Y/N)?");
        do {} while(!Serial.available());
        char input = Serial.read();
        if (input == 'Y' || input == 'y')
        {   
          endinp = true;
          Serial.println("OK.");
          Serial.println();
        }
        else
        {
          Serial.println("new value (+-10.0): ");
        }
      }
      else
      {
        Serial.println("Error, try again !");
        Serial.print("received: ");
        Serial.println(result);
      }
    }
  }
  while (!endinp);
  return result;
}

void HandleEeprom::SetDebugMode(bool mode)
{
  m_debug = mode;
}
