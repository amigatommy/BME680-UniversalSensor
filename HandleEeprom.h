#ifndef _HANDLEEEPROM_h
#define _HANDLEEEPROM_h

#include "Arduino.h"
#include <EEPROM.h>

class HandleEeprom
{

public:
  uint8_t ReadNodeID();
  float ReadAltitude();
  float ReadTempOffset();
  void SaveSettings(uint8_t m_nodeid, float m_altitude, float m_tempoffset);
  void RestoreDefaults();     
  uint8_t EditNodeID(uint8_t m_nodeid);
  float EditAltitude(float m_altitude);
  float EditTempOffset(float m_tempoffset);
  void SetDebugMode(bool mode);
  
private:
  uint8_t m_nodeid;
  float m_altitude;
  float m_tempoffset;
  bool m_debug;
  
  const int startaddr_nodeid = 0;     //address in EEPROM

  const int startaddr_altitude = 1;   //start address in EEPROM
  const uint8_t bytes_altitude = 3;   //number of bytes

  const int startaddr_tempoffset = 4; //start address in EEPROM
  const uint8_t bytes_tempoffset = 2; //number of bytes

  const int startaddr_settings = 0;   //start address in EEPROM
  const uint8_t bytes_settings = 6;   //number of bytes

};

#endif
