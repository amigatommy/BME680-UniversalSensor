/*
 This is a (Arduino) library for the BH1750FVI Digital Light Sensor.
 
 Description:
 http://www.rohm.com/web/global/products/-/product/BH1750FVI
 
 Datasheet:
 http://rohmfs.rohm.com/en/products/databook/datasheet/ic/sensor/light/bh1750fvi-e.pdf
 
 Copyright (c) 2013 Alexander Schulz.  All right reserved.
 
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the GNU
 Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA	 02110-1301	 USA
 
 edit: auto detect sensor on primary or secondary I2C adress, 13.01.2018, t.hirte@gmx.de
 */

#include "AS_BH1750.h"

// Debug-Flag
#define BH1750_DEBUG 0

/**
 * Constructor.
 * Erlaubt die I2C-Adresse des Sensors zu ändern (NEU: automatische Erkennung, siehe "NEU: ..." am Ende dieses Textes). 
 * Standardadresse: 0x23, Alternativadresse: 0x5C. 
 * Es sind entsprechende Konstanten definiert: BH1750_DEFAULT_I2CADDR  und BH1750_SECOND_I2CADDR.
 * Bei Nichtangabe wird die Standardadresse verwendet. 
 * Um die Alternativadresse zu nutzen, muss der Sensorpin 'ADR' des Chips auf VCC gelegt werden.
 * NEU: In der Funktion AS_BH1750::begin wird als Erstes automatisch geprüft, ob ein Sensor auf
 * der primaeren oder sekundaeren Adresse vorhanden ist.
 */
AS_BH1750::AS_BH1750() {
  _address = BH1750_DEFAULT_I2CADDR;
  _hardwareMode = 255;
}

/**
 * Führt die anfängliche Initialisierung des Sensors.
 * Mögliche Parameter: 
 *  - Modus für die Sensorauflösung:
 *    -- RESOLUTION_LOW:         Physische Sensormodus mit 4 lx Auflösung. Messzeit ca. 16ms. Bereich 0-54612.
 *    -- RESOLUTION_NORMAL:      Physische Sensormodus mit 1 lx Auflösung. Messzeit ca. 120ms. Bereich 0-54612.
 *    -- RESOLUTION_HIGH:        Physische Sensormodus mit 0,5 lx Auflösung. Messzeit ca. 120ms. Bereich 0-54612.
 *                               (Die Messbereiche können durch Änderung des MTreg verschoben werden.)
 *    -- RESOLUTION_AUTO_HIGH:   Die Werte im MTreg werden je nach Helligkeit automatisch so angepasst, 
 *                               dass eine maximalmögliche Auflösung und Messbereich erziehlt werden.
 *                               Die messbaren Werte fangen von 0,11 lx und gehen bis über 100000 lx.
 *                               (ich weis nicht, wie genau die Werte in Grenzbereichen sind, 
 *                               besonders bei hohen Werte habe ich da meine Zweifel.
 *                               Die Werte scheinen jedoch weitgehend linear mit der steigenden Helligkeit zu wachsen.)
 *                               Auflösung im Unteren Bereich ca. 0,13 lx, im mittleren 0,5 lx, im oberen etwa 1-2 lx.
 *                               Die Messzeiten verlängern sich durch mehrfache Messungen und 
 *                               die Änderungen von Measurement Time (MTreg) bis max. ca. 500 ms.
 *   
 * - AutoPowerDown: true = Der Sensor wird nach der Messung in den Stromsparmodus versetzt. 
 *   Das spätere Aufwecken wird ggf. automatisch vorgenommen, braucht jedoch geringfügig mehr Zeit.
 *
 * Defaultwerte: RESOLUTION_AUTO_HIGH, true
 *
 */
bool AS_BH1750::begin(sensors_resolution_t mode, bool autoPowerDown) {
#if BH1750_DEBUG == 1
  Serial.print("sensors_resolution_mode (virtual): ");
  Serial.println(mode, DEC);
#endif
  _virtualMode = mode;
  _autoPowerDown = autoPowerDown;
  
  #ifndef __STM32F1__ //second call of Wire.begin cause STM to hang 
    Wire.begin();
  #endif
    
  /* Automatische Erkennung auf primaerer/sekundaerer I2C Adresse */
  #if BH1750_DEBUG == 1
    Serial.println("BH1750 init on standard I2C adress ...");
  #endif
  if(!isPresent()) {                  //Wenn sich kein Sensor auf der Standardadresse meldet,
    _address = BH1750_SECOND_I2CADDR;
    #if BH1750_DEBUG == 1
      Serial.print("BH1750 not found on 0x");
      Serial.print(BH1750_DEFAULT_I2CADDR, HEX);
      Serial.print(", try 0x");
      Serial.print(BH1750_SECOND_I2CADDR, HEX);
    #endif
    if(!isPresent()) {                //dann auf der zweiten Adresse testen.
      #if BH1750_DEBUG == 1
        Serial.println(", sensor not found");
      #endif
    return false;
    }
  }
  #if BH1750_DEBUG == 1
    Serial.println("BH1750 success !");
  #endif
  
  defineMTReg(BH1750_MTREG_DEFAULT); // eigentlich normalerweise unnötig, da standard

  // Hardware-Modus zum gewünschten VitrualModus ermitteltn
  switch (_virtualMode) {
  case RESOLUTION_LOW:
    _hardwareMode = autoPowerDown?BH1750_ONE_TIME_LOW_RES_MODE:BH1750_CONTINUOUS_LOW_RES_MODE;
    break;
  case RESOLUTION_NORMAL:
    _hardwareMode = autoPowerDown?BH1750_ONE_TIME_HIGH_RES_MODE:BH1750_CONTINUOUS_HIGH_RES_MODE;
    break;
  case RESOLUTION_HIGH:
    _hardwareMode = autoPowerDown?BH1750_ONE_TIME_HIGH_RES_MODE_2:BH1750_CONTINUOUS_HIGH_RES_MODE_2;
    break;
  case RESOLUTION_AUTO_HIGH:
    _hardwareMode = BH1750_CONTINUOUS_LOW_RES_MODE;
    break;
  default:
    // darf eigentlich nicht passieren...
    _hardwareMode = 255;
    break;
  }

#if BH1750_DEBUG == 1
  Serial.print("BH1750 hardware mode: ");
  Serial.println(_hardwareMode, DEC);
#endif

  if(_hardwareMode==255) {
    return false;
  }

  // Versuchen, den gewählten Hardwaremodus zu aktivieren
  if(selectResolutionMode(_hardwareMode)){
#if BH1750_DEBUG == 1
    Serial.print("BH1750 hardware mode defined successfully ");
    Serial.println(_hardwareMode, DEC);
#endif
    return true;
  }

  // Initialisierung fehlgeschlagen
#if BH1750_DEBUG == 1
  Serial.print("BH1750 failed to aktivate hardware mode ");
  Serial.println(_hardwareMode, DEC);
#endif
  _hardwareMode = 255;
  return false;
}

/**
 * Erlaub eine Prüfung, ob ein (ansprechbarer) BH1750-Sensor vorhanden ist.
 */
bool AS_BH1750::isPresent() {
  // Check I2C Adresse
  Wire.beginTransmission(_address);
  if(Wire.endTransmission()!=0) {
    return false; 
  }

  // Check device: ist es ein BH1750
  if(!isInitialized()) {
    // zuvor inaktiv, daher zu Testen schnelltes einmal-Mode aktivieren
    //write8(BH1750_POWER_ON);
    selectResolutionMode(BH1750_ONE_TIME_LOW_RES_MODE);
    _hardwareMode=255;
  } 
  else {
    // falls einmal-modus aktiv war, muss der Sensor geweckt werden
    powerOn(); 
  }

  // Prüfen, ob Werte auch wirklich geliefert werden (letztes Modus, ggf. wird auto-PowerDown ausgeführt)
  return true; //return (readLightLevel()>=0);
}

/**
 * Weckt ein im PowerDown-Modus befindlichen Sensor auf (schadet auch dem 'wachen' Sensor nicht).
 * Funktionier nur, wenn der Sensor bereits initialisiert wurde.
 */
void AS_BH1750::powerOn() {
  if(!isInitialized()) {
#if BH1750_DEBUG == 1
    Serial.println("BH1750 sensor not initialized");
#endif
    return;
  }

  _valueReaded=false;
  //write8(BH1750_POWER_ON); //
  //fDelayPtr(10); // Nötig?
  // Scheinbar reicht das Setzen von HardwareMode auch ohne PowerON-Befehl aus
  selectResolutionMode(_hardwareMode); // letzten Modus wieder aktivieren
}

/**
 * Schickt den Sensor in Stromsparmodus.
 * Funktionier nur, wenn der Sensor bereits initialisiert wurde.
 */
void AS_BH1750::powerDown() {
  if(!isInitialized()) {
#if BH1750_DEBUG == 1
    Serial.println("BH1750 sensor not initialized");
#endif
    return;
  }

  write8(BH1750_POWER_DOWN);
}

/**
 * Sendet zum Sensor ein Befehl zum Auswahl von HardwareMode.
 *
 * Parameter:
 * - mode: s.o. 
 * - DelayFuncPtr: delay(n) Möglichkeit, eigene Delay-Funktion mitzugeben (z.B. um sleep-Modus zu verwenden).
 * 
 * Defaultwerte: delay()
 *
 */
bool AS_BH1750::selectResolutionMode(uint8_t mode, DelayFuncPtr fDelayPtr) {
#if BH1750_DEBUG == 1
    Serial.print("BH1750 selectResolutionMode: ");
    Serial.println(mode, DEC);
#endif
  if(!isInitialized()) {
#if BH1750_DEBUG == 1
    Serial.println("BH1750 sensor not initialized");
#endif
    return false;
  }

  _hardwareMode=mode;
  _valueReaded=false;

  // Prüfen, ob ein valides Modus vorliegt und im positiven Fall das gewünschte Modus aktivieren
  switch (mode) {
  case BH1750_CONTINUOUS_HIGH_RES_MODE:
  case BH1750_CONTINUOUS_HIGH_RES_MODE_2:
  case BH1750_CONTINUOUS_LOW_RES_MODE:
  case BH1750_ONE_TIME_HIGH_RES_MODE:
  case BH1750_ONE_TIME_HIGH_RES_MODE_2:
  case BH1750_ONE_TIME_LOW_RES_MODE:
    // Modus aktivieren
    if(write8(mode)) {
    // Kurze pause nötig, sonst wird das Modus nicht sicher Aktiviert
    // (z.B. liefert Sensor im AutoHigh Modus abwechselnd übersteuerte Werte, etwa so: 54612.5, 68123.4, 54612.5, 69345.3,..)
    fDelayPtr(5);
      return true;
    }
    break;
  default:
    // Invalid measurement mode
#if BH1750_DEBUG == 1
    Serial.println("BH1750 Invalid measurement mode");
#endif
    break;
  }

  return false;
}

/**
 * Liefert aktuell gemessenen Wert für Helligkeit in lux (lx).
 * Falls sich der Sensorf in Stromsparmodus befindet, wird er automatisch geweckt.
 *
 * Wurde der Sensor (noch) nicht initialisiert (begin), wird der Wert -1 geliefert.
 */
float AS_BH1750::readLightLevel(DelayFuncPtr fDelayPtr) {
#if BH1750_DEBUG == 1
    Serial.print("BH1750 call: readLightLevel. virtualMode: ");
    Serial.println(_virtualMode, DEC);
#endif

  if(!isInitialized()) {
#if BH1750_DEBUG == 1
    Serial.println("BH1750 sensor not initialized");
#endif
    return -1;
  }

  // ggf. PowerOn  
  if(_autoPowerDown && _valueReaded){
    powerOn();
  }

  // Das Automatische Modus benötigt eine Sonderbehandlung.
  // Zuerst wird die Helligkeit im LowRes-Modus gelesen, 
  // je nach Bereich (dunkel, normal, sehr hell) werden die Werte von MTreg gesetzt und
  // danach wird die eigentliche Messung vorgenommen.
  /*
     Die feste Grenzwerte verursachen möglicherweise einen 'Sprung' in der Messkurve. 
   In diesem Fall wäre eine laufende Anpassung von MTreg in Grenzbereichen vermutlich besser.
   Für meine Zwecke ist das jedoch ohne Bedeutung.
   */
  if(_virtualMode==RESOLUTION_AUTO_HIGH) {
    defineMTReg(BH1750_MTREG_DEFAULT);
    selectResolutionMode(BH1750_CONTINUOUS_LOW_RES_MODE, fDelayPtr);
    fDelayPtr(16); // Lesezeit in LowResMode
    uint16_t level = readRawLevel();
#if BH1750_DEBUG == 1
    Serial.print("BH1750 AutoHighMode: check level read: ");
    Serial.println(level, DEC);
#endif
    if(level<10) {
#if BH1750_DEBUG == 1
    Serial.println("BH1750 level 0: dark");
#endif    
      // Dunkel, Empfindlichkeit auf Maximum. 
      // Der Wert ist zufällig. Ab ca. 16000 wäre diese Vorgehnsweise möglich.
      // Ich brauche diese Genauigkeit aber nur in den ganz dunklen Bereichen (zu erkennen, wann wirklich 'dunkel' ist).
      defineMTReg(BH1750_MTREG_MAX);
      selectResolutionMode(_autoPowerDown?BH1750_ONE_TIME_HIGH_RES_MODE_2:BH1750_CONTINUOUS_HIGH_RES_MODE_2, fDelayPtr);
      fDelayPtr(120*3.68); // TODO: Wert prüfen
      //fDelayPtr(122);
    }
    else if(level<32767) {
#if BH1750_DEBUG == 1
    Serial.println("level 1: normal");
#endif    
      // Bis hierher reicht die 0,5 lx Modus. Normale Empfindlichkeit.
      defineMTReg(BH1750_MTREG_DEFAULT);
      selectResolutionMode(_autoPowerDown?BH1750_ONE_TIME_HIGH_RES_MODE_2:BH1750_CONTINUOUS_HIGH_RES_MODE_2, fDelayPtr);
      fDelayPtr(120); // TODO: Wert prüfen
    } 
    else if(level<60000) {
#if BH1750_DEBUG == 1
    Serial.println("level 2: bright");
#endif    
      // hoher Bereich, 1 lx Modus, normale Empfindlichkeit. Der Wert von 60000 ist mehr oder weniger zufällig, es mus einfach ein hoher Wert, nah an der Grenze sein.
      defineMTReg(BH1750_MTREG_DEFAULT);
      selectResolutionMode(_autoPowerDown?BH1750_ONE_TIME_HIGH_RES_MODE:BH1750_CONTINUOUS_HIGH_RES_MODE, fDelayPtr);
      fDelayPtr(120); // TODO: Wert prüfen
    }
    else {
#if BH1750_DEBUG == 1
    Serial.println("level 3: very bright");
#endif    
      // sehr hoher Bereich, Empfindlichkeit verringern
      defineMTReg(32); // Min+1, bei dem Minimum aus Doku spielt der Sensor (zumindest meiner) verrückt: Die Werte sind ca. 1/10 von den Erwarteten.
      selectResolutionMode(_autoPowerDown?BH1750_ONE_TIME_HIGH_RES_MODE:BH1750_CONTINUOUS_HIGH_RES_MODE, fDelayPtr);
      fDelayPtr(120); // TODO: Wert prüfen
    }
  } 

  // Hardware Wert lesen und in Lux umrechnen.
  uint16_t raw = readRawLevel();
  if(raw==65535) {
    // Wert verdächtig hoch. Sensor prüfen. 
    // Check I2C Adresse
    Wire.beginTransmission(_address);
    if(Wire.endTransmission()!=0) {
      return -1; 
    }
  }
  return convertRawValue(raw); 
}

/**
 * Roh-Wert der Helligkeit auslesen. 
 * Wertebereich 0-65535.
 */
uint16_t AS_BH1750::readRawLevel(void) {
  uint16_t level;
  Wire.beginTransmission(_address);
  Wire.requestFrom(_address, 2);
#if (ARDUINO >= 100)
  level = Wire.read();
  level <<= 8;
  level |= Wire.read();
#else
  level = Wire.receive();
  level <<= 8;
  level |= Wire.receive();
#endif

#ifndef __STM32F1__ //returnvalue on STM32 I2C lib is 3 = nack !  
  if(Wire.endTransmission()!=0) {
    #if BH1750_DEBUG == 1
      Serial.println("BH1750 I2C read error");
    #endif
    return 65535; // Error marker
  }
#endif

#if BH1750_DEBUG == 1
  Serial.print("BH1750 Raw light level: ");
  Serial.println(level);
#endif

  _valueReaded=true;

  return level;
}

/**
 * Rechnet Roh-Werte in Lux um.
 */
float AS_BH1750::convertRawValue(uint16_t raw) {
  // Basisumrechnung
  float flevel = raw/1.2;

#if BH1750_DEBUG == 1
  Serial.print("convert light level (1): ");
  Serial.println(flevel);
#endif

  // MTreg-Einfluss berechnen
  if(_MTreg!=BH1750_MTREG_DEFAULT) { // bei 69 ist der Faktor = 1
    flevel = flevel * _MTregFactor;
#if BH1750_DEBUG == 1
    Serial.print("convert light level (2): ");
    Serial.println(flevel);
#endif
  }

  // je nach Modus ist eine weitere Umrechnung nötig
  switch (_hardwareMode) {
  case BH1750_CONTINUOUS_HIGH_RES_MODE_2:
  case BH1750_ONE_TIME_HIGH_RES_MODE_2:
    flevel = flevel/2;
#if BH1750_DEBUG == 1
    Serial.print("convert light level (3): ");
    Serial.println(flevel);
#endif
    break;
  default:
    // nothing to do
    break;
  }

#if BH1750_DEBUG == 1
  Serial.print("Light level: ");
  Serial.println(flevel);
#endif

  return flevel;
}

/**
 * MTreg setzen. Definiert Empfindlichkeit des Sensors.
 * Min.Wert (BH1750_MTREG_MIN) =  31 (Empfindlichkeit: default*0,45)
 * Max.Wert (BH1750_MTREG_MAX) = 254 (Empfindlichkeit: default*3,68)
 * Default (BH1750_MTREG_DEFAULT) = 69.
 * Mit der Empfindlichkeit verändert sich die Lesezeit (höhere Empfindlichkeit bedeutet längere Zeitspanne).
 */
void AS_BH1750::defineMTReg(uint8_t val) {
  if(val<BH1750_MTREG_MIN) {
    val = BH1750_MTREG_MIN;
  }
  if(val>BH1750_MTREG_MAX) {
    val = BH1750_MTREG_MAX;
  }
  if(val!=_MTreg) {
    _MTreg = val;
    _MTregFactor = (float)BH1750_MTREG_DEFAULT / _MTreg;

    // Change Measurement time
    // Übertragung in zwei Schritten: 3 Bit und 5 Bit, jeweils mit einem Prefix.
    //   High bit: 01000_MT[7,6,5]
    //   Low bit:  011_MT[4,3,2,1,0]
    uint8_t hiByte = val>>5;
    hiByte |= 0b01000000;
#if BH1750_DEBUG == 1
    Serial.print("MGTreg high byte: ");
    Serial.println(hiByte, BIN);
#endif
    write8(hiByte);
    //fDelayPtr(10);
    // Pause nötig?
    uint8_t loByte = val&0b00011111;
    loByte |= 0b01100000;
#if BH1750_DEBUG == 1
    Serial.print("MGTreg low byte: ");
    Serial.println(loByte, BIN);
#endif
    write8(hiByte);
    //fDelayPtr(10);
  }
}

/**
 * Gibt an, ob der Sensor initialisiert ist.
 */
bool AS_BH1750::isInitialized() {
  return _hardwareMode!=255; 
}

/**
 * Schreibt ein Byte auf I2C Bus (auf die Adresse des Sensors).
 */
bool AS_BH1750::write8(uint8_t d) {
  Wire.beginTransmission(_address);
#if (ARDUINO >= 100)
  Wire.write(d);
#else
  Wire.send(d);
#endif
  return (Wire.endTransmission()==0);
}


