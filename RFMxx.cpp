#include "RFMxx.h"

/* constructor */
RFMxx::RFMxx(byte mosi, byte miso, byte sck, byte ss, bool soft_spi) {
  m_mosi = mosi;
  m_miso = miso;
  m_sck = sck;
  m_ss = ss;
  m_soft_spi = soft_spi;
  
  m_debug = false;
  m_dataRate = 17241;
  m_frequency = 868300;
  m_payloadPointer = 0;
  m_lastReceiveTime = 0;
  m_payloadReady = false;
  m_radioType = RFMxx::None;

  pinMode(m_ss, OUTPUT);
  digitalWrite(m_ss, HIGH);
  if (m_soft_spi)
  {
    pinMode(m_mosi, OUTPUT);
    pinMode(m_miso, INPUT);
    pinMode(m_sck, OUTPUT);
  }
}

bool RFMxx::Begin() {
  // No radio found until now
  m_radioType = RFMxx::None;
  
  if (!m_soft_spi)
  {
    SPI.begin();
    #ifdef ESP8266    
      SPI.setFrequency(400000); //SPI clock 400kHz (because of many crc errors with higher clock speeds)
    #elif defined(__STM32F1__)
      SPI.setClockDivider(SPI_CLOCK_DIV128); //72 / 128 = 562.5kHz SPI_1 speed
    #endif    

    if (m_debug)
    {
      Serial.println("Hardware-SPI fuer RFM69 aktiviert");
      #ifdef ESP8266
        Serial.print("NSS an GPIO-Pin ");Serial.println(m_ss);
      #elif defined(__STM32F1__)
        Serial.println("SPI_1 Takt: 562.50kHz");
        Serial.print("NSS  an Pin PA");Serial.println(m_ss);
        Serial.print("SCK  an Pin PA");Serial.println(m_sck);
        Serial.print("MISO an Pin PA");Serial.println(m_miso);
        Serial.print("MOSI an Pin PA");Serial.println(m_mosi);
      #endif
    }
  }
  else
  {
    if (m_debug)
    {
      Serial.println("Soft-SPI fuer RFM69 aktiviert");
      #ifdef ESP8266
        Serial.print("MOSI an GPIO-Pin ");Serial.println(m_mosi);
        Serial.print("MISO an GPIO-Pin ");Serial.println(m_miso);
        Serial.print("SCK  an GPIO-Pin ");Serial.println(m_sck);
        Serial.print("NSS  an GPIO-Pin ");Serial.println(m_ss);
      #elif defined(__STM32F1__)
        Serial.print("NSS  an Pin PA");Serial.println(m_ss);
        Serial.print("SCK  an Pin PA");Serial.println(m_sck);
        Serial.print("MISO an Pin PA");Serial.println(m_miso);
        Serial.print("MOSI an Pin PA");Serial.println(m_mosi);
      #endif
    }
  }
  
  // Is there a RFM69 ?
  WriteReg(REG_PAYLOADLENGTH, 0xA);
  if (ReadReg(REG_PAYLOADLENGTH) == 0xA)
  {
    WriteReg(REG_PAYLOADLENGTH, 0x40);
    if (ReadReg(REG_PAYLOADLENGTH) == 0x40)
    {
      m_radioType = RFMxx::RFM69;
    }
  }

  if (m_radioType != RFMxx::None) {
    EnableReceiver(false);
    return true;
  }
  return false;
}

void RFMxx::SetDebugMode(boolean mode) {
  m_debug = mode;
}

void RFMxx::Receive() {
  if (ReadReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY) {
    for (int i = 0; i < PAYLOADSIZE; i++) {
      byte bt = GetByteFromFifo();
      m_payload[i] = bt;
    }
    m_payloadReady = true;
  }
}

void RFMxx::GetPayload(byte *data) {
  m_payloadReady = false;
  m_payloadPointer = 0;
  for (int i = 0; i < PAYLOADSIZE; i++) {
    data[i] = m_payload[i];
  }
}

void RFMxx::SetDataRate(unsigned long dataRate) {
  m_dataRate = dataRate;

  word r = ((32000000UL + (m_dataRate / 2)) / m_dataRate);
  WriteReg(0x03, r >> 8);
  WriteReg(0x04, r & 0xFF);
}

void RFMxx::SetFrequency(unsigned long kHz) {
  m_frequency = kHz;

  unsigned long f = (((kHz * 1000) << 2) / (32000000L >> 11)) << 6;
  WriteReg(0x07, f >> 16);
  WriteReg(0x08, f >> 8);
  WriteReg(0x09, f);
}

void RFMxx::EnableReceiver(bool enable){
  if (enable) {
    WriteReg(REG_OPMODE, (ReadReg(REG_OPMODE) & 0xE3) | RF_OPMODE_RECEIVER);
  }
  else {
    WriteReg(REG_OPMODE, (ReadReg(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY);
  }
  ClearFifo();
}

void RFMxx::EnableTransmitter(bool enable){
  if (enable) {
    WriteReg(REG_OPMODE, (ReadReg(REG_OPMODE) & 0xE3) | RF_OPMODE_TRANSMITTER);
  }
  else {
    WriteReg(REG_OPMODE, (ReadReg(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY);
  }
}

byte RFMxx::GetByteFromFifo() {
  return ReadReg(0x00);
}

bool RFMxx::PayloadIsReady() {
  return m_payloadReady;
}

void RFMxx::ClearFifo() {
  WriteReg(REG_IRQFLAGS2, 16);
}

void RFMxx::PowerDown(){
  WriteReg(REG_OPMODE, (ReadReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SLEEP);
}

void RFMxx::InitializeLaCrosse() {
    /* 0x01 */ WriteReg(REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY);
    /* 0x02 */ WriteReg(REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00);
    /* 0x05 */ WriteReg(REG_FDEVMSB, RF_FDEVMSB_90000);
    /* 0x06 */ WriteReg(REG_FDEVLSB, RF_FDEVLSB_90000);
    /* 0x11 */ WriteReg(REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | RF_PALEVEL_OUTPUTPOWER_11111);
    /* 0x12 */ WriteReg(REG_PARAMP,RF_PARAMP_10);
    /* 0x13 */ WriteReg(REG_OCP, RF_OCP_OFF);
    /* 0x19 */ WriteReg(REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_2);
    /* 0x28 */ WriteReg(REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN);
    /* 0x29 */ WriteReg(REG_RSSITHRESH, 220);
    /* 0x2E */ WriteReg(REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_0);
    /* 0x2F */ WriteReg(REG_SYNCVALUE1, 0x2D);
    /* 0x30 */ WriteReg(REG_SYNCVALUE2, 0xD4);
    /* 0x37 */ WriteReg(REG_PACKETCONFIG1, RF_PACKET1_CRCAUTOCLEAR_OFF);
    /* 0x38 */ WriteReg(REG_PAYLOADLENGTH, PAYLOADSIZE);
    /* 0x3C */ WriteReg(REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE);
    /* 0x3D */ WriteReg(REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF);
    /* 0x6F */ WriteReg(REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0);
  ClearFifo();
}

void RFMxx::InitializePCA301() {
  /*
     PCA      LGW
     0x94C5                 // RX: LNA Gain Max / Pin VDI / Bandwidth 67kHz  / VDI FAST / DRSSI -73dBm
              0x94a0        // RX: LNA Gain Max / Pin VDI / Bandwidth 134kHz / VDI FAST / DRSSI -103dBm
     
     0xCA83                 // FIFO: INT Level 8 / Sync 2 Byte / FillStart=Sync / Sens low  /  Enabled
              0xCA12        // FIFO: INT Level 1 / Sync 2 Byte / FillStart=Sync / Sens high / Enabled
              
     0xC477                 // AFC: Enabled / once after power up  / Limit +3..-4         / High Accuracy     / Enable  frequenct offset register / no strobe
              0xC481        // AFC: Enabled / only during VDI=high / Limit no restriction / NO High Accuracy  / Disable frequenct offset register / no strobe
              
     0xC2AF                 // Filter Digital / Recovery Auto    / Quality Tresh. 7 / Recovery Slow
              0xC26a        // Filter Digital / Recovery Manuell / Quality Tresh. 0 / Recovery Fast
     
     
  */
  
  WriteReg(REG_FDEVMSB, RF_FDEVMSB_45000);
  WriteReg(REG_FDEVLSB, RF_FDEVLSB_45000);
  WriteReg(REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_OUTPUTPOWER_10110);

  SetFrequency(868950);
  SetDataRate(6631);
}

void RFMxx::InitializeEC3000() {
    /* 0x01 */ WriteReg(REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY);
    /* 0x02 */ WriteReg(REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00);
    /* 0x05 */ WriteReg(REG_FDEVMSB, RF_FDEVMSB_20000);
    /* 0x06 */ WriteReg(REG_FDEVLSB, RF_FDEVLSB_20000);
    /* 0x11 */ WriteReg(REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | RF_PALEVEL_OUTPUTPOWER_11111);
    /* 0x13 */ WriteReg(REG_OCP, RF_OCP_OFF);
    /* 0x18 */ WriteReg(REG_LNA, RF_LNA_GAINSELECT_MAX | RF_LNA_ZIN_200);
    /* 0x19 */ WriteReg(REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_2);
    /* 0x28 */ WriteReg(REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN);
    /* 0x29 */ WriteReg(REG_RSSITHRESH, 220);
    /* 0x2E */ WriteReg(REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_5 | RF_SYNC_TOL_0);
    /* 0x2F */ WriteReg(REG_SYNCVALUE1, 0x13);
    /* 0x30 */ WriteReg(REG_SYNCVALUE2, 0xF1);
    /* 0x31 */ WriteReg(REG_SYNCVALUE3, 0x85);
    /* 0x32 */ WriteReg(REG_SYNCVALUE4, 0xD3);
    /* 0x33 */ WriteReg(REG_SYNCVALUE5, 0xAC);
    /* 0x37 */ WriteReg(REG_PACKETCONFIG1, RF_PACKET1_CRCAUTOCLEAR_OFF);
    /* 0x38 */ WriteReg(REG_PAYLOADLENGTH, PAYLOADSIZE);
    /* 0x3C */ WriteReg(REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE);
    /* 0x3D */ WriteReg(REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF);
    /* 0x6F */ WriteReg(REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0);
}

void RFMxx::InitializeElero() {
  WriteReg(REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY);
  WriteReg(REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00);
  WriteReg(REG_FDEVMSB, RF_FDEVLSB_35000);
  WriteReg(REG_FDEVLSB, RF_FDEVLSB_35000);
  WriteReg(REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | RF_PALEVEL_OUTPUTPOWER_11111);
  WriteReg(REG_OCP, RF_OCP_OFF);
  WriteReg(REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_EXP_3);
  WriteReg(REG_AFCBW, RF_AFCBW_DCCFREQAFC_010 | RF_AFCBW_EXPAFC_2);
  WriteReg(REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN);
  WriteReg(REG_RSSITHRESH, 220);
  WriteReg(REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_SIZE_2);
  WriteReg(REG_SYNCVALUE1, 0xD3);
  WriteReg(REG_SYNCVALUE2, 0x91);
  WriteReg(REG_PREAMBLELSB, 0xAA);
  WriteReg(REG_PREAMBLEMSB, 0xAA);
  WriteReg(REG_PACKETCONFIG1, RF_PACKET1_CRCAUTOCLEAR_OFF);
  WriteReg(REG_PAYLOADLENGTH, PAYLOADSIZE);
  WriteReg(REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE);
  WriteReg(REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF);
  WriteReg(REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0);

  SetFrequency(869525);
  SetDataRate(76767);
}

int RFMxx::GetRSSI(bool doTrigger) {
  int rssi = -1024;
    if (doTrigger) {
      WriteReg(REG_RSSICONFIG, RF_RSSI_START);
      unsigned long to = millis() + 100;
      while ((ReadReg(REG_RSSICONFIG) & RF_RSSI_DONE) == 0x00 && millis() < to);
    }
    rssi = -ReadReg(REG_RSSIVALUE);
    rssi >>= 1;
  
  return rssi;
}

// soft SPI Interface 8 bit
byte RFMxx::spi8(byte value) {
  for (byte i = 8; i; i--) {
    digitalWrite(m_sck, LOW);
    if (value & 0x80) {
      digitalWrite(m_mosi, HIGH);
    }
    else {
      digitalWrite(m_mosi, LOW);
    }
    value <<= 1;
    digitalWrite(m_sck, HIGH);
    if (digitalRead(m_miso)) {
      value |= 1;
    }
  }
  digitalWrite(m_sck, LOW);
  return value;
}

// soft SPI Interface 16 bit
unsigned short RFMxx::spi16(unsigned short value) {  
  for (byte i = 0; i < 16; i++) {
    if (value & 32768) {
      digitalWrite(m_mosi, HIGH);
    }
    else {
      digitalWrite(m_mosi, LOW);
    }
    value <<= 1;
    if (digitalRead(m_miso)) {
      value |= 1;
    }
    digitalWrite(m_sck, HIGH);
    delayMicroseconds(1);
    digitalWrite(m_sck, LOW);
  }
  return value;
}

byte RFMxx::ReadReg(byte addr) {
  byte result;
  digitalWrite(m_ss, LOW);
  if (!m_soft_spi)
  {
    SPI.transfer(addr & 0x7F);
    result = SPI.transfer(0);
  }
  else  
  {
    spi8(addr & 0x7F);
    result = spi8(0);
  }
  digitalWrite(m_ss, HIGH);
  return result;
}

void RFMxx::WriteReg(byte addr, byte value) {
  digitalWrite(m_ss, LOW);
  if (!m_soft_spi)
  {
    SPI.transfer(addr | 0x80);
    SPI.transfer(value);
  }
  else
  {
    spi8(addr | 0x80);
    spi8(value);
  }
  digitalWrite(m_ss, HIGH);
}

RFMxx::RadioType RFMxx::GetRadioType() {
  return m_radioType;
}

String RFMxx::GetRadioName() {
  switch (GetRadioType()) {
  case RFMxx::RFM69:
    return String("RFM69");
    break;
  default:
    return String("None");
  }
}

bool RFMxx::IsConnected() {
  return m_radioType != RFMxx::None;
}

//

unsigned long RFMxx::GetDataRate() {
  return m_dataRate;
}

unsigned long RFMxx::GetFrequency() {
  return m_frequency;
}

void RFMxx::SendArray(byte *data, byte length) {
  WriteReg(REG_PACKETCONFIG2, (ReadReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks
  EnableReceiver(false);
  ClearFifo();
  noInterrupts();
  digitalWrite(m_ss, LOW);
  if (!m_soft_spi)
  {
    SPI.transfer(REG_FIFO | 0x80);
  }
  else
  {
    spi8(REG_FIFO | 0x80);
  }
  for (byte i = 0; i < length; i++) {
    if (!m_soft_spi)
    {
      SPI.transfer(data[i]);
    }
    else
    {
      spi8(data[i]);
    }
  }

  digitalWrite(m_ss, HIGH);
  interrupts();

  EnableTransmitter(true);

  // Wait until transmission is finished
  unsigned long txStart = millis();
  while (!(ReadReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PACKETSENT) && millis() - txStart < 50);
  if (m_debug) {
    int t = millis() - txStart;
    Serial.print("transmission time: ");Serial.print(t);Serial.println(" ms");
  }
  EnableTransmitter(false);
  
  if (m_debug) {
    Serial.println("data: ");
    for (int p = 0; p < length; p++) {
      Serial.print(data[p], DEC);
      Serial.print(" ");
    }
    Serial.println();
    for (int p = 0; p < length; p++) {
      Serial.print(data[p], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
}
