/**************************************************************************/
/*!
    @file     Adafruit_ADS1115.cpp
    @author   K.Townsend (Adafruit Industries)

    @mainpage Adafruit ADS1X15 ADC Breakout Driver

    @section intro_sec Introduction

    This is a library for the Adafruit ADS1X15 ADC breakout boards.

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    @section author Author

    Written by Kevin "KTOWN" Townsend for Adafruit Industries.

    @section  HISTORY

    v1.0  - First release
    v1.1  - Added ADS1115 support - W. Earl
    v1.2  - Library customized for ADS1115 and ESP32. Pointer to Wire needs to
   be added to library initialization. This is to make the library more flex for
   ESP32 with 2 I2C. It allows setting custom SDA,SCL Pins - by Tschaban

    @section license License

    BSD license, all text here must be included in any redistribution
*/
/**************************************************************************/
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "Adafruit_ADS1115.h"
#include <Wire.h>

/**************************************************************************/
/*!
    @brief  Writes 16-bits to the specified destination register

    @param reg register address to write to
    @param value value to write to register
*/
/**************************************************************************/
void Adafruit_ADS1115::writeRegister(uint8_t reg, uint16_t value) {
  WirePort->beginTransmission(m_i2cAddress);
  WirePort->write((uint8_t)reg);
  WirePort->write((uint8_t)(value >> 8));
  WirePort->write((uint8_t)(value & 0xFF));
  WirePort->endTransmission();
}

/**************************************************************************/
/*!
    @brief  Read 16-bits from the specified destination register

    @param reg register address to read from

    @return 16 bit register value read
*/
/**************************************************************************/
uint16_t Adafruit_ADS1115::readRegister(uint8_t reg) {
  WirePort->beginTransmission(m_i2cAddress);
  WirePort->write(reg);
  WirePort->endTransmission();
  WirePort->requestFrom(m_i2cAddress, (uint8_t)2);
  return ((WirePort->read() << 8) | WirePort->read());
}

/**************************************************************************/
/*!
    @brief  Instantiates a new ADS1115 class w/appropriate properties

    @param i2cAddress I2C address of device
*/
/**************************************************************************/
Adafruit_ADS1115::Adafruit_ADS1115() {}

/**************************************************************************/
/*!
    @brief  Sets up the HW (reads coefficients values, etc.)
*/
/**************************************************************************/

void Adafruit_ADS1115::begin(TwoWire *_WirePort, uint8_t i2cAddress) {
  WirePort = _WirePort;
  m_i2cAddress = i2cAddress;
  m_conversionDelay = ADS1115_CONVERSIONDELAY;
  m_bitShift = 0;
  m_gain = GAIN_TWOTHIRDS; /* +/- 6.144V range (limited to VDD +0.3V max!) */
}

/**************************************************************************/
/*!
    @brief  Sets the gain and input voltage range

    @param gain gain setting to use
*/
/**************************************************************************/
void Adafruit_ADS1115::setGain(adsGain_t gain) { m_gain = gain; }

/**************************************************************************/
/*!
    @brief  Gets a gain and input voltage range

    @return the gain setting
*/
/**************************************************************************/
adsGain_t Adafruit_ADS1115::getGain() { return m_gain; }

/**************************************************************************/
/*!
    @brief  Gets a single-ended ADC reading from the specified channel

    @param channel ADC channel to read

    @return the ADC reading
*/
/**************************************************************************/
uint16_t Adafruit_ADS1115::readADC_SingleEnded(uint8_t channel) {
  if (channel > 3) {
    return 0;
  }

  // Start with default values
  uint16_t config =
      ADS1115_REG_CONFIG_CQUE_NONE |    // Disable the comparator (default val)
      ADS1115_REG_CONFIG_CLAT_NONLAT |  // Non-latching (default val)
      ADS1115_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
      ADS1115_REG_CONFIG_CMODE_TRAD |   // Traditional comparator (default val)
      ADS1115_REG_CONFIG_DR_1600SPS |   // 1600 samples per second (default)
      ADS1115_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)

  // Set PGA/voltage range
  config |= m_gain;

  // Set single-ended input channel
  switch (channel) {
  case (0):
    config |= ADS1115_REG_CONFIG_MUX_SINGLE_0;
    break;
  case (1):
    config |= ADS1115_REG_CONFIG_MUX_SINGLE_1;
    break;
  case (2):
    config |= ADS1115_REG_CONFIG_MUX_SINGLE_2;
    break;
  case (3):
    config |= ADS1115_REG_CONFIG_MUX_SINGLE_3;
    break;
  }

  // Set 'start single-conversion' bit
  config |= ADS1115_REG_CONFIG_OS_SINGLE;

  // Write config register to the ADC
  writeRegister(ADS1115_REG_POINTER_CONFIG, config);

  // Wait for the conversion to complete
  delay(m_conversionDelay);

  // Read the conversion results
  // Shift 12-bit results right 4 bits for the ADS1115
  return readRegister(ADS1115_REG_POINTER_CONVERT) >> m_bitShift;
}

/**************************************************************************/
/*!
    @brief  Reads the conversion results, measuring the voltage
            difference between the P (AIN0) and N (AIN1) input.  Generates
            a signed value since the difference can be either
            positive or negative.

    @return the ADC reading
*/
/**************************************************************************/
int16_t Adafruit_ADS1115::readADC_Differential_0_1() {
  // Start with default values
  uint16_t config =
      ADS1115_REG_CONFIG_CQUE_NONE |    // Disable the comparator (default val)
      ADS1115_REG_CONFIG_CLAT_NONLAT |  // Non-latching (default val)
      ADS1115_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
      ADS1115_REG_CONFIG_CMODE_TRAD |   // Traditional comparator (default val)
      ADS1115_REG_CONFIG_DR_1600SPS |   // 1600 samples per second (default)
      ADS1115_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)

  // Set PGA/voltage range
  config |= m_gain;

  // Set channels
  config |= ADS1115_REG_CONFIG_MUX_DIFF_0_1; // AIN0 = P, AIN1 = N

  // Set 'start single-conversion' bit
  config |= ADS1115_REG_CONFIG_OS_SINGLE;

  // Write config register to the ADC
  writeRegister(ADS1115_REG_POINTER_CONFIG, config);

  // Wait for the conversion to complete
  delay(m_conversionDelay);

  // Read the conversion results
  uint16_t res = readRegister(ADS1115_REG_POINTER_CONVERT) >> m_bitShift;
  if (m_bitShift == 0) {
    return (int16_t)res;
  } else {
    // Shift 12-bit results right 4 bits for the ADS1115,
    // making sure we keep the sign bit intact
    if (res > 0x07FF) {
      // negative number - extend the sign to 16th bit
      res |= 0xF000;
    }
    return (int16_t)res;
  }
}

/**************************************************************************/
/*!
    @brief  Reads the conversion results, measuring the voltage
            difference between the P (AIN2) and N (AIN3) input.  Generates
            a signed value since the difference can be either
            positive or negative.

    @return the ADC reading
*/
/**************************************************************************/
int16_t Adafruit_ADS1115::readADC_Differential_2_3() {
  // Start with default values
  uint16_t config =
      ADS1115_REG_CONFIG_CQUE_NONE |    // Disable the comparator (default val)
      ADS1115_REG_CONFIG_CLAT_NONLAT |  // Non-latching (default val)
      ADS1115_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
      ADS1115_REG_CONFIG_CMODE_TRAD |   // Traditional comparator (default val)
      ADS1115_REG_CONFIG_DR_1600SPS |   // 1600 samples per second (default)
      ADS1115_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)

  // Set PGA/voltage range
  config |= m_gain;

  // Set channels
  config |= ADS1115_REG_CONFIG_MUX_DIFF_2_3; // AIN2 = P, AIN3 = N

  // Set 'start single-conversion' bit
  config |= ADS1115_REG_CONFIG_OS_SINGLE;

  // Write config register to the ADC
  writeRegister(ADS1115_REG_POINTER_CONFIG, config);

  // Wait for the conversion to complete
  delay(m_conversionDelay);

  // Read the conversion results
  uint16_t res = readRegister(ADS1115_REG_POINTER_CONVERT) >> m_bitShift;
  if (m_bitShift == 0) {
    return (int16_t)res;
  } else {
    // Shift 12-bit results right 4 bits for the ADS1115,
    // making sure we keep the sign bit intact
    if (res > 0x07FF) {
      // negative number - extend the sign to 16th bit
      res |= 0xF000;
    }
    return (int16_t)res;
  }
}

/**************************************************************************/
/*!
    @brief  Sets up the comparator to operate in basic mode, causing the
            ALERT/RDY pin to assert (go from high to low) when the ADC
            value exceeds the specified threshold.

            This will also set the ADC in continuous conversion mode.

    @param channel ADC channel to use
    @param threshold comparator threshold
*/
/**************************************************************************/
void Adafruit_ADS1115::startComparator_SingleEnded(uint8_t channel,
                                                   int16_t threshold) {
  // Start with default values
  uint16_t config =
      ADS1115_REG_CONFIG_CQUE_1CONV |   // Comparator enabled and asserts on 1
                                        // match
      ADS1115_REG_CONFIG_CLAT_LATCH |   // Latching mode
      ADS1115_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
      ADS1115_REG_CONFIG_CMODE_TRAD |   // Traditional comparator (default val)
      ADS1115_REG_CONFIG_DR_1600SPS |   // 1600 samples per second (default)
      ADS1115_REG_CONFIG_MODE_CONTIN |  // Continuous conversion mode
      ADS1115_REG_CONFIG_MODE_CONTIN;   // Continuous conversion mode

  // Set PGA/voltage range
  config |= m_gain;

  // Set single-ended input channel
  switch (channel) {
  case (0):
    config |= ADS1115_REG_CONFIG_MUX_SINGLE_0;
    break;
  case (1):
    config |= ADS1115_REG_CONFIG_MUX_SINGLE_1;
    break;
  case (2):
    config |= ADS1115_REG_CONFIG_MUX_SINGLE_2;
    break;
  case (3):
    config |= ADS1115_REG_CONFIG_MUX_SINGLE_3;
    break;
  }

  // Set the high threshold register
  // Shift 12-bit results left 4 bits for the ADS1115
  writeRegister(ADS1115_REG_POINTER_HITHRESH, threshold << m_bitShift);

  // Write config register to the ADC
  writeRegister(ADS1115_REG_POINTER_CONFIG, config);
}

/**************************************************************************/
/*!
    @brief  In order to clear the comparator, we need to read the
            conversion results.  This function reads the last conversion
            results without changing the config value.

    @return the last ADC reading
*/
/**************************************************************************/
int16_t Adafruit_ADS1115::getLastConversionResults() {
  // Wait for the conversion to complete
  delay(m_conversionDelay);

  // Read the conversion results
  uint16_t res = readRegister(ADS1115_REG_POINTER_CONVERT) >> m_bitShift;
  if (m_bitShift == 0) {
    return (int16_t)res;
  } else {
    // Shift 12-bit results right 4 bits for the ADS1115,
    // making sure we keep the sign bit intact
    if (res > 0x07FF) {
      // negative number - extend the sign to 16th bit
      res |= 0xF000;
    }
    return (int16_t)res;
  }
}
