/**************************************************************************/
/*!
    @file     TCS34725.cpp
    @author   KTOWN (Adafruit Industries)
    @license  BSD (see license.txt)

    Driver for the TCS34725 digital color sensors.

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    @section  HISTORY

    v1.0 - First release

    Modifications by Leo Linbeck III to use SoftI2CMaster library
    to allow multiple sensors to be used with the same I2C address
    by using different SCL and SDA pins for each sensor
*/
/**************************************************************************/
//#include <avr/pgmspace.h>
//#include <util/delay.h>
#include "application.h"
#include <stdlib.h>
#include <math.h>

#include "SoftI2CMaster.h"
#include "TCS34725.h"

/*========================================================================*/
/*                          PRIVATE FUNCTIONS                             */
/*========================================================================*/

/**************************************************************************/
/*!
    @brief  Implements missing powf function
*/
/**************************************************************************/
float powf(const float x, const float y)
{
  return (float)(pow((double)x, (double)y));
}

/**************************************************************************/
/*!
    @brief  Writes a register and an 8 bit value over I2C
*/
/**************************************************************************/
void TCS34725::write8 (uint8_t reg, uint8_t value)
{
  _i2c.beginWriteTransmission(TCS34725_ADDRESS, (uint8_t) TCS34725_COMMAND_BIT | reg);
  _i2c.write(value);
  _i2c.endTransmission();
}

/**************************************************************************/
/*!
    @brief  Reads an 8 bit value over I2C
*/
/**************************************************************************/
uint8_t TCS34725::read8(uint8_t reg)
{
  uint16_t res;

  _i2c.beginReadTransmission(TCS34725_ADDRESS, (uint8_t) TCS34725_COMMAND_BIT | reg);
  res = _i2c.readLast();
  _i2c.endTransmission();
  return res;
}

/**************************************************************************/
/*!
    @brief  Reads a 16 bit values over I2C
*/
/**************************************************************************/
uint16_t TCS34725::read16(uint8_t reg)
{
  uint16_t x; uint16_t t;

  _i2c.beginReadTransmission(TCS34725_ADDRESS, (uint8_t) TCS34725_COMMAND_BIT | reg);
  t = _i2c.read();
  x = _i2c.readLast();
  _i2c.endTransmission();

  x <<= 8;
  x |= t;
  return x;
}

/**************************************************************************/
/*!
    Enables the device
*/
/**************************************************************************/
void TCS34725::enable(void)
{
  write8(TCS34725_ENABLE, TCS34725_ENABLE_PON);
  delay(3);
  write8(TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);
}

/**************************************************************************/
/*!
    Disables the device (putting it in lower power sleep mode)
*/
/**************************************************************************/
void TCS34725::disable(void)
{
  /* Turn the device off to save power */
  uint8_t reg = 0;
  reg = read8(TCS34725_ENABLE);
  write8(TCS34725_ENABLE, reg & ~(TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN));
}

/*========================================================================*/
/*                            CONSTRUCTORS                                */
/*========================================================================*/

/**************************************************************************/
/*!
    Constructor
*/
/**************************************************************************/
TCS34725::TCS34725() {

}

TCS34725::TCS34725(tcs34725IntegrationTime_t it, tcs34725Gain_t gain, uint8_t sdaPin, uint8_t sclPin)
{
  _tcs34725Initialised = false;
  _tcs34725IntegrationTime = it;
  _tcs34725Gain = gain;
  _sdaPin = sdaPin;
  _sclPin = sclPin;
  _i2c = SoftI2CMaster(sdaPin, sclPin);
}

/*========================================================================*/
/*                           PUBLIC FUNCTIONS                             */
/*========================================================================*/

/**************************************************************************/
/*!
    Initializes I2C and configures the sensor (call this function before
    doing anything else)
*/
/**************************************************************************/
bool TCS34725::begin(void)
{
  /* Make sure we're actually connected */
  uint8_t x = read8(TCS34725_ID);
  if (x != 0x44)
  {
  	Serial.print("Error initializing sensor: ");
  	Serial.println(x, HEX);
    _tcs34725Initialised = false;
    return false;
  }
  else {
    _tcs34725Initialised = true;

    /* Set default integration time and gain */
    setIntegrationTime(_tcs34725IntegrationTime);
    setGain(_tcs34725Gain);

    /* Note: by default, the device is in power down mode on bootup */
    enable();
    return true;
  }
}

/**************************************************************************/
/*!
    Sets the integration time for the TC34725
*/
/**************************************************************************/
void TCS34725::setIntegrationTime(tcs34725IntegrationTime_t it)
{
  if (!_tcs34725Initialised) begin();

  /* Update the timing register */
  write8(TCS34725_ATIME, it);

  /* Update value placeholders */
  _tcs34725IntegrationTime = it;
}

/**************************************************************************/
/*!
    Adjusts the gain on the TCS34725 (adjusts the sensitivity to light)
*/
/**************************************************************************/
void TCS34725::setGain(tcs34725Gain_t gain)
{
  if (!_tcs34725Initialised) begin();

  /* Update the timing register */
  write8(TCS34725_CONTROL, gain);

  /* Update value placeholders */
  _tcs34725Gain = gain;
}

/**************************************************************************/
/*!
    @brief  Reads the raw red, green, blue and clear channel values
*/
/**************************************************************************/
void TCS34725::getRawData (uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
  if (!_tcs34725Initialised) begin();

  *c = read16(TCS34725_CDATAL);
  *r = read16(TCS34725_RDATAL);
  *g = read16(TCS34725_GDATAL);
  *b = read16(TCS34725_BDATAL);

  /* Set a delay for the integration time */
  switch (_tcs34725IntegrationTime)
  {
    case TCS34725_INTEGRATIONTIME_2_4MS:
      delay(3);
      break;
    case TCS34725_INTEGRATIONTIME_24MS:
      delay(24);
      break;
    case TCS34725_INTEGRATIONTIME_50MS:
      delay(50);
      break;
    case TCS34725_INTEGRATIONTIME_101MS:
      delay(101);
      break;
    case TCS34725_INTEGRATIONTIME_154MS:
      delay(154);
      break;
    case TCS34725_INTEGRATIONTIME_700MS:
      delay(700);
      break;
  }
}

/**************************************************************************/
/*!
    @brief  Converts the raw R/G/B values to color temperature in degrees
            Kelvin
*/
/**************************************************************************/
uint16_t TCS34725::calculateColorTemperature(uint16_t r, uint16_t g, uint16_t b)
{
  float X, Y, Z;      /* RGB to XYZ correlation      */
  float xc, yc;       /* Chromaticity co-ordinates   */
  float n;            /* McCamy's formula            */
  float cct;

  /* 1. Map RGB values to their XYZ counterparts.    */
  /* Based on 6500K fluorescent, 3000K fluorescent   */
  /* and 60W incandescent values for a wide range.   */
  /* Note: Y = Illuminance or lux                    */
  X = (-0.14282F * r) + (1.54924F * g) + (-0.95641F * b);
  Y = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);
  Z = (-0.68202F * r) + (0.77073F * g) + ( 0.56332F * b);

  /* 2. Calculate the chromaticity co-ordinates      */
  xc = (X) / (X + Y + Z);
  yc = (Y) / (X + Y + Z);

  /* 3. Use McCamy's formula to determine the CCT    */
  n = (xc - 0.3320F) / (0.1858F - yc);

  /* Calculate the final CCT */
  cct = (449.0F * powf(n, 3)) + (3525.0F * powf(n, 2)) + (6823.3F * n) + 5520.33F;

  /* Return the results in degrees Kelvin */
  return (uint16_t)cct;
}

/**************************************************************************/
/*!
    @brief  Converts the raw R/G/B values to color temperature in degrees
            Kelvin
*/
/**************************************************************************/
uint16_t TCS34725::calculateLux(uint16_t r, uint16_t g, uint16_t b)
{
  float illuminance;

  /* This only uses RGB ... how can we integrate clear or calculate lux */
  /* based exclusively on clear since this might be more reliable?      */
  illuminance = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);

  return (uint16_t)illuminance;
}


void TCS34725::setInterrupt(bool flag) {
//   uint8_t r = read8(TCS34725_ENABLE);
//   if (flag) {
//     r |= TCS34725_ENABLE_AIEN;
//   } else {
//     r &= ~TCS34725_ENABLE_AIEN;
//   }
//   write8(TCS34725_ENABLE, r);
  if (flag) {
     write8(TCS34725_ENABLE, 0x11);
  	 delay(10);
     write8(TCS34725_ENABLE, 0x13);
  }
  else {
     write8(TCS34725_ENABLE, 0x01);
  	 delay(10);
     write8(TCS34725_ENABLE, 0x03);
  }
}

void TCS34725::startReading(bool useLED) {
  uint8_t r;

  if (useLED) {
	 do {
     	write8(TCS34725_ENABLE, 0x01);
  	 	delay(10);
     	write8(TCS34725_ENABLE, 0x03);
	 	r = read8(TCS34725_ENABLE);
	 } while (r != 0x03);
  }
  else {
	 do {
     	write8(TCS34725_ENABLE, 0x11);
  	 	delay(10);
     	write8(TCS34725_ENABLE, 0x13);
	 	r = read8(TCS34725_ENABLE);
	 } while (r != 0x13);
  }
}

void TCS34725::endReading(void) {
	uint8_t r;

	do {
		write8(TCS34725_ENABLE, 0x11);
	 	r = read8(TCS34725_ENABLE);
	} while (r != 0x11);
}

void TCS34725::clearInterrupt(void) {
	write8(TCS34725_ADDRESS, 0x66);
}


void TCS34725::setIntLimits(uint16_t low, uint16_t high) {
   write8(0x04, low & 0xFF);
   write8(0x05, low >> 8);
   write8(0x06, high & 0xFF);
   write8(0x07, high >> 8);
}
