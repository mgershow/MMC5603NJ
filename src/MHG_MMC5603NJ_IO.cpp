/* library for the mmc5603nj magnetometer - adapted from sparkfun 5983 library, original header follows:
 *
  This is a library written for the MMC5983MA High Performance Magnetometer.
  SparkFun sells these at its website:
  https://www.sparkfun.com/products/19034

  Do you like this library? Help support open source hardware. Buy a board!

  Written by Ricardo Ramos  @ SparkFun Electronics, February 2nd, 2022.
  This file implements all functions used in the MMC5983MA High Performance Magnetometer Arduino Library IO layer.

  SparkFun code, firmware, and software is released under the MIT License(http://opensource.org/licenses/MIT).
  See LICENSE.md for more information.
*/

#include "MHG_MMC5603NJ_IO.h"
#include "MHG_MMC5603NJ_Arduino_Library_Constants.h"




// Read operations must have the most significant bit set
#define READ_REG(x) (0x80 | x)

bool MHG_MMC5603NJ_IO::begin(TwoWire &i2cPort)
{



    _i2cPort = &i2cPort;
   // _i2cPort->setWireTimeout(2000, true); //2 ms
    _i2cPort->setTimeout(2); //in ms, does not have autoreset


    return isConnected();


}


bool MHG_MMC5603NJ_IO::isConnected()
{
    bool result = false;

    _i2cPort->beginTransmission(I2C_ADDR);
    if (_i2cPort->endTransmission() == 0)
    	result = readSingleByte(PROD_ID_REG) == PROD_ID;

    return result;
}

void MHG_MMC5603NJ_IO::writeMultipleBytes(const uint8_t registerAddress, uint8_t *const buffer, uint8_t const packetLength)
{
	_i2cPort->beginTransmission(I2C_ADDR);
	_i2cPort->write(registerAddress);
	for (uint8_t i = 0; i < packetLength; i++)
		_i2cPort->write(buffer[i]);

	_i2cPort->endTransmission();
}

void MHG_MMC5603NJ_IO::readMultipleBytes(const uint8_t registerAddress, uint8_t *const buffer, const uint8_t packetLength)
{

	_i2cPort->beginTransmission(I2C_ADDR);
	_i2cPort->write(registerAddress);
	_i2cPort->endTransmission();

	_i2cPort->requestFrom(I2C_ADDR, packetLength);
	for (uint8_t i = 0; (i < packetLength); i++)
		buffer[i] = _i2cPort->read();

}

uint8_t MHG_MMC5603NJ_IO::readSingleByte(const uint8_t registerAddress)
{
	uint8_t result = 0;

	_i2cPort->beginTransmission(I2C_ADDR);
	_i2cPort->write(registerAddress);
	_i2cPort->endTransmission();
	_i2cPort->requestFrom(I2C_ADDR, 1U);
	result = _i2cPort->read();

	return result;
}

void MHG_MMC5603NJ_IO::writeSingleByte(const uint8_t registerAddress, const uint8_t value)
{
	const int maxtries = 10;

	for (int j = 0; j < maxtries; ++j){

		_i2cPort->beginTransmission(I2C_ADDR);

		_i2cPort->write(registerAddress);
		_i2cPort->write(value);

		if (_i2cPort->endTransmission() == 0) {
			return;
		}
	}


}

void MHG_MMC5603NJ_IO::setRegisterBit(const uint8_t registerAddress, const uint8_t bitMask)
{
    uint8_t value = readSingleByte(registerAddress);
    value |= bitMask;
    writeSingleByte(registerAddress, value);
}

void MHG_MMC5603NJ_IO::clearRegisterBit(const uint8_t registerAddress, const uint8_t bitMask)
{
    uint8_t value = readSingleByte(registerAddress);
    value &= ~bitMask;
    writeSingleByte(registerAddress, value);
}

bool MHG_MMC5603NJ_IO::isBitSet(const uint8_t registerAddress, const uint8_t bitMask)
{
    uint8_t value = readSingleByte(registerAddress);
    return (value & bitMask);
}


