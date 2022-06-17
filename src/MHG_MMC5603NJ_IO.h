/* This is a library for the MMC5603NJ magnetometer - it is adapted from Sparkfun_MMC5983MA
 * original .h and license follows
 *
  This is a library written for the MMC5983MA High Performance Magnetometer.
  SparkFun sells these at its website:
  https://www.sparkfun.com/products/19034

  Do you like this library? Help support open source hardware. Buy a board!

  Written by Ricardo Ramos  @ SparkFun Electronics, February 2nd, 2022.
  This file declares all functions used in the MMC5983MA High Performance Magnetometer Arduino Library I2C/SPI IO layer.

  SparkFun code, firmware, and software is released under the MIT License(http://opensource.org/licenses/MIT).
  See LICENSE.md for more information.
*/

/* modified to remove serial port; SPI not available on the 5603*/

#ifndef _MHG_MMC5603NJ_IO_
#define _MHG_MMC5603NJ_IO_

#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>
class MHG_MMC5603NJ_IO
{
private:
    TwoWire *_i2cPort = nullptr;

public:
    // Default empty constructor.
    MHG_MMC5603NJ_IO() = default;

    // Default empty destructor
    ~MHG_MMC5603NJ_IO() = default;

    // Configures and starts the I2C I/O layer.
    bool begin(TwoWire &wirePort);


    // Returns true if we get the correct product ID from the device.
    bool isConnected();

    // Read a single uint8_t from a register.
    uint8_t readSingleByte(const uint8_t registerAddress);

    // Writes a single uint8_t into a register.
    void writeSingleByte(const uint8_t registerAddress, const uint8_t value);

    // Reads multiple bytes from a register into buffer uint8_t array.
    void readMultipleBytes(const uint8_t registerAddress, uint8_t* const buffer , const uint8_t packetLength);

    // Writes multiple bytes to register from buffer uint8_t array.
    void writeMultipleBytes(const uint8_t registerAddress, uint8_t* const buffer, const uint8_t packetLength);

    // Sets a single bit in a specific register. Bit position ranges from 0 (lsb) to 7 (msb).
    void setRegisterBit(const uint8_t registerAddress, const uint8_t bitMask);

    // Clears a single bit in a specific register. Bit position ranges from 0 (lsb) to 7 (msb).
    void clearRegisterBit(const uint8_t registerAddress, const uint8_t bitMask);

    // Returns true if a specific bit is set in a register. Bit position ranges from 0 (lsb) to 7 (msb).
    bool isBitSet(const uint8_t  registerAddress, const uint8_t bitMask);

};

#endif
