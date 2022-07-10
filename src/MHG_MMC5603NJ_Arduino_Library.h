/* library for the mmc5603nj magnetometer - adapted from sparkfun 5983 library, original header follows:
 *
  This is a library written for the MMC5983MA High Performance Magnetometer.
  SparkFun sells these at its website:
  https://www.sparkfun.com/products/19034

  Do you like this library? Help support open source hardware. Buy a board!

  Written by Ricardo Ramos  @ SparkFun Electronics, February 2nd, 2022.
  This file declares all functions used in the MMC5983MA High Performance Magnetometer Arduino Library.

  SparkFun code, firmware, and software is released under the MIT License(http://opensource.org/licenses/MIT).
  See LICENSE.md for more information.
*/

#ifndef _MHG_MMC5603NJ_
#define _MHG_MMC5603NJ_

#include <Arduino.h>
#include <Wire.h>
#include "MHG_MMC5603NJ_IO.h"
#include "MHG_MMC5603NJ_Arduino_Library_Constants.h"

class MHG_MMC5603NJ
{
private:
  // I2C communication object instance.
  MHG_MMC5603NJ_IO mmc_io;
  // Error callback function pointer.
  // Function must accept a MHG_MMC5603NJ_ERROR as errorCode.
  void (*errorCallback)(MHG_MMC5603NJ_ERROR errorCode) = nullptr;

  // Since some registers are write-only in MMC5983MA all operations
  // are done in shadow memory locations. Default reset values are
  // set to the shadow memory locations upon initialization and after
  // any bit set in the shadow location the register is atomically written.
  struct MemoryShadow
  {
    uint8_t internalControl0 = 0x0;
    uint8_t internalControl1 = 0x0;
    uint8_t internalControl2 = 0x0;
    uint8_t odr = 0x0;
  } memoryShadow;

  //rewrites entire register with shadow
  void writeShadowRegister(uint8_t registerAddress, const uint8_t value);

  //sets or clears bit depending on whether val is true or false
  void setShadowBitVal(uint8_t registerAddress, const uint8_t bitMask, const bool val);

  // Sets register bit(s) on memory shadows and then registers
  void setShadowBit(uint8_t registerAddress, const uint8_t bitMask);

  //sets register bits on device but does not update shadow register
  void setShadowBitSelfClearing(uint8_t registerAddress, const uint8_t bitMask);

  // Clears register bit(s) on memory shadows and then registers
  void clearShadowBit(uint8_t registerAddress, const uint8_t bitMask);

  //reads entire register
  uint8_t readShadowRegister(uint8_t registerAddress);

  // Checks if a specific bit is set on a register memory shadow
  bool isShadowBitSet(uint8_t registerAddress, const uint8_t bitMask);

public:
  // Default constructor.
  MHG_MMC5603NJ() = default;

  // Default destructor.
  ~MHG_MMC5603NJ() = default;

  // Sets the error callback function.
  void setErrorCallback(void (*errorCallback)(MHG_MMC5603NJ_ERROR errorCode));

  // Initializes MMC5983MA using I2C
  bool begin(TwoWire &wirePort = Wire);


  // Polls if MMC5983MA is connected and if chip ID matches MMC5603NJ chip id.
  bool isConnected();

  //returns false if self test passes 
  bool didSelfTestFail();
  
  // Returns die temperature. Range is -75C to 125C.
  int getTemperature();

  // Soft resets the device.
  void softReset(bool waitForReset = true);

  // Performs SET operation
  void performSetOperation();

  // Performs RESET operation
  void performResetOperation();

  // Enables automatic SET/RESET
  void enableAutomaticSetReset();

  // Disables automatic SET/RESET
  void disableAutomaticSetReset();

  // Checks if automatic SET/RESET is enabled
  bool isAutomaticSetResetEnabled();

  // Enables X channel output
  void enableXChannel();

  // Disables X channel output
  void disableXChannel();

  // Checks if X channel output is enabled
  bool isXChannelEnabled();

  // Enables Y and Z channels outputs
  void enableYZChannels();

  // Disables Y and Z channels outputs
  void disableYZChannels();

  // Checks if YZ channels outputs are enabled
  bool areYZChannelsEnabled();

  // Sets decimation filter bandwidth.
  // Allowed values are 0 - measurement time = 6.6 ms
  // 					1 - measurement time = 3.5 ms
  //					2 - measurement time = 2.0 ms
  //					3 - measurement time = 1.2 ms
  void setFilterBandwidth(uint8_t bandwidth);

  // Gets current decimation filter bandwith. Values are above.
  uint8_t getFilterBandwith();

  // Enables continuous mode. Continuous mode frequency must be greater than 0.
  void enableContinuousMode();

  // Disables continuous mode.
  void disableContinuousMode();

  // Checks if continuous mode is enabled.
  bool isContinuousModeEnabled();

  // Sets continuous mode frequency. Allowed values are 1-255, 1000 and 0 (off). Defaults to 0 (off).
  void setContinuousModeFrequency(uint16_t frequency);

  // Gets continuous mode frequency.
  uint16_t getContinuousModeFrequency();

  // Enables periodic set
  void enablePeriodicSet();

  // Disables periodic set
  void disablePeriodicSet();

  // Checks if periodic set is enabled
  bool isPeriodicSetEnabled();  

  // Sets how often the chip will perform an automatic set operation. Allowed values are 1, 25, 75, 100, 250, 500, 1000, 2000. Defaults to 1.
  void setPeriodicSetSamples(uint16_t numberOfSamples);

  // Gets how many times the chip is performing an automatic set operation.
  uint16_t getPeriodicSetSamples();

//  // Get X axis measurement
//  uint32_t getMeasurementX();
//
//  // Get Y axis measurement
//  uint32_t getMeasurementY();
//
//  // Get Z axis measurement
//  uint32_t getMeasurementZ();

  // Get all 3 measurements;
  //if readAllBits is true, reads 20 bits, if false, reads 16 bits (per axis)

  void requestMagMeasurement();
  void readMeasurementXYZ(float &x, float &y, float &z, bool readAllBits = false);

  void getMeasurementXYZ(float &x, float &y, float &z, bool readAllBits = false);

  //sets BW to most precise value that supports selected ODR/auto_sr options
  void autoSetBW();

  bool isMeasurementReady(uint16_t delay_us = 0, uint16_t maxreps = 0);
};

#endif
