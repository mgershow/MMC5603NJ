/* array of magnetometers
 *
   released under the MIT License(http://opensource.org/licenses/MIT).
  See LICENSE.md for more information.
*/

#ifndef _MHG_MMC5603NJ_Array
#define _MHG_MMC5603NJ_Array

#include <Arduino.h>
#include <Wire.h>
#include "MHG_MMC5603NJ_Arduino_Library.h"
#include "MHG_MMC5603NJ_IO.h"
#include "MHG_MMC5603NJ_Arduino_Library_Constants.h"

#define MAX_SENSORS 8

typedef struct
{
  uint64_t us;
  float x[MAX_SENSORS];
  float y[MAX_SENSORS];
  float z[MAX_SENSORS];
  bool sensorOnline[MAX_SENSORS];
} multiMagMeasurementT;

class MHG_MMC5603NJ_Array
{
private:

  uint8_t sel0pin = 1;
  uint8_t sel1pin = 2;
  uint8_t sel2pin = 3;
  uint8_t nsensors = 8;
  bool readAllBits = false;

  // sensors
   MHG_MMC5603NJ mmc[MAX_SENSORS];
   bool sensorOnline[MAX_SENSORS];

   uint8_t currentsensor = 0;
   multiMagMeasurementT currentMeasurement;
   bool measurementFinished = false;
public:



  // Default constructor.
  MHG_MMC5603NJ_Array() = default;

  // Default destructor.
  ~MHG_MMC5603NJ_Array() = default;

  // Sets the error callback function.
  void setErrorCallback(void (*errorCallback)(MHG_MMC5603NJ_ERROR errorCode));

  // Initializes MMC5603NJ using I2C, does soft reset
  bool begin(TwoWire &wirePort = Wire, uint8_t nsensors = 8);

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

  // Sets decimation filter bandwidth.
  // Allowed values are 0 - measurement time = 6.6 ms
  // 					1 - measurement time = 3.5 ms
  //					2 - measurement time = 2.0 ms
  //					3 - measurement time = 1.2 ms
  void setFilterBandwidth(uint8_t bandwidth);

  // Enables continuous mode. Continuous mode frequency must be greater than 0.
  void enableContinuousMode();

  // Disables continuous mode.
  void disableContinuousMode();

  // Sets continuous mode frequency. Allowed values are 1-255, 1000 and 0 (off). Defaults to 0 (off).
  void setContinuousModeFrequency(uint16_t frequency);

  // Enables periodic set
  void enablePeriodicSet();

  // Disables periodic set
  void disablePeriodicSet();

  // Sets how often the chip will perform an automatic set operation. Allowed values are 1, 25, 75, 100, 250, 500, 1000, 2000. Defaults to 1.
  void setPeriodicSetSamples(uint16_t numberOfSamples);

  //sets the i2c channel for the selected mmc and returns a pointer to it
  MHG_MMC5603NJ *getMMC(uint8_t ind);

  void requestMagMeasurement(uint64_t timeInUs);
  multiMagMeasurementT getMeasurement();
  bool isMeasurementReady();
  void initMeasurementCycle(uint64_t timeInUs);

  uint8_t measurementCycle(uint64_t timeInUs, bool &dataready, multiMagMeasurementT &measurement);

};

#endif
