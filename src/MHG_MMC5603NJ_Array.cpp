/* library for the mmc5603nj magnetometer - adapted from sparkfun 5983 library, original header follows:
 *
 *
  This is a library written for the MMC5983MA High Performance Magnetometer.
  SparkFun sells these at its website:
  https://www.sparkfun.com/products/19034

  Do you like this library? Help support open source hardware. Buy a board!

  Written by Ricardo Ramos  @ SparkFun Electronics, February 2nd, 2022.
  This file implements all functions used in the MMC5983MA High Performance Magnetometer Arduino Library.

  SparkFun code, firmware, and software is released under the MIT License(http://opensource.org/licenses/MIT).
  See LICENSE.md for more information.
*/

#include "MHG_MMC5603NJ_Array.h"
#include "MHG_MMC5603NJ_Arduino_Library.h"
#include "MHG_MMC5603NJ_Arduino_Library_Constants.h"

//const uint8_t indicatorPins[8] = {16, 17, 18, 19, 20, 21, 22, 12};

bool MHG_MMC5603NJ_Array:: begin(TwoWire &wirePort, uint8_t nsensors)
{

	bool success = true;
	this->nsensors = nsensors;

	for (int j = 0; j < nsensors; ++j) {
		sensorOnline[j] = getMMC(j)->begin(wirePort);
		success = success && sensorOnline[j];
	}
	for (int j = 0; j < MAX_SENSORS; ++j) {
		zero_x[j] = 0;
		zero_y[j] = 0;
		zero_z[j] = 0;
		num_auto_zero_cycles[j] = -1;
	}
	doingAutoZero = false;

	reinitialize();

	return success;
}

void MHG_MMC5603NJ_Array:: reinitialize()
{




	softReset();
	performSetOperation();
	performResetOperation();
	for (int j = 0; j < nsensors; ++j) {
		sensorOnline[j] = getMMC(j)->isConnected() && !getMMC(j)->didSelfTestFail();
	}
	performSetOperation();
	performResetOperation();
	enableAutomaticSetReset();

}

void MHG_MMC5603NJ_Array::softReset(bool waitForReset)
{
	for (int j = 0; j < nsensors; ++j) {
		getMMC(j)->softReset(false);
	}

	if (waitForReset) {
		// The reset time is 20 msec. but we'll wait 15 msec. just in case.
		delay(25);
	}
}



void MHG_MMC5603NJ_Array::performSetOperation()
{
	for (int j = 0; j < nsensors; ++j) {
		getMMC(j)->performSetOperation();
	}

    // Wait until bit clears itself.
    delay(1);
}

void MHG_MMC5603NJ_Array::performResetOperation()
{
	for (int j = 0; j < nsensors; ++j) {
		getMMC(j)->performResetOperation();
	}

    // Wait until bit clears itself.
    delay(1);
}

void MHG_MMC5603NJ_Array::enableAutomaticSetReset()
{
	for (int j = 0; j < nsensors; ++j) {
		getMMC(j)->enableAutomaticSetReset();
	}
}

void MHG_MMC5603NJ_Array::disableAutomaticSetReset()
{
	for (int j = 0; j < nsensors; ++j) {
		getMMC(j)->disableAutomaticSetReset();
	}
}
void MHG_MMC5603NJ_Array::setFilterBandwidth(uint8_t bandwidth)
{
	for (int j = 0; j < nsensors; ++j) {
		getMMC(j)->setFilterBandwidth(bandwidth);
	}
}

void MHG_MMC5603NJ_Array::enableContinuousMode()
{
	for (int j = 0; j < nsensors; ++j) {
		getMMC(j)->enableContinuousMode();
	}
}

void MHG_MMC5603NJ_Array::disableContinuousMode()
{
	for (int j = 0; j < nsensors; ++j) {
		getMMC(j)->disableContinuousMode();
	}
}


void MHG_MMC5603NJ_Array::setContinuousModeFrequency(uint16_t frequency)
{
	for (int j = 0; j < nsensors; ++j) {
		getMMC(j)->setContinuousModeFrequency(frequency);
	}
}


void MHG_MMC5603NJ_Array::enablePeriodicSet()
{
	for (int j = 0; j < nsensors; ++j) {
		getMMC(j)->enablePeriodicSet();
	}
}


void MHG_MMC5603NJ_Array::disablePeriodicSet()
{
	for (int j = 0; j < nsensors; ++j) {
		getMMC(j)->disablePeriodicSet();
	}
}



void MHG_MMC5603NJ_Array::setPeriodicSetSamples(const uint16_t numberOfSamples)
{
	for (int j = 0; j < nsensors; ++j) {
		getMMC(j)->setPeriodicSetSamples(numberOfSamples);
	}
}



void MHG_MMC5603NJ_Array::requestMagMeasurement(uint64_t timeInUs)
{
	//3 cycles = 27 clocks to request a measurement
	//at 400 kHz, each start is delayed by 67.5 us relative to the previous
	//total request time = .54 ms for 8 sensors
	for (int j = 0; j < nsensors; ++j) {
		currentMeasurement.sensorOnline[j] = sensorOnline[j];

		if (!doingAutoZero) {
			currentMeasurement.zero_x[j] = zero_x[j];
			currentMeasurement.zero_y[j] = zero_y[j];
			currentMeasurement.zero_z[j] = zero_z[j];
		}
		if (sensorOnline[j]) {
			getMMC(j)->requestMagMeasurement();
		}
	}
	currentMeasurement.us = timeInUs;
	measurementFinished = false;
}

multiMagMeasurementT MHG_MMC5603NJ_Array::getMeasurement(){
	return currentMeasurement;
}
bool MHG_MMC5603NJ_Array::isMeasurementReady() {
	return measurementFinished;
}
void MHG_MMC5603NJ_Array::initMeasurementCycle(uint64_t timeInUs) {
	requestMagMeasurement(timeInUs);
	currentsensor = 0;
}

uint8_t MHG_MMC5603NJ_Array::measurementCycle(uint64_t timeInUs, bool &dataready, multiMagMeasurementT &measurement) {
	static uint8_t firstWorkingSensor = 255;
	if (firstWorkingSensor == 255) {
		for (uint8_t j = 0; j < nsensors; ++j) {
			if (sensorOnline[j]) {
				firstWorkingSensor = j;
				break;
			}
		}
	}

	const static uint64_t resetInterval = 4e4; //reset if readings not completed in 0.04 s (25 Hz minimum readout)

	if (measurementFinished || currentsensor >= nsensors || currentsensor < firstWorkingSensor || (timeInUs - currentMeasurement.us) > resetInterval) {
		dataready = measurementFinished;
		measurement = currentMeasurement;
		initMeasurementCycle(timeInUs);
		return 0;
	}
	//only check if sensor 0 is ready; readout is slow enough that all others should finish
	//by the time they are read

	if (currentsensor > firstWorkingSensor || getMMC(currentsensor)->isMeasurementReady()) {

		getMMC(currentsensor)->readMeasurementXYZ(currentMeasurement.x[currentsensor], currentMeasurement.y[currentsensor], currentMeasurement.z[currentsensor], readAllBits);
		if (doingAutoZero) {
			zero_x[currentsensor] += currentMeasurement.x[currentsensor];
			zero_y[currentsensor] += currentMeasurement.y[currentsensor];
			zero_z[currentsensor] += currentMeasurement.z[currentsensor];
			num_auto_zero_cycles[currentsensor]++;
		}
		measurementFinished = (++currentsensor >= nsensors);
	}
	if (measurementFinished && doingAutoZero) {
		for (int j = 0; j < nsensors; ++j) {
			if (num_auto_zero_cycles[j] >= autoZeroTarget) {
				doingAutoZero = false;
				break;
			}
		}
		if (!doingAutoZero) {
			for (int j = 0; j < nsensors; ++j) {
				zero_x[j] /= num_auto_zero_cycles[j];
				zero_y[j] /= num_auto_zero_cycles[j];
				zero_z[j] /= num_auto_zero_cycles[j];
			}
		}
	}

	dataready = false;
	return currentsensor;

}

MHG_MMC5603NJ *MHG_MMC5603NJ_Array::getMMC(uint8_t ind) {
	if (ind < 0) {
		ind = 0;
	}
	if (ind >= nsensors){
		ind = nsensors -1;
	}
	digitalWrite(sel0pin, ind & 1);
	digitalWrite(sel1pin, ind & 2);
	digitalWrite(sel2pin, ind & 4);
	//rely on function overhead for the 25ns delay needed for setup time
	return mmc + ind;


}
uint64_t MHG_MMC5603NJ_Array::sensorStatus(){
	uint64_t rv;
	for (uint8_t j = 0; j < nsensors && j < 64; ++j){
		rv = rv + (sensorOnline[j] << j);
	}
	return rv;
}

 //whether individual sensor is working
 bool MHG_MMC5603NJ_Array::isSensorActive(uint8_t sensorIndex){
	 return sensorOnline[sensorIndex];
 }

 bool MHG_MMC5603NJ_Array::isSensorConnected(uint8_t sensorIndex){
 	 return getMMC(sensorIndex)->isConnected();
  }

 void MHG_MMC5603NJ_Array::autoZeroSensors(int targetCycles) {
	 if (targetCycles <= 0) {
		 return;
	 }
	 for (int j = 0; j < MAX_SENSORS; ++j) {
	 		zero_x[j] = 0;
	 		zero_y[j] = 0;
	 		zero_z[j] = 0;
	 		num_auto_zero_cycles[j] = 0;
	 	}
	 	doingAutoZero = true;
	 	autoZeroTarget = targetCycles;
 }
