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


bool MHG_MMC5603NJ_Array:: begin(TwoWire &wirePort, uint8_t nsensors)
{

	bool success = true;
	this->nsensors = nsensors;

	for (int j = 0; j < nsensors; ++j) {
		success = success && getMMC(j)->begin(wirePort);
	}

	softReset();
	enableAutomaticSetReset();
	return success;
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
		getMMC(j)->requestMagMeasurement();
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

void MHG_MMC5603NJ_Array::measurementCycle(uint64_t timeInUs, bool &dataready, multiMagMeasurementT &measurement) {
	if (measurementFinished) {
		dataready = true;
		measurement = currentMeasurement;
		requestMagMeasurement(timeInUs);
		currentsensor = 0;
		return;
	}
	if (getMMC(currentsensor)->isMeasurementReady()) {
		getMMC(currentsensor)->readMeasurementXYZ(currentMeasurement.x[currentsensor], currentMeasurement.y[currentsensor], currentMeasurement.z[currentsensor], readAllBits);
		measurementFinished = (++currentsensor >= nsensors);
	}
	dataready = false;
	return;

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



