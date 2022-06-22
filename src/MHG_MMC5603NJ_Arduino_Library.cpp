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

#include "MHG_MMC5603NJ_Arduino_Library.h"
#include "MHG_MMC5603NJ_Arduino_Library_Constants.h"

void MHG_MMC5603NJ::setShadowBitVal(uint8_t registerAddress, const uint8_t bitMask, const bool val) {
	if (val){
		setShadowBit(registerAddress, bitMask);
	} else {
		clearShadowBit(registerAddress, bitMask);
	}
}

void MHG_MMC5603NJ::setShadowBitSelfClearing(uint8_t registerAddress, const uint8_t bitMask)
{ //sets a bit but does not update shadow register
    uint8_t *shadowRegister = nullptr;

    // Which register are we referring to?
    switch (registerAddress)
    {
    case INT_CTRL_0_REG:
    {
        shadowRegister = &memoryShadow.internalControl0;
    }
    break;

    case INT_CTRL_1_REG:
    {
        shadowRegister = &memoryShadow.internalControl1;
    }
    break;

    case INT_CTRL_2_REG:
    {
        shadowRegister = &memoryShadow.internalControl2;
    }
    break;

    case ODR_REG:
    {
        shadowRegister = &memoryShadow.odr;
    }
    break;

    default:
        break;
    }

    if (shadowRegister)
    {
        mmc_io.writeSingleByte(registerAddress, *shadowRegister | bitMask);
    }
}
void MHG_MMC5603NJ::setShadowBit(uint8_t registerAddress, const uint8_t bitMask)
{
    uint8_t *shadowRegister = nullptr;

    // Which register are we referring to?
    switch (registerAddress)
    {
    case INT_CTRL_0_REG:
    {
        shadowRegister = &memoryShadow.internalControl0;
    }
    break;

    case INT_CTRL_1_REG:
    {
        shadowRegister = &memoryShadow.internalControl1;
    }
    break;

    case INT_CTRL_2_REG:
    {
        shadowRegister = &memoryShadow.internalControl2;
    }
    break;

    case ODR_REG:
    {
        shadowRegister = &memoryShadow.odr;
    }
    break;

    default:
        break;
    }

    if (shadowRegister)
    {
        *shadowRegister |= bitMask;
        mmc_io.writeSingleByte(registerAddress, *shadowRegister);
    }
}

void MHG_MMC5603NJ::writeShadowRegister(uint8_t registerAddress, const uint8_t value)
{
    uint8_t *shadowRegister = nullptr;

    // Which register are we referring to?
    switch (registerAddress)
    {
    case INT_CTRL_0_REG:
    {
        shadowRegister = &memoryShadow.internalControl0;
    }
    break;

    case INT_CTRL_1_REG:
    {
        shadowRegister = &memoryShadow.internalControl1;
    }
    break;

    case INT_CTRL_2_REG:
    {
        shadowRegister = &memoryShadow.internalControl2;
    }
    break;

    case ODR_REG:
    {
        shadowRegister = &memoryShadow.odr;
    }
    break;

    default:
        break;
    }

    if (shadowRegister)
    {
        *shadowRegister = value;
        mmc_io.writeSingleByte(registerAddress, *shadowRegister);
    }
}

void MHG_MMC5603NJ::clearShadowBit(uint8_t registerAddress, const uint8_t bitMask)
{
    uint8_t *shadowRegister = nullptr;

    // Which register are we referring to?
    switch (registerAddress)
    {
    case INT_CTRL_0_REG:
    {
        shadowRegister = &memoryShadow.internalControl0;
    }
    break;

    case INT_CTRL_1_REG:
    {
        shadowRegister = &memoryShadow.internalControl1;
    }
    break;

    case INT_CTRL_2_REG:
    {
        shadowRegister = &memoryShadow.internalControl2;
    }
    break;

    case ODR_REG:
    {
    	shadowRegister = &memoryShadow.odr;
    }
    break;

    default:
        break;
    }

    if (shadowRegister)
    {
        *shadowRegister &= ~bitMask;
        mmc_io.writeSingleByte(registerAddress, *shadowRegister);
    }
}

uint8_t MHG_MMC5603NJ::readShadowRegister(uint8_t registerAddress) {
	return isShadowBitSet(registerAddress, 0xFF);
}

bool MHG_MMC5603NJ::isShadowBitSet(uint8_t registerAddress, const uint8_t bitMask)
{
    // Which register are we referring to?
    switch (registerAddress)
    {
    case INT_CTRL_0_REG:
    {
        return (memoryShadow.internalControl0 & bitMask);
    }
    break;

    case INT_CTRL_1_REG:
    {
        return (memoryShadow.internalControl1 & bitMask);
    }
    break;

    case INT_CTRL_2_REG:
    {
        return (memoryShadow.internalControl2 & bitMask);
    }
    break;

    case ODR_REG:
    {
        return (memoryShadow.odr & bitMask);
    }
    break;

    default:
        break;
    }

    return false;
}

void MHG_MMC5603NJ::setErrorCallback(void (*_errorCallback)(MHG_MMC5603NJ_ERROR errorCode))
{
    errorCallback = _errorCallback;
}

bool MHG_MMC5603NJ::begin(TwoWire &wirePort)
{
    // Initializes I2C and check if device responds
    bool success = mmc_io.begin(wirePort);

    if (!success)
    {
        SAFE_CALLBACK(errorCallback, MHG_MMC5603NJ_ERROR::I2C_INITIALIZATION_ERROR);
        return false;
    }
    return isConnected();
}


bool MHG_MMC5603NJ::isConnected()
{
    // Poll device for its ID.
    uint8_t response;
    response = mmc_io.readSingleByte(PROD_ID_REG);

    if (response != PROD_ID)
    {
        SAFE_CALLBACK(errorCallback, MHG_MMC5603NJ_ERROR::INVALID_DEVICE);
        return false;
    }
    return true;
}

int MHG_MMC5603NJ::getTemperature()
{
    // Send command to device. Since TM_T clears itself we don't need to
    // update the shadow register
	setShadowBitSelfClearing(INT_CTRL_0_REG, TM_T);
    int count = 0;
    // Wait until measurement is completed
    do
    {
        // Wait a little so we won't flood MMC with requests
        delay(5);
        ++ count;
    } while (count < 200 && !mmc_io.isBitSet(STATUS_REG, MEAS_T_DONE));

    if (count >= 200) {
    	SAFE_CALLBACK(errorCallback, MHG_MMC5603NJ_ERROR::MEAS_TIMEOUT);
    }
    // Get raw temperature value from the IC.
    uint8_t result = mmc_io.readSingleByte(T_OUT_REG);

    // Convert it using the equation provided in the datasheet
    float temperature = -75.0f + (static_cast<float>(result) * (200.0f / 255.0f));

    // Return the integer part of the temperature.
    return static_cast<int>(temperature);
}

void MHG_MMC5603NJ::softReset()
{
    // SW_RST bit clears itself
	setShadowBitSelfClearing(INT_CTRL_1_REG, SW_RST); //pg 9

    // The reset time is 10 msec. but we'll wait 15 msec. just in case.
    delay(15);
}



void MHG_MMC5603NJ::performSetOperation()
{
    //  SET bit clears itself
	setShadowBitSelfClearing(INT_CTRL_0_REG, SET_OPERATION); //pg 9

    // Wait until bit clears itself.
    delay(1);
}

void MHG_MMC5603NJ::performResetOperation()
{
    //  RESET bit clears itself
	setShadowBitSelfClearing(INT_CTRL_0_REG, RESET_OPERATION); //pg 9

    // Wait until bit clears itself.
    delay(1);
}

void MHG_MMC5603NJ::enableAutomaticSetReset()
{
    // This bit must be set through the shadow memory or we won't be
    // able to check if automatic set/reset is enabled using isAutomaticSetResetEnabled()
    setShadowBit(INT_CTRL_0_REG, AUTO_SR_EN); //pg 9
}

void MHG_MMC5603NJ::disableAutomaticSetReset()
{
    // This bit must be cleared through the shadow memory or we won't be
    // able to check if automatic set/reset is enabled using isAutomaticSetResetEnabled()
    clearShadowBit(INT_CTRL_0_REG, AUTO_SR_EN); //pg 9
}

bool MHG_MMC5603NJ::isAutomaticSetResetEnabled()
{
    // Get the bit value from the shadow register since the IC does not
    // allow reading INT_CTRL_0_REG register.
    return isShadowBitSet(INT_CTRL_0_REG, AUTO_SR_EN);
}

void MHG_MMC5603NJ::enableXChannel()
{
    // This bit must be cleared through the shadow memory or we won't be
    // able to check if the channel is enabled using isXChannelEnabled()
    // and since it's a inhibit bit it must be cleared so X channel will
    // be enabled.
    clearShadowBit(INT_CTRL_1_REG, X_INHIBIT); //pg 9
}

void MHG_MMC5603NJ::disableXChannel()
{
    // This bit must be set through the shadow memory or we won't be
    // able to check if the channel is enabled using isXChannelEnabled()
    // and since it's a inhibit bit it must be set so X channel will
    // be disabled.
    setShadowBit(INT_CTRL_1_REG, X_INHIBIT);
}

bool MHG_MMC5603NJ::isXChannelEnabled()
{
    // Get the bit value from the shadow register since the IC does not
    // allow reading INT_CTRL_1_REG register.
    return isShadowBitSet(INT_CTRL_1_REG, X_INHIBIT);
}

void MHG_MMC5603NJ::enableYZChannels()
{
    // This bit must be cleared through the shadow memory or we won't be
    // able to check if channels are enabled using areYZChannelsEnabled()
    // and since it's a inhibit bit it must be cleared so X channel will
    // be enabled.
    clearShadowBit(INT_CTRL_1_REG, YZ_INHIBIT); //pg 9
}

void MHG_MMC5603NJ::disableYZChannels()
{
    // This bit must be set through the shadow memory or we won't be
    // able to check if channels are enabled using areYZChannelsEnabled()
    // and since it's a inhibit bit it must be cleared so X channel will
    // be disabled.
    setShadowBit(INT_CTRL_1_REG, YZ_INHIBIT);
}

bool MHG_MMC5603NJ::areYZChannelsEnabled()
{
    // Get the bit value from the shadow register since the IC does not
    // allow reading INT_CTRL_1_REG register.
    return isShadowBitSet(INT_CTRL_1_REG, YZ_INHIBIT);
}

void MHG_MMC5603NJ::setFilterBandwidth(uint8_t bandwidth)
{
    // These must be set/cleared using the shadow memory since it can be read
    // using getFilterBandwith()

	setShadowBitVal(INT_CTRL_1_REG, BW0, bandwidth & (1 << 0));
	setShadowBitVal(INT_CTRL_1_REG, BW1, bandwidth & (1 << 1));

}

uint8_t MHG_MMC5603NJ::getFilterBandwith()
{
	uint8_t bw0 = (uint8_t) (isShadowBitSet(INT_CTRL_1_REG, BW0));
	uint8_t bw1 = (uint8_t) (isShadowBitSet(INT_CTRL_1_REG, BW1));

    return bw0 + bw1 << 1;

}

void MHG_MMC5603NJ::enableContinuousMode()
{
	// first calculate the measurement period
	//  CMM_FREQ_EN bit clears itself
	setShadowBitSelfClearing(INT_CTRL_0_REG, CMM_FREQ_EN); //pg 9

    // This bit must be set through the shadow memory or we won't be
    // able to check if continuous mode is enabled using isContinuousModeEnabled()

    setShadowBit(INT_CTRL_2_REG, CMM_EN); //pg 10
}

void MHG_MMC5603NJ::disableContinuousMode()
{
    // This bit must be cleared through the shadow memory or we won't be
    // able to check if continuous mode is enabled using isContinuousModeEnabled()
    clearShadowBit(INT_CTRL_2_REG, CMM_EN);
}

bool MHG_MMC5603NJ::isContinuousModeEnabled()
{
    // Get the bit value from the shadow register since the IC does not
    // allow reading INT_CTRL_2_REG register.
    return isShadowBitSet(INT_CTRL_2_REG, CMM_EN);
}

void MHG_MMC5603NJ::setContinuousModeFrequency(uint16_t frequency)
{
    // These must be set/cleared using the shadow memory since it can be read
    // using getContinuousModeFrequency()

	if (frequency == 0) {
		disableContinuousMode();
	}

	if (frequency < 256) {
		clearShadowBit(INT_CTRL_2_REG, HPOWER);
		writeShadowRegister(ODR_REG, (uint8_t) frequency);
	} else {
		setShadowBit(INT_CTRL_2_REG, HPOWER);
		writeShadowRegister(ODR_REG, 0xFF);
	}


}

uint16_t MHG_MMC5603NJ::getContinuousModeFrequency()
{
    // Since we cannot read INT_CTRL_2_REG we evaluate the shadow
    // memory contents and return the corresponding frequency.

    // Remove unwanted bits
    uint8_t registerValue = readShadowRegister(ODR_REG);
    bool hpower = isShadowBitSet(INT_CTRL_2_REG, HPOWER);
    if (hpower) {
    	return 1000;
    } else {
    	return registerValue;
    }

}

void MHG_MMC5603NJ::enablePeriodicSet()
{
    // This bit must be set through the shadow memory or we won't be
    // able to check if periodic set is enabled using isContinuousModeEnabled()
    setShadowBit(INT_CTRL_2_REG, EN_PRD_SET); //pg 10
}

void MHG_MMC5603NJ::disablePeriodicSet()
{
    // This bit must be cleared through the shadow memory or we won't be
    // able to check if periodic set is enabled using isContinuousModeEnabled()
    clearShadowBit(INT_CTRL_2_REG, EN_PRD_SET);
}

bool MHG_MMC5603NJ::isPeriodicSetEnabled()
{
    // Get the bit value from the shadow register since the IC does not
    // allow reading INT_CTRL_2_REG register.
    return isShadowBitSet(INT_CTRL_2_REG, EN_PRD_SET);
}

void MHG_MMC5603NJ::setPeriodicSetSamples(const uint16_t numberOfSamples)
{
    // We must use the shadow memory to do all bits manipulations but
    // we need to access the shadow memory directly, change bits and
    // write back at once.
    switch (numberOfSamples)
    {

   //note case 1 is default at end

    case 25:
    {
        // PRD_SET[2:0] = 001
        clearShadowBit(INT_CTRL_2_REG, PRD_SET_2);
        clearShadowBit(INT_CTRL_2_REG, PRD_SET_1);
        setShadowBit(INT_CTRL_2_REG, PRD_SET_0);
    }
    break;

    case 75:
    {
        // PRD_SET[2:0] = 010
        clearShadowBit(INT_CTRL_2_REG, PRD_SET_2);
        setShadowBit(INT_CTRL_2_REG, PRD_SET_1);
        clearShadowBit(INT_CTRL_2_REG, PRD_SET_0);
    }
    break;

    case 100:
    {
        // PRD_SET[2:0] = 011
        clearShadowBit(INT_CTRL_2_REG, PRD_SET_2);
        setShadowBit(INT_CTRL_2_REG, PRD_SET_1);
        setShadowBit(INT_CTRL_2_REG, PRD_SET_0);
    }
    break;

    case 250:
    {
        // PRD_SET[2:0] = 100
        setShadowBit(INT_CTRL_2_REG, PRD_SET_2);
        clearShadowBit(INT_CTRL_2_REG, PRD_SET_1);
        clearShadowBit(INT_CTRL_2_REG, PRD_SET_0);
    }
    break;

    case 500:
    {
        // PRD_SET[2:0] = 101
        setShadowBit(INT_CTRL_2_REG, PRD_SET_2);
        clearShadowBit(INT_CTRL_2_REG, PRD_SET_1);
        setShadowBit(INT_CTRL_2_REG, PRD_SET_0);
    }
    break;

    case 1000:
    {
        // PRD_SET[2:0] = 110
        setShadowBit(INT_CTRL_2_REG, PRD_SET_2);
        setShadowBit(INT_CTRL_2_REG, PRD_SET_1);
        clearShadowBit(INT_CTRL_2_REG, PRD_SET_0);
    }
    break;

    case 2000:
    {
        // PRD_SET[2:0] = 111
        setShadowBit(INT_CTRL_2_REG, PRD_SET_2);
        setShadowBit(INT_CTRL_2_REG, PRD_SET_1);
        setShadowBit(INT_CTRL_2_REG, PRD_SET_0);
    }
    break;

    case 1:
    default:
    {
        // PRD_SET[2:0] = 000
        clearShadowBit(INT_CTRL_2_REG, PRD_SET_2);
        clearShadowBit(INT_CTRL_2_REG, PRD_SET_1);
        clearShadowBit(INT_CTRL_2_REG, PRD_SET_0);
    }
    break;
    }
}

uint16_t MHG_MMC5603NJ::getPeriodicSetSamples()
{

    // Since we cannot read INT_CTRL_2_REG we evaluate the shadow
    // memory contents and return the corresponding period.

    // Remove unwanted bits
    uint8_t registerValue = memoryShadow.internalControl2 & 0x70;
    uint16_t period = 1;

    switch (registerValue)
    {
    case 0x10:
    {
        period = 25;
    }
    break;

    case 0x20:
    {
        period = 75;
    }
    break;

    case 0x30:
    {
        period = 100;
    }
    break;

    case 0x40:
    {
        period = 250;
    }
    break;

    case 0x50:
    {
        period = 500;
    }
    break;

    case 0x60:
    {
        period = 1000;
    }
    break;

    case 0x70:
    {
        period = 2000;
    }
    break;

    case 0x0:
    default:
        break;
    }

    return period;
}


void MHG_MMC5603NJ::getMeasurementXYZ(float &x, float &y, float &z, bool readAllBits)
{
	if (!isContinuousModeEnabled()) {
		// Send command to device. TM_M self clears
		setShadowBitSelfClearing(INT_CTRL_0_REG, TM_M);

		uint16_t delay_us;
		switch(getFilterBandwith()){
			//datasheet pg 15
		case 3:
			delay_us = 1200;
			break;
		case 2:
			delay_us = 2000;
			break;
		case 1:
			delay_us = 3500;
			break;
		case 0:
		default:
			delay_us = 6600;
			break;
		}
		delayMicroseconds(delay_us);
		// Wait until measurement is completed
		if (!isMeasurementReady(250,40)) {
			SAFE_CALLBACK(errorCallback, MHG_MMC5603NJ_ERROR::NO_MEASUREMENT);
			return;
		}

	}

	uint8_t buffer[9];

	if (readAllBits) {
		mmc_io.readMultipleBytes(X_OUT_0_REG, buffer, 9);
	} else {
		mmc_io.readMultipleBytes(X_OUT_0_REG, buffer, 6);
		buffer[6] = buffer[7] = buffer[8] = 0;
	}

	uint32_t xraw, yraw, zraw;
	xraw = buffer[0]; xraw = (xraw << 8) + buffer[1]; xraw = (xraw << 4) + buffer[6];
	yraw = buffer[2]; yraw = (yraw << 8) + buffer[3]; yraw = (yraw << 4) + buffer[7];
	zraw = buffer[4]; zraw = (zraw << 8) + buffer[5]; zraw = (zraw << 4) + buffer[8];

	const uint32_t zero = 524288; //datasheet pg 2
	const float countsPerUT = 163.84;
	x = (xraw - zero)/countsPerUT;
	y = (yraw - zero)/countsPerUT;
	z = (zraw - zero)/countsPerUT;
}

void MHG_MMC5603NJ::autoSetBW() {
	//based on data sheet page 8
	uint8_t bw;
	uint16_t f = getContinuousModeFrequency();
	if (isAutomaticSetResetEnabled()) {
		if (f <= 75){
			bw = 0;
		} else if (f <= 150) {
			bw = 1;
		} else {
			bw = 2;
		}
	} else {
		if (f <= 150){
			bw = 0;
		} else if (f <= 255) {
			bw = 1;
		} else {
			bw = 3;
		}
	}
	setFilterBandwidth(bw);
}

bool MHG_MMC5603NJ::isMeasurementReady(uint16_t delay_us, uint16_t maxreps) {
	if (mmc_io.isBitSet(STATUS_REG, MEAS_M_DONE)){
		return true;
	}
	if (delay_us > 0 && maxreps > 0) {
		for (int j = 0; j < maxreps; ++j) {
			delayMicroseconds(delay_us);
			if (mmc_io.isBitSet(STATUS_REG, MEAS_M_DONE)){
					return true;
				}
		}
	}
	return false;
}
