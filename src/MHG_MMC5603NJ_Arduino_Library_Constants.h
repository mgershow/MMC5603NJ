/* library for the mmc5603nj magnetometer - adapted from sparkfun 5983 library, original header follows:
 *
  This is a library written for the MMC5983MA High Performance Magnetometer.
  SparkFun sells these at its website:
  https://www.sparkfun.com/products/19034

  Do you like this library? Help support open source hardware. Buy a board!

  Written by Ricardo Ramos  @ SparkFun Electronics, February 2nd, 2022.
  This file declares all constants used in the MMC5983MA High Performance Magnetometer Arduino Library.

  SparkFun code, firmware, and software is released under the MIT License(http://opensource.org/licenses/MIT).
  See LICENSE.md for more information.
*/

#ifndef _MHG_MMC5603NJ_CONSTANTS_
#define _MHG_MMC5603NJ_CONSTANTS_

#include <Arduino.h>

// Macro for invoking the callback if the function pointer is valid
#define SAFE_CALLBACK(cb, code) \
    if (cb != nullptr)          \
    {                           \
        cb(code);               \
    }


// Registers definitions

//X-Z out registers, datasheet pg 7
static const uint8_t X_OUT_0_REG    = 0x0;
static const uint8_t X_OUT_1_REG    = 0X01;
static const uint8_t X_OUT_2_REG    = 0X06;
static const uint8_t Y_OUT_0_REG    = 0x02;
static const uint8_t Y_OUT_1_REG    = 0x03;
static const uint8_t Y_OUT_2_REG    = 0x07;
static const uint8_t Z_OUT_0_REG    = 0x04;
static const uint8_t Z_OUT_1_REG    = 0x05;
static const uint8_t Z_OUT_2_REG    = 0x08;

static const uint8_t T_OUT_REG      = 0x09; //pg 8
static const uint8_t STATUS_REG     = 0x18; //pg 8
static const uint8_t ODR_REG        = 0x1a; //pg 8
static const uint8_t INT_CTRL_0_REG = 0x1b; //pg 9
static const uint8_t INT_CTRL_1_REG = 0x1c; //pg 9
static const uint8_t INT_CTRL_2_REG = 0x1d; //pg 10

static const uint8_t ST_X 			= 0x27;
static const uint8_t ST_Y 			= 0x28;
static const uint8_t ST_Z 			= 0x29;
static const uint8_t ST_X_TH 		= 0x1E;
static const uint8_t ST_Y_TH 		= 0x1F;
static const uint8_t ST_Z_TH 		= 0x20;




//not used so far, ST_X, ST_Y, ST_Z
static const uint8_t PROD_ID_REG    = 0x39; //pg 11 of data sheet
static const uint8_t DUMMY          = 0x0;
									
// Constants definitions            
static const uint8_t I2C_ADDR       = 0x30; //pg 12
static const uint8_t PROD_ID        = 0x10; //pg 11 of data sheet

// Bits definitions
#define MEAS_M_DONE                 (1 << 6) //pg 8 - status1
#define MEAS_T_DONE                 (1 << 7) //pg 8 - status1
#define OTP_READ_DONE               (1 << 4) //pg 8 - status1
#define SAT_SENSOR					(1 << 5) //pg 8 - status1

#define TM_M                        (1 << 0) //pg 9 - int_ctrl_0
#define TM_T                        (1 << 1) //pg 9 - int_ctrl_0

#define SET_OPERATION               (1 << 3) //pg 9 - int_ctrl_0 (do set)
#define RESET_OPERATION             (1 << 4) //pg 9 - int_ctrl_0 (do reset)
#define AUTO_SR_EN                  (1 << 5) //pg 9 - int_ctrl_0
#define AUTO_ST_EN					(1 << 6) //pg 9 - int_ctrl_0
#define CMM_FREQ_EN					(1 << 7) //pg 9 - int_ctrl_0 - required before cmm_en
//#define OTP_READ                    (1 << 6)
#define BW0                         (1 << 0) //pg 9 - int_ctrl_1
#define BW1                         (1 << 1) //pg 9 - int_ctrl_1
#define X_INHIBIT                   (1 << 2) //pg 9 - int_ctrl_1
#define YZ_INHIBIT                  (3 << 3) //pg 9 - int_ctrl_1
#define SW_RST                      (1 << 7) //pg 9 - int_ctrl_1
//#define CM_FREQ_0                   (1 << 0)
//#define CM_FREQ_1                   (1 << 1)
//#define CM_FREQ_2                   (1 << 2)
#define CMM_EN                      (1 << 4) // pg 10 - int_crl_2
#define PRD_SET_0                   (1 << 0) // pg 10 - int_crl_2
#define PRD_SET_1                   (1 << 1) // pg 10 - int_crl_2
#define PRD_SET_2                   (1 << 2) // pg 10 - int_crl_2
#define EN_PRD_SET                  (1 << 3) // pg 10 - int_ctrl_2
#define HPOWER						(1 << 7) // pg 10 - int_ctrl_2



enum class MHG_MMC5603NJ_ERROR
{
  NONE,
  I2C_INITIALIZATION_ERROR,
  INVALID_DEVICE,
  NO_MEASUREMENT,
  MEAS_TIMEOUT,

};

#endif
