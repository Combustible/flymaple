/**
 * @file   Accelerometer.h
 * @author breadbread1984 <breadbread1984@163.com>
 * @date   Sat Jul 21 15:12:00 2012
 *
 * Last updated by: Byron Marohn <combustible@live.com>
 *                  August 3, 2014
 *
 * @brief  The namespace for manipulating the accelerometer.
 *
 * Caller should call Accelerometer::init() before otherwise accessing accelerometer routines.
 *
 * @copyright GPLv3
 */

#include "Error.h"
#include "Sensor.h"

#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H


#define ACCEL_I2C_ADDR                      (0x53)

#define ACCEL_REG_DEVID                     (0x00)
#define ACCEL_REG_OFSX                      (0x1E)
#define ACCEL_REG_OFSY                      (0x1F)
#define ACCEL_REG_OFSZ                      (0x20)
#define ACCEL_REG_BW_RATE                   (0x2C)
#define ACCEL_REG_POWER_CTL                 (0x2D)
#define ACCEL_REG_DATA_FORMAT               (0x31)
#define ACCEL_REG_DATA                      (0x32)
#define ACCEL_REG_FIFO_CTL                  (0x38)

#define ACCEL_DEVID                         (0xE5)

#define ACCEL_BW_RATE_1600HZ                (B00001111)
#define ACCEL_BW_RATE_800HZ                 (B00001110)
#define ACCEL_BW_RATE_400HZ                 (B00001101)
#define ACCEL_BW_RATE_200HZ                 (B00001100)
#define ACCEL_BW_RATE_100HZ                 (B00001011)
#define ACCEL_BW_RATE_50HZ                  (B00001010)
#define ACCEL_BW_RATE_25HZ                  (B00001001)
#define ACCEL_BW_RATE_LOW_POWER_200HZ       (B00011100)
#define ACCEL_BW_RATE_LOW_POWER_100HZ       (B00011011)
#define ACCEL_BW_RATE_LOW_POWER_50HZ        (B00011010)
#define ACCEL_BW_RATE_LOW_POWER_25HZ        (B00011001)

#define ACCEL_POWER_CTL_MEASURE_ENABLE      (B00001000)
#define ACCEL_POWER_CTL_MEASURE_DISABLE     (B00000000)

#define ACCEL_DATA_FORMAT_SELF_TEST_ENABLE  (B10000000)
#define ACCEL_DATA_FORMAT_FULL_RES_ENABLE   (B00001000)
#define ACCEL_DATA_FORMAT_RANGE_2G          (B00000000)
#define ACCEL_DATA_FORMAT_RANGE_4G          (B00000001)
#define ACCEL_DATA_FORMAT_RANGE_8G          (B00000010)
#define ACCEL_DATA_FORMAT_RANGE_16G         (B00000011)

#define ACCEL_FIFO_CTL_MODE_BYPASS          (B00000000)
#define ACCEL_FIFO_CTL_MODE_FIFO            (B01000000)
#define ACCEL_FIFO_CTL_MODE_STREAM          (B10000000)
#define ACCEL_FIFO_CTL_MODE_TRIGGER         (B11000000)

namespace Accelerometer
{
	/**
	 * Acceleration vector
	 */
	extern int16_t x;
	extern int16_t y;
	extern int16_t z;

	extern double gravity_magnitude;

	/**
	 * Initialize Accelerometer subsystem if not done already
	 */
	status init();

	/**
	 * Get the acceleration into x,y,z
	 */
	status getReading();

}


#endif
