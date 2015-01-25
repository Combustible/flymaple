/**
 * @file   Gyroscope.h
 * @author breadbread1984 <breadbread1984@163.com>
 * @date   Sat Jul 21 15:12:00 2012
 *
 * Last updated by: Byron Marohn <combustible@live.com>
 *                  August 3, 2014
 *
 * @brief  The namespace for manipulating the gyroscope.
 *
 * Caller should call Gyroscope::init() before otherwise accessing gyroscope routines.
 *
 * @copyright GPLv3
 */

#include "Error.h"
#include "Sensor.h"

#ifndef GYROSCOPE_H
#define GYROSCOPE_H

#define GYRO_I2C_ADDR                      (0x68)

#define GYRO_REG_DEVID                     (0x00)
#define GYRO_REG_SMPLRT_DIV                (0x15)
#define GYRO_REG_DLPF_FS                   (0x16)
#define GYRO_REG_INT_CFG                   (0x17)
#define GYRO_REG_INT_STATUS                (0x1A)
#define GYRO_REG_TEMP_OUT                  (0x1B) // Two bytes wide, big-endian
#define GYRO_REG_DATA                      (0x1D) // Six bytes wide, big endian
#define GYRO_REG_PWR_MGM                   (0x3E)

#define GYRO_DEVID                         (B01101000)

#define GYRO_DLPF_FS_SEL_2000DEG           (B00011000)
#define GYRO_DLPF_CFG_BW_256HZ             (B00000000)
#define GYRO_DLPF_CFG_BW_188HZ             (B00000001)
#define GYRO_DLPF_CFG_BW_98HZ              (B00000010)
#define GYRO_DLPF_CFG_BW_42HZ              (B00000011)
#define GYRO_DLPF_CFG_BW_20HZ              (B00000100)
#define GYRO_DLPF_CFG_BW_10HZ              (B00000101)
#define GYRO_DLPF_CFG_BW_5HZ               (B00000110)

#define GYRO_PWR_MGM_RESET                 (B10000000)
#define GYRO_PWR_MGM_SLEEP                 (B01000000)
#define GYRO_PWR_MGM_STBY_XG               (B00100000)
#define GYRO_PWR_MGM_STBY_YG               (B00010000)
#define GYRO_PWR_MGM_STBY_ZG               (B00001000)
#define GYRO_PWR_MGM_CLK_SEL_INTERNAL      (B00000000)
#define GYRO_PWR_MGM_CLK_SEL_GYRO_X        (B00000001)
#define GYRO_PWR_MGM_CLK_SEL_GYRO_Y        (B00000010)
#define GYRO_PWR_MGM_CLK_SEL_GYRO_Z        (B00000011)
#define GYRO_PWR_MGM_CLK_SEL_EXTERN_32KHZ  (B00000100)
#define GYRO_PWR_MGM_CLK_SEL_EXTERN_19MHZ  (B00000101)

#define GYRO_SENSITIVITY                   (14.375f) // 14.375 LSBs per degree second

namespace Gyroscope
{
	/**
	 * Raw rotational speed about each axis in degrees * GYRO_SENSITIVITY per second
	 */
	extern int16_t x;
	extern int16_t y;
	extern int16_t z;

	/**
	 * Initialize Gyroscope subsystem if not done already
	 */
	status init();

	/**
	 * Get the angular velocity into x,y,z
	 */
	status getReading();

	/*
	 * Return the current reading in radians per second, sensitivity compensated
	 */
	void computeRadians(double *rad_x, double *rad_y, double *rad_z);
}


#endif
