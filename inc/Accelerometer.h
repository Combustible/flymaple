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

#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H


#include "Error.h"
#include "Sensor.h"
#include "Vector.h"

namespace Accelerometer
{

	/**
	 * Initialize Accelerometer subsystem if not done already
	 */
	void init();
	
	/**
	 * Get the accelerations on three axes of the body frame.
	 * 
	 * @return the acceleration on three axes (g)
	 */
	Vector<double> getReading();
};


#endif
