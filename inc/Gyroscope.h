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

#ifndef GYROSCOPE_H
#define GYROSCOPE_H


#include "Sensor.h"
#include "Vector.h"

namespace Gyroscope
{

	/**
	 * Initialize Gyroscope subsystem if not done already
	 */
	void init();

	/**
	 * Get the angular speed from the gyroscope.
	 * 
	 * @return the angular speeds (rad/s) around the three axis of the body frame.
	 */
	Vector<double> getReading();

};


#endif
