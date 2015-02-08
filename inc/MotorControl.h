/*
 * MotorControl.h
 *
 *  Created on: Feb 7, 2015
 *      Author: bmarohn
 */

#ifndef MOTORCONTROL_H_
#define MOTORCONTROL_H_



namespace MotorControl
{
	void setspeed(int16_t speed);
	int16_t getspeed();

	void update();

};


#endif /* MOTORCONTROL_H_ */
