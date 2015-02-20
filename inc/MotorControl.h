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
	void setheight(double target_height);
	void unset_height(void);
	void setspeed(int16_t speed);
	int16_t getspeed(void);
	bool is_height_set(void);

	void update(void);

};


#endif /* MOTORCONTROL_H_ */
