/**
 * @file   HMC5883.cpp
 * @author tonghuix <tonghuix@tonghuix-Studio-1450>
 * @date   Tue Jul 10 05:41:42 2012
 * 
 * @brief  Compass Sensor Code
 * 
 * 
 */

#include "wirish.h"
#include "misc.h"
#include "config.h"
#include "HMC5883.h"

float x_scale = 1;
float y_scale = 1;
float z_scale = 1;
float x_max, y_max, z_max;

#define HMC5883_CALIBRATE_SUCCESS 	0
#define HMC5883_CALIBRATE_ERROR 	-1

#define HMC5883_SET_GAIN(gain) writeTo(HMC5883_ADDR, HMC5883_R_CONFB, ((gain << 5) & 0xE0))

void compassInit(void) {
	delay(10);
	writeTo(HMC5883_ADDR, HMC5883_R_CONFA, 0x70);	// 8-average, 15 Hz default, normal measurement 每次输出采样8次，15HZ输出采样率，普通模式
	HMC5883_SET_GAIN(5);							// Gain=5
	writeTo(HMC5883_ADDR, HMC5883_R_MODE, 0x00);	// Continuous mode
	delay(10);

	// First read sets the gain
	compassData dummy;
	compassRead(&dummy);
	delay(80);
}

void compassRead(compassData *result) {
	uint8 buff[6];
	readFrom(HMC5883_ADDR, HMC5883_R_XM, 6, buff);

	// MSB byte first, then LSB; X,Z,Y
	result->x = ((((int16)buff[0]) << 8) & 0xFF00) | (((int16)buff[1]) & 0x00FF);   // X axis
	result->z = ((((int16)buff[2]) << 8) & 0xFF00) | (((int16)buff[3]) & 0x00FF);   // Z axis
	result->y = ((((int16)buff[4]) << 8) & 0xFF00) | (((int16)buff[5]) & 0x00FF);   // Y axis
}

/* Auto scales the gain to get good values */
int compassCalibrate(void) {
	uint8 gain = 0;
	compassData data;
	int status = HMC5883_CALIBRATE_ERROR;

	/* Set up self test mode */
	writeTo(HMC5883_ADDR, HMC5883_R_CONFA, 0x71); // 8-average, 15 Hz, self test on
	HMC5883_SET_GAIN(gain);
	writeTo(HMC5883_ADDR, HMC5883_R_MODE, 0x00); // Continuous mode
	delay(10);

	while (status != HMC5883_CALIBRATE_SUCCESS) {
		// First read sets the gain
		compassRead(&data);
		delay(80);

		// Do the real read to see what the data is
		compassRead(&data);
		delay(80);

		// Check if values are above an upper threshold (from datasheet example)
		if (data.x > 575 || data.y > 575 || data.z > 575) {
			// Increasing the gain won't fix this, return an error
			return HMC5883_CALIBRATE_ERROR;

		// If values are below a lower threshold, increase gain
		} else if (data.x < 243 || data.y < 243 || data.z < 243) {
			gain ++;
			if (gain > 7) {
				// Gain can only go up to 7
				return HMC5883_CALIBRATE_ERROR;
			}

			HMC5883_SET_GAIN(gain);
			delay(10);

		// Everything is in range
		} else {
			status = HMC5883_CALIBRATE_SUCCESS;
		}
	}

	writeTo(HMC5883_ADDR, HMC5883_R_CONFA, 0x71); // 8-average, 15 Hz, self test on



}

//
//void compassCalibrate(uint8 gain)
//{
//  int16 compassdata[3];
//  float fx = 0;
//  float fy = 0;
//  float fz = 0;
//
//  writeTo(HMC5883_ADDR, HMC5883_R_CONFA, 0x10 + HMC_POS_BIAS); // Reg A DOR=0x010 + MS1,MS0 set to pos bias
//  compassSetGain(gain);
//  float x, y, z, mx=0, my=0, mz=0;
//
//  for (uint8 i=0; i<10; i++)
//  {
//    commpassSetMode(1);
//    compassRead(compassdata);
//
//    fx = ((float) compassdata[0]) / x_scale;
//    fy = ((float) compassdata[1]) / y_scale;
//    fz = ((float) compassdata[2]) / z_scale;
//    x= (int16) (fx + 0.5);
//    y= (int16) (fy + 0.5);
//    z= (int16) (fz + 0.5);
//
//    if (x>mx) mx = x;
//    if (y>my) my = y;
//    if (z>mz) mz = z;
//  }
//
//  float max = 0;
//  if (mx>max) max = mx;
//  if (my>max) max = my;
//  if (mz>max) max = mz;
//  x_max = mx;
//  y_max = my;
//  z_max = mz;
//  x_scale = max/mx; // calc scales
//  y_scale = max/my;
//  z_scale = max/mz;
//  writeTo(HMC5883_ADDR, HMC5883_R_CONFA, 0x010); // set RegA/DOR back to default
//}

//
//// set data output rate
//// 0-6, 4 default, normal operation assumed
//void compassSetDOR(uint8 DOR)
//{
//  if (DOR>6) return;
//  writeTo(HMC5883_ADDR, HMC5883_R_CONFA, DOR<<2);
//}
//
//
//void compassSetGain(uint8 gain)
//{
//  // 0-7, 1 default
//  if (gain > 7) return;
//  writeTo(HMC5883_ADDR, HMC5883_R_CONFB, gain << 5);
//}



double compassHeading(void)
{
  float fx = 0;
  float fy = 0;
  float heading  = 0;
//  int16 compassdata[3];
//  compassRead(compassdata);
//  delay(67);//如果采样率为15HZ需要延迟67MS
//
//  fx = ((float) compassdata[0]) / x_scale;
//  fy = ((float) compassdata[1]) / y_scale;
//  heading = atan2(fy, fx);
//
//  // Correct for when signs are reversed.
//  if(heading < 0)    heading += 2*PI;

  return(heading * 180/PI); 

  
  
     /*使用俯仰角进行补偿的计算方式详见 SF9DOF代码
  float MAG_X;
  float MAG_Y;
  float cos_roll;
  float sin_roll;
  float cos_pitch;
  float sin_pitch;
  
  cos_roll = cos(roll);
  sin_roll = sin(roll);
  cos_pitch = cos(pitch);
  sin_pitch = sin(pitch);
  // Tilt compensated Magnetic filed X:
  MAG_X = magnetom_x*cos_pitch+magnetom_y*sin_roll*sin_pitch+magnetom_z*cos_roll*sin_pitch;
  // Tilt compensated Magnetic filed Y:
  MAG_Y = magnetom_y*cos_roll-magnetom_z*sin_roll;
  // Magnetic Heading
  MAG_Heading = atan2(-MAG_Y,MAG_X);
 */
}
