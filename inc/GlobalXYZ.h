#ifndef GLOBALXYZ_H
#define GLOBALXYZ_H

/**
 * @file   GlobalXYZ.h
 * @author breadbread1984 <breadbread1984@163.com>
 * @date   Sat Jul 21 15:12:00 2012
 * 
 * @brief	The static class for calculating the axes of the global frame represented in the body frame. 
 * 
 * @copyright GPLv3 
 */

#include "MapleFreeRTOS.h"
#include "Vector.h"
#include "Sensor.h"

// If you change this, you probably also have to change the sampling rate of the sensors
#define UPDATE_INTERVAL 	50
#define UPDATE_FREQ_IN_HZ 	(1000/UPDATE_INTERVAL)

#define GYRO_TO_ACCEL_WEIGHT_RATIO 	6.0

namespace GlobalXYZ
{
	/*
	 * The 3D vector orientation, compared to the normal vector to a horizontal plane.
	 *
	 * If the sensors on the board are horizontal, the value should be <0, 0, 1>
	 *
	 * up[0] - X component
	 * up[1] - Y component
	 * up[2] - Z component
	 *
	 * Accelerometer data is augmented with gyroscope data to obtain this value
	 */
	extern double up[3];

	/*
	 * Relative height from the altitude when the board initialized.
	 *
	 * Barometer data is augmented with accelerometer data to obtain this value
	 */
	extern double rel_height;

	/*
	 * 2D vector of rotation. This is tilt-compensated.
	 *
	 * Compass data is augmented with orientation data and gyroscope data to obtain this value
	 *
	 * orientation[0] - north component
	 * orientation[1] - east component
	 */
	extern double orientation[2];

	/**
	 * Update the values using sensor data
	 */
	void update();
}

#ifdef NOTYET
class GlobalXYZ {
	static GlobalXYZ xyz;
	/**
	 * Constructor initializing the GlobalXYZ.
	 * To do the initialization for only once, the constructor is hide from developer.
	 * The only one GlobalXYZ object is a static one created automatically.
	 */
	GlobalXYZ();
	//全局坐标系的xyz
	static Vector<double> X;
	static Vector<double> Y;
	static Vector<double> Z;
	static Vector<double> estX;
	static Vector<double> estY;
	static Vector<double> estZ;
	static xSemaphoreHandle mutex;
	static Vector<double> northMagneticPole;
	static unsigned int timestamp;
	void getZ(Vector<double> & newZ,Vector<double> & deltaTheta);
	void getX(const Vector<double> & newZ,Vector<double> & newX,Vector<double> & deltaTheta);
	void getY(const Vector<double> & newX,const Vector<double> & newZ,Vector<double> & newY);
	void update(const Vector<double> & newX,const Vector<double> & newY,const Vector<double> & newZ);
public:
	/**
	 * Destructor.
	 */
	~GlobalXYZ();
	/**
	 * Get the axes of the global frame represented in the body frame.
	 * 
	 * @param X is the X axis of the global frame represented in the body frame.
	 * @param Y is the Y axis of the global frame represented in the body frame.
	 * @param Z is the Z axis of the global frame represented in the body frame.
	 */
	//获得全局坐标系的X,Y,Z的在体坐标系中的向量
	static void getXYZ();
	/**
	 * Get the roll, pitch and yaw angle calculated from the axes of the global frame.
	 * 
	 * @param roll the roll angle.
	 * @param pitch the pitch angle.
	 * @param yaw the yaw angle.
	 * @return the direction cosine matrix.
	 */
	//获得飞机的滚动角，俯仰角，偏航角
	static Matrix<double> getRPY(double & roll,double & pitch,double & yaw);
	/**
	 * Get the quaternion representation of the rotation
	 * 
	 * @return the quaterion vector.
	 */
	//获得四元数表示的旋转向量
	static Vector<double> getQuaternion();
};

void vTaskUpdateXYZ(void * pvParameters);
#endif

#endif
