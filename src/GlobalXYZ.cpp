#include "wirish.h"
#include "GlobalXYZ.h"
#include "Accelerometer.h"
#include "Gyroscope.h"
#include "Compass.h"
#include "Pressure.h"
#include "MapleFreeRTOS.h"
#include "flymaple_utils.h"

using namespace Sensor;
using namespace GlobalXYZ;

/********************** Locally Accessible **********************/

static inline void update_up_vector(void)
{
	double new_up[3];
	double temp_up[3];
	double accel_x, accel_y, accel_z;
	double gyro_x, gyro_y, gyro_z;
	double magnitude;

	static int loopcount = 0;


	// Need to get all the sensor data at once
	taskENTER_CRITICAL();

	new_up[0] = up[0];
	new_up[1] = up[1];
	new_up[2] = up[2];

	accel_x = (double)Accelerometer::x;
	accel_y = (double)Accelerometer::y;
	accel_z = (double)Accelerometer::z;

	Gyroscope::computeRadians(&gyro_x, &gyro_y, &gyro_z);

	taskEXIT_CRITICAL();

	// Normalize accelerometer vector
	magnitude = sqrt(accel_x*accel_x + accel_y*accel_y + accel_z*accel_z);
	accel_x /= magnitude;
	accel_y /= magnitude;
	accel_z /= magnitude;

	// Reduce radians per second to radians per tick, and update sign to match accelerometer
	gyro_x /= UPDATE_FREQ_IN_HZ * (-1);
	gyro_y /= UPDATE_FREQ_IN_HZ * (-1);
	gyro_z /= UPDATE_FREQ_IN_HZ;

#ifdef ROTATION_2D_TEST_OLD
	temp_up[0] = new_up[0] * cos(gyro_y) + new_up[2] * sin(gyro_y);
	temp_up[1] = new_up[1];
	temp_up[2] = -new_up[0] * sin(gyro_y) + new_up[2] * cos(gyro_y);
	for(int i = 0; i < 3; i++) new_up[i] = temp_up[i];
#endif

	/*
	 *  Rotate the up vector by the gyroscope rotations, x first, then y
	 *
	 *  Note that z rotation is not used here! @todo - investigate ramifications
	 */

	double sinx = sin(gyro_x);
	double cosx = cos(gyro_x);
	double siny = sin(gyro_y);
	double cosy = cos(gyro_y);

	temp_up[0] = new_up[0] * cosy + new_up[1] * sinx * siny + new_up[2] * siny * cosx;
	temp_up[1] = new_up[1] * cosx - new_up[2] * sinx;
	temp_up[2] = -new_up[0] * siny + new_up[1] * cosy * sinx + new_up[2] * cosx * cosy;
	for(int i = 0; i < 3; i++) new_up[i] = temp_up[i];


	/*
	 * Combine gyroscope prediction with accelerometer feedback by averaging the vectors
	 * Below code tends towards gyroscope value with 8x more weight
	 */
	new_up[0] = (GYRO_TO_ACCEL_WEIGHT_RATIO * new_up[0] + accel_x) / (GYRO_TO_ACCEL_WEIGHT_RATIO + 1);
	new_up[1] = (GYRO_TO_ACCEL_WEIGHT_RATIO * new_up[1] + accel_y) / (GYRO_TO_ACCEL_WEIGHT_RATIO + 1);
	new_up[2] = (GYRO_TO_ACCEL_WEIGHT_RATIO * new_up[2] + accel_z) / (GYRO_TO_ACCEL_WEIGHT_RATIO + 1);

	// Floating point math isn't perfect, so every few runs re-normalize the vector
	if(loopcount > 5) {
		magnitude = sqrt(new_up[0]*new_up[0] + new_up[1]*new_up[1] + new_up[2]*new_up[2]);
		accel_x /= magnitude;
		accel_y /= magnitude;
		accel_z /= magnitude;
		loopcount = 0;
	}


	// Update the globally accessible values
	taskENTER_CRITICAL();

	up[0] = new_up[0];
	up[1] = new_up[1];
	up[2] = new_up[2];

	taskEXIT_CRITICAL();

	loopcount ++;
}


/********************** Globally Accessible **********************/

double GlobalXYZ::up[3] = {0, 0, 1};
double GlobalXYZ::rel_height;
double GlobalXYZ::orientation[2];

void GlobalXYZ::update()
{
	update_up_vector();
}





#ifdef NOTYET
#include <cmath>
#include <limits>
#include "wirish_time.h"
#include "io.h"
#include "Accelerometer.h"
#include "Compass.h"
#include "Gyroscope.h"
#include "GlobalXYZ.h"

#undef max

using std::sqrt;
using std::numeric_limits;

GlobalXYZ GlobalXYZ::xyz;
Vector<double> GlobalXYZ::X(3);
Vector<double> GlobalXYZ::Y(3);
Vector<double> GlobalXYZ::Z(3);
Vector<double> GlobalXYZ::estX(3);
Vector<double> GlobalXYZ::estY(3);
Vector<double> GlobalXYZ::estZ(3);
xSemaphoreHandle GlobalXYZ::mutex = xSemaphoreCreateMutex();
Vector<double> GlobalXYZ::northMagneticPole(3);
unsigned int GlobalXYZ::timestamp;

GlobalXYZ::GlobalXYZ()
{
#ifndef NDEBUG
	if(mutex == NULL) {
		while(1) {
			toggleLED();
			delay(1000);
		}
	}
#endif
	Vector<double> dTheta;
	//初始化全局坐标系的Z向量
	getZ(Z,dTheta);
	//初始化全局坐标系的X向量
	getX(Z,X,dTheta);
	//初始化全局坐标系的Y向量
	getY(X,Z,Y);
	//设置时间戳
	timestamp = micros();
}

GlobalXYZ::~GlobalXYZ()
{
}

void GlobalXYZ::getZ(Vector<double> & newZ,Vector<double> & deltaTheta)
{
	Vector<double> accOfG = Accelerometer::getReading();
	double modulus = sqrt(accOfG(0) * accOfG(0) + accOfG(1) * accOfG(1) + accOfG(2) * accOfG(2));
	newZ = Vector<double>(3);
	newZ(0) = -accOfG(0) / modulus;
	newZ(1) = -accOfG(1) / modulus;
	newZ(2) = -accOfG(2) / modulus;
	Vector<double> dZ = newZ - Z;
	deltaTheta = cross_prod(Z,dZ);
}

void GlobalXYZ::getX(const Vector<double> & newZ,Vector<double> & newX,Vector<double> & deltaTheta)
{
	//NOTICE:注意这里的北磁极是正北方向地平线以下的一个点，我们要的正北是与我们脚下地平线相切的方向
	Vector<double> north = Compass::getReading();
	//施密特正交化
	double offset = inner_prod(newZ,north) / inner_prod(newZ,newZ);
	north = north - offset * newZ;
	double modulus = sqrt(north(0) * north(0) + north(1) * north(1) + north(2) * north(2));
	newX = Vector<double>(3);
	newX(0) = north(0) / modulus;
	newX(1) = north(1) / modulus;
	newX(2) = north(2) / modulus;
	Vector<double> dX = newX - X;
	deltaTheta = cross_prod(X,dX);
}

void GlobalXYZ::getY(const Vector<double> & newX,const Vector<double> & newZ,Vector<double> & newY)
{
	newY = cross_prod(newZ,newX);
	double modulus = sqrt(newY(0) * newY(0) + newY(1) * newY(1) + newY(2) * newY(2));
	newY(0) = newY(0) / modulus;
	newY(1) = newY(1) / modulus;
	newY(2) = newY(2) / modulus;
}

inline void GlobalXYZ::update(const Vector<double> & newX,const Vector<double> & newY,const Vector<double> & newZ)
{
	//更新状态
	X = newX; Y = newY; Z = newZ;
}

void GlobalXYZ::getXYZ()
{
	//计算与上次时间戳之间的时间
	unsigned int newtimestamp = micros();
	double dt = ((newtimestamp > timestamp)?(newtimestamp - timestamp):(numeric_limits<unsigned int>::max() - timestamp + newtimestamp)) * 1.0 / 1e6;
	//计算XYZ三轴
	Vector<double> newZ,newX,newY;
	Vector<double> dThetaComp,dThetaAcc,dThetaGyro;
	xyz.getZ(newZ,dThetaAcc);
	xyz.getX(newZ,newX,dThetaComp);
	xyz.getY(newX,newZ,newY);
	Vector<double> angularSpd = Gyroscope::getReading();
	dThetaGyro = angularSpd * dt;
	//融合
	Vector<double> sum;
	sum = dThetaAcc + dThetaComp;
	sum = sum + dThetaGyro;
	Vector<double> dTheta = sum / 3;
	//计算返回值
	Vector<double> dX,dY,dZ;
	dX = cross_prod(dTheta,X);
	dY = cross_prod(dTheta,Y);
	dZ = cross_prod(dTheta,Z);
	xSemaphoreTake(mutex,portMAX_DELAY);
	//融合版本
	estX = X + dX; estY = Y + dY; estZ = Z + dZ;
	xSemaphoreGive(mutex);
	//更新状态
	xyz.update(newX,newY,newZ);
	//更新时间戳
	timestamp = micros();
}

Matrix<double> GlobalXYZ::getRPY(double & roll,double & pitch,double & yaw)
{
	Matrix<double> DCM(3,3);
	xSemaphoreTake(mutex,portMAX_DELAY);
	Vector<double> newX = estX, newY = estY, newZ = estZ;
	xSemaphoreGive(mutex);
	//计算体坐标系->全局坐标系的转换矩阵
	DCM(0,0) = newX(0);	DCM(0,1) = newX(1);	DCM(0,2) = newX(2);
	DCM(1,0) = newY(0);	DCM(1,1) = newY(1);	DCM(1,2) = newY(2);
	DCM(2,0) = newZ(0);	DCM(2,1) = newZ(1);	DCM(2,2) = newZ(2);
	//计算返回值
	roll = -atan2(DCM(2,1),DCM(2,2));
	pitch = asin(DCM(2,0));
	yaw = atan2(DCM(1,0),DCM(0,0));
	return DCM;
}

Vector<double> GlobalXYZ::getQuaternion()
{
	Vector<double> quaternion(4);
	xSemaphoreTake(mutex,portMAX_DELAY);
	Vector<double> newX = estX, newY = estY, newZ = estZ;
	xSemaphoreGive(mutex);
	Matrix<double> DCM(3,3);
	//计算体坐标系->全局坐标系的转换矩阵
	DCM(0,0) = newX(0);	DCM(0,1) = newX(1);	DCM(0,2) = newX(2);
	DCM(1,0) = newY(0);	DCM(1,1) = newY(1);	DCM(1,2) = newY(2);
	DCM(2,0) = newZ(0);	DCM(2,1) = newZ(1);	DCM(2,2) = newZ(2);
	double trace = DCM(0,0) + DCM(1,1) + DCM(2,2);
	if(trace > 0) {
		double s = 0.5 / sqrt(trace + 1.0);
		quaternion(0) = 0.25 / s;
		quaternion(1) = (DCM(2,1) - DCM(1,2)) * s;
		quaternion(2) = (DCM(0,2) - DCM(2,0)) * s;
		quaternion(3) = (DCM(1,0) - DCM(0,1)) * s;
	} else {
		if(DCM(0,0) > DCM(1,1) && DCM(0,0) > DCM(2,2)) {
			double s = 2.0 * sqrt(1.0 + DCM(0,0) - DCM(1,1) - DCM(2,2));
			quaternion(0) = (DCM(2,1) - DCM(1,2)) / s;
			quaternion(1) = 0.25 * s;
			quaternion(2) = (DCM(0,1) + DCM(1,0)) / s;
			quaternion(3) = (DCM(0,2) + DCM(2,0)) / s;
		} else if(DCM(1,1) > DCM(2,2)) {
			double s = 2.0 * sqrt(1.0 + DCM(1,1) - DCM(0,0) - DCM(2,2));
			quaternion(0) = (DCM(0,2) - DCM(2,0)) / s;
			quaternion(1) = (DCM(0,1) - DCM(1,0)) / s;
			quaternion(2) = 0.25 * s;
			quaternion(3) = (DCM(1,2) + DCM(2,1)) / s;
		} else {
			double s = 2.0 * sqrt(1.0 + DCM(2,2) - DCM(0,0) - DCM(1,1));
			quaternion(0) = (DCM(1,0) - DCM(0,1)) / s;
			quaternion(1) = (DCM(0,2) + DCM(2,0)) / s;
			quaternion(2) = (DCM(1,2) + DCM(2,1)) / s;
			quaternion(3) = 0.25 * s;
		}
	}
	
	return quaternion;
}

void vTaskUpdateXYZ(void * pvParameters)
{
	while(true) {
		GlobalXYZ::getXYZ();
		vTaskDelay(UPDATE_INTERVAL);
	}
}
#endif
