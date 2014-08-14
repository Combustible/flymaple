#include "wirish.h"
#include "Accelerometer.h"
#include "MapleFreeRTOS.h"
#include "flymaple_utils.h"

using namespace Sensor;

/********************** Locally Accessible **********************/

static bool gIsInit = false;

// Device-specific calibration information
static const uint8_t ofs[3] = {0, 0, 0};

static inline void getRawReading(int16_t *new_x, int16_t *new_y, int16_t *new_z)
{
	uint8_t buffer[6];
	read(ACCEL_I2C_ADDR, ACCEL_I2C_REG_DATA, 6, buffer);
	*new_x = LITTLE_ENDIAN_INT16_FROM_PTR(&buffer[0]);
	*new_y = LITTLE_ENDIAN_INT16_FROM_PTR(&buffer[2]);
	*new_z = LITTLE_ENDIAN_INT16_FROM_PTR(&buffer[4]);
}

#ifdef OLD
Accelerometer Accelerometer::accelerometer;
const unsigned short Accelerometer::GRAVITY = 248;
//NOTICE:加速仪的z输出是飞机提坐标系的Z的反方向
const short Accelerometer::sign[3] = {1,1,-1};
const float Accelerometer::sensitivity = 0.004;
short Accelerometer::offset[3] = {0,0,0};
#endif

/********************** Globally Accessible **********************/

int16_t Accelerometer::x;
int16_t Accelerometer::y;
int16_t Accelerometer::z;

status Accelerometer::init()
{
	status ret;
	unsigned char buffer[2];

	if (gIsInit != true) {
		Sensor::init();

		// Check device id
		read(ACCEL_I2C_ADDR, ACCEL_I2C_REG_DEVID, 1, &buffer[0]);
		if (buffer[0] != ACCEL_DEVID) {
			FLY_PRINT_ERR("ERROR: Accelerometer failure! Got invalid device ID");
			return FLYMAPLE_ERR_ACCELEROMETER_FAIL;
		}

		// Set offsets
		write(ACCEL_I2C_ADDR, ACCEL_I2C_REG_OFSX, ofs[0]);
		write(ACCEL_I2C_ADDR, ACCEL_I2C_REG_OFSY, ofs[1]);
		write(ACCEL_I2C_ADDR, ACCEL_I2C_REG_OFSZ, ofs[2]);

		// Set sample rate
		write(ACCEL_I2C_ADDR, ACCEL_I2C_REG_BW_RATE, ACCEL_BW_RATE_100HZ);

		// Set data format
		write(ACCEL_I2C_ADDR, ACCEL_I2C_REG_DATA_FORMAT, ACCEL_DATA_FORMAT_SELF_TEST_DISABLE |
				ACCEL_DATA_FORMAT_FULL_RES_ENABLE |
				ACCEL_DATA_FORMAT_RANGE_2G);

		// Disable FIFO
		write(ACCEL_I2C_ADDR, ACCEL_I2C_REG_FIFO_CTL, ACCEL_FIFO_CTL_MODE_BYPASS);

		// Wait 5 milliseconds
		vTaskDelay(5 / portTICK_RATE_MS);

		// Enable measurements
		write(ACCEL_I2C_ADDR, ACCEL_I2C_REG_POWER_CTL, ACCEL_POWER_CTL_MEASURE_ENABLE);

		// Wait 100 milliseconds
		vTaskDelay(100 / portTICK_RATE_MS);

		// Get the first reading
		ret = getReading();
		if (ret) {
			FLY_PRINT_ERR("ERROR: Accelerometer failure! First read returned error");
			return ret;
		}

		gIsInit = true;
	}

	return FLYMAPLE_SUCCESS;
}


#ifdef OLD
	//计算误差(静止状态下加速仪应该所有读数是0，如果不为零就是误差)
	float accumulator[] = {0,0,0};
	for(int i = 0 ; i < 100 ; i++) {
		short x,y,z;
		getRawReading(x,y,z);
		accumulator[0] += x;
		accumulator[1] += y;
		accumulator[2] += z;
	}
	for(int i = 0 ; i < 3 ; i++) accumulator[i] /= 100;
	//FIXME:程序假设飞机一开始水平摆放，但是不一定正确
	accumulator[2] -= GRAVITY;
	for(int i = 0 ; i < 3 ; i++) offset[i] = accumulator[i];
#endif

status Accelerometer::getReading()
{
	int16_t tmpx = -32760, tmpy = -32760, tmpz = -32760;
	static uint8_t read_fail_count = 0;

	getRawReading(&tmpx, &tmpy, &tmpz);

	if (tmpx < -30000 && tmpy < -30000 && tmpz < -30000) {
		read_fail_count++;

		FLY_PRINT_ERR("WARNING: Accelerometer read failure - value out of range");

		if (read_fail_count >= 5) {
			FLY_PRINT_ERR("ERROR: Accelerometer failure! Last 5 values out of range");
			return FLYMAPLE_ERR_ACCELEROMETER_FAIL;
		}

		return FLYMAPLE_WARN_ACCELEROMETER_READ_FAIL;
	}

	/*
	 * Update the global values
	 *
	 * Need to do this in a critical section, just in case control transfers in the middle
	 * leaving one or two components updated and the others stale.
	 */
	taskENTER_CRITICAL();
	x = tmpx;
	y = tmpy;
	z = tmpz;
	taskEXIT_CRITICAL();

	read_fail_count = 0;

	return FLYMAPLE_SUCCESS;
}
