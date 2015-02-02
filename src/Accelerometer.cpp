#include "wirish.h"
#include "Accelerometer.h"
#include "MapleFreeRTOS.h"
#include "flymaple_utils.h"

using namespace Sensor;

/********************** Locally Accessible **********************/

static bool gIsInit = false;

// Device-specific calibration information
static const int8_t ofs[3] = {0, -1, 12};

static inline void getRawReading(int16_t *new_x, int16_t *new_y, int16_t *new_z)
{
	uint8_t buffer[6];
	read(ACCEL_I2C_ADDR, ACCEL_REG_DATA, 6, buffer);
	*new_x = LITTLE_ENDIAN_INT16_FROM_PTR(&buffer[0]);
	*new_y = LITTLE_ENDIAN_INT16_FROM_PTR(&buffer[2]);
	*new_z = LITTLE_ENDIAN_INT16_FROM_PTR(&buffer[4]);
}

#ifdef OLD
Accelerometer Accelerometer::accelerometer;
const unsigned short Accelerometer::GRAVITY = 248;
//NOTICE:加速仪的z输出是飞机提坐标系的Z的反方向
const short Accelerometer::sign[3] = {1, 1, -1};
const float Accelerometer::sensitivity = 0.004;
short Accelerometer::offset[3] = {0, 0, 0};
#endif

/********************** Globally Accessible **********************/

int16_t Accelerometer::x;
int16_t Accelerometer::y;
int16_t Accelerometer::z;
double Accelerometer::gravity_magnitude;

status Accelerometer::init()
{
	status ret;
	unsigned char buffer[2];

	if (gIsInit != true) {
		Sensor::init();

		// Check device id
		read(ACCEL_I2C_ADDR, ACCEL_REG_DEVID, 1, &buffer[0]);
		if (buffer[0] != ACCEL_DEVID) {
			FLY_PRINT_ERR("ERROR: Accelerometer failure! Got invalid device ID");
			return FLYMAPLE_ERR_ACCELEROMETER_FAIL;
		}

		// Set offsets
		write(ACCEL_I2C_ADDR, ACCEL_REG_OFSX, ofs[0]);
		write(ACCEL_I2C_ADDR, ACCEL_REG_OFSY, ofs[1]);
		write(ACCEL_I2C_ADDR, ACCEL_REG_OFSZ, ofs[2]);

		// Set sample rate
		write(ACCEL_I2C_ADDR, ACCEL_REG_BW_RATE, ACCEL_BW_RATE_25HZ);

		// Set data format
		write(ACCEL_I2C_ADDR,
		      ACCEL_REG_DATA_FORMAT,
		      ACCEL_DATA_FORMAT_FULL_RES_ENABLE | ACCEL_DATA_FORMAT_RANGE_2G);

		// Disable FIFO
		write(ACCEL_I2C_ADDR, ACCEL_REG_FIFO_CTL, ACCEL_FIFO_CTL_MODE_BYPASS);

		// Wait 5 milliseconds
		vTaskDelay(5 / portTICK_RATE_MS);

		// Enable measurements
		write(ACCEL_I2C_ADDR, ACCEL_REG_POWER_CTL, ACCEL_POWER_CTL_MEASURE_ENABLE);

		// Wait 1 second
		vTaskDelay(1000 / portTICK_RATE_MS);

		// Average 10 samples (500 ms) to determine gravity magnitude.
		// This is orientation independent, as long as the board is not moving
		{
			int16_t temp_x, temp_y, temp_z;
			int32_t acc_x = 0, acc_y = 0, acc_z = 0; // Accumulator

			// Throw away first reading
			getRawReading(&temp_x, &temp_y, &temp_z);

			for (int i = 0 ; i < 10 ; i++) {
				getRawReading(&temp_x, &temp_y, &temp_z);
				acc_x += temp_x;
				acc_y += temp_y;
				acc_z += temp_z;
				vTaskDelay(20 / portTICK_RATE_MS);
			}

			// Compute average vector magnitude
			gravity_magnitude = sqrt(((double)acc_x * (double)acc_x) / 100.0 +
			                         ((double)acc_y * (double)acc_y) / 100.0 +
			                         ((double)acc_z * (double)acc_z) / 100.0);
#ifdef DEBUG
			FLY_PRINT("Gravity magnitude is: ");
			FLY_PRINTLN(gravity_magnitude);
#endif
		}

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

status Accelerometer::getReading()
{
	int16_t tmpx = -32760, tmpy = -32760, tmpz = -32760;
	static uint8_t read_fail_count = 0;

	getRawReading(&tmpx, &tmpy, &tmpz);

	// 30000 is past the maximum sensor range. This also catches 0xFFFF as an error.
	if (tmpx < -30000 || tmpy < -30000 || tmpz < -30000 || tmpx > 30000 || tmpy > 30000 || tmpz > 30000) {
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
