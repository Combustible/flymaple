#include "wirish.h"
#include "Gyroscope.h"
#include "MapleFreeRTOS.h"
#include "flymaple_utils.h"

using namespace Sensor;

/********************** Locally Accessible **********************/

static bool gIsInit = false;

// Device-specific calibration information
int16_t ofs[3] = {0, 0, 0};

static inline void getRawReading(int16_t *new_x, int16_t *new_y, int16_t *new_z)
{
	uint8_t buffer[6];
	read(GYRO_I2C_ADDR, GYRO_REG_DATA, 6, buffer);
	*new_x = BIG_ENDIAN_INT16_FROM_PTR(&buffer[0]) - ofs[0];
	*new_y = BIG_ENDIAN_INT16_FROM_PTR(&buffer[2]) - ofs[1];
	*new_z = BIG_ENDIAN_INT16_FROM_PTR(&buffer[4]) - ofs[2];
}

/********************** Globally Accessible **********************/

int16_t Gyroscope::x;
int16_t Gyroscope::y;
int16_t Gyroscope::z;

status Gyroscope::init()
{
	status ret;
	unsigned char buffer[2];

	if (gIsInit != true) {
		Sensor::init();

		// Check device id
		read(GYRO_I2C_ADDR, GYRO_REG_DEVID, 1, &buffer[0]);
		if ((buffer[0] & 0x7E) != GYRO_DEVID) {
			FLY_PRINT_ERR("ERROR: Gyroscope failure! Got invalid device ID");
			return FLYMAPLE_ERR_GYROSCOPE_FAIL;
		}

		// Set power management - use X gyro as a clock reference as recommended by datasheet
		write(GYRO_I2C_ADDR, GYRO_REG_PWR_MGM, GYRO_PWR_MGM_CLK_SEL_GYRO_X);

		// Wait 5 milliseconds
		vTaskDelay(5 / portTICK_RATE_MS);

		// Set sample rate
		write(GYRO_I2C_ADDR, GYRO_REG_DLPF_FS, GYRO_DLPF_FS_SEL_2000DEG | GYRO_DLPF_CFG_BW_20HZ);

		// Set the sample rate divider to update the registers at approximately 30 Hz
		// 1kHz / (33 + 1) = 29.411
		write(GYRO_I2C_ADDR, GYRO_REG_SMPLRT_DIV, 33);

		// Interrupts are not used
		write(GYRO_I2C_ADDR, GYRO_REG_INT_CFG, 0x00);

		// Wait 100 milliseconds
		vTaskDelay(100 / portTICK_RATE_MS);

		// Average 10 samples (500 ms) to get device offset. This is orientation independent.
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

			ofs[0] += acc_x / 10;
			ofs[1] += acc_y / 10;
			ofs[2] += acc_z / 10;
		}

		// Get the first reading
		ret = getReading();
		if (ret) {
			FLY_PRINT_ERR("ERROR: Gyroscope failure! First read returned error");
			return ret;
		}


		gIsInit = true;
	}

	return FLYMAPLE_SUCCESS;
}

status Gyroscope::getReading()
{
	int16_t tmpx = -32760, tmpy = -32760, tmpz = -32760;
	static uint8_t read_fail_count = 0;

	getRawReading(&tmpx, &tmpy, &tmpz);

	/*
	 *  Sensor is only rated for 2000 degrees / second, at 14.375 bits per degree.
	 *  That's roughly +-26750 range of values
	 *  This comparison will match values much outside that range, including 0xFFFF
	 */
	if (tmpx < -30000 || tmpy < -30000 || tmpz < -30000 || tmpx > 30000 || tmpy > 30000 || tmpz > 30000) {
		read_fail_count++;

		FLY_PRINT_ERR("WARNING: Gyroscope read failure - value out of range");

		if (read_fail_count >= 5) {
			FLY_PRINT_ERR("ERROR: Gyroscope failure! Last 5 values out of range");
			return FLYMAPLE_ERR_GYROSCOPE_FAIL;
		}

		return FLYMAPLE_WARN_GYROSCOPE_READ_FAIL;
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

void Gyroscope::computeRadians(double *rad_x, double *rad_y, double *rad_z)
{
	/*
	 * Need to do this in a critical section, just in case control transfers in the middle
	 * leaving one or two components updated and the others stale.
	 */
	taskENTER_CRITICAL();
	*rad_x = ((double)x / GYRO_SENSITIVITY) * (PI / 180.0);
	*rad_y = ((double)y / GYRO_SENSITIVITY) * (PI / 180.0);
	*rad_z = ((double)z / GYRO_SENSITIVITY) * (PI / 180.0);
	taskEXIT_CRITICAL();
}
