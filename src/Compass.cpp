#include "wirish.h"
#include "Compass.h"
#include "MapleFreeRTOS.h"
#include "flymaple_utils.h"

using namespace Sensor;

/********************** Locally Accessible **********************/

static bool gIsInit = false;

// Device-specific calibration information
static const float gSign[3] = {1, 1, 1};
static const float gScale[3] = {1.18, 1, 1.10};

// Max sensitivity to start, will automatically decrease if it causes problems
static uint8_t gGain = 7;

static inline void getRawReading(int16_t *new_x, int16_t *new_y, int16_t *new_z)
{
	uint8_t buffer[6];
	read(COMPASS_I2C_ADDR, COMPASS_I2C_REG_DATA_OUT, 6, buffer);
	*new_x = BIG_ENDIAN_INT16_FROM_PTR(&buffer[0]);
	*new_y = BIG_ENDIAN_INT16_FROM_PTR(&buffer[4]);
	*new_z = BIG_ENDIAN_INT16_FROM_PTR(&buffer[2]);
}

/********************** Globally Accessible **********************/

float Compass::x = 1;
float Compass::y = 0;
float Compass::z = 0;

status Compass::init()
{
	status ret;

	if (gIsInit != true) {
		Sensor::init();

		write(COMPASS_I2C_ADDR, COMPASS_I2C_REG_CONFIG_A,
		      (COMPASS_CONFIG_A_AVG_SAMPLES_8 |
		       COMPASS_CONFIG_A_DATA_OUT_HZ_30 |
		       COMPASS_CONFIG_A_MEAS_MODE_NORMAL));
		write(COMPASS_I2C_ADDR, COMPASS_I2C_REG_CONFIG_B,
		      COMPASS_CONFIG_B_GAIN_VALUE(gGain));
		write(COMPASS_I2C_ADDR, COMPASS_I2C_REG_MODE,
		      COMPASS_MODE_I2C_REGULAR_SPEED | COMPASS_MODE_CONTINUOUS);

		// Wait 100 milliseconds
		vTaskDelay(100 / portTICK_RATE_MS);

		// Get the first reading
		ret = getReading();
		if (ret) {
			FLY_PRINT_ERR("ERROR: Compass failure! First read returned error");
			return ret;
		}

		gIsInit = true;
	}

	return FLYMAPLE_SUCCESS;
}

status Compass::getReading()
{
	int16_t tmpx, tmpy, tmpz;
	float floatx, floaty, floatz;
	double magnitude;
	static bool last_read_fail = false;
	static uint8_t read_fail_count = 0;

	getRawReading(&tmpx, &tmpy, &tmpz);


	// Check for compass data overflow indication
	if (tmpx == COMPASS_DATA_OUT_ERR_VAL ||
	    tmpy == COMPASS_DATA_OUT_ERR_VAL ||
	    tmpz == COMPASS_DATA_OUT_ERR_VAL) {

		if (last_read_fail == true) {
			/*
			 * The last read failed, too. To avoid accidentally decrementing gain twice,
			 * abort and wait for the next call
			 */
			last_read_fail = false;

			return FLYMAPLE_WARN_COMPASS_DATA_OVERFLOW;
		}

		if (gGain == 0) {
			FLY_PRINT_ERR("ERROR: Compass Failure! Attempting to decrement gain below 0.");
			return FLYMAPLE_ERR_COMPASS_FAIL;
		}

		gGain--;

		FLY_PRINT("WARNING: Compass overflow! Gain decreased to ");
		FLY_PRINT(gGain);
		FLY_PRINTLN();

		write(COMPASS_I2C_ADDR, COMPASS_I2C_REG_CONFIG_B,
		      COMPASS_CONFIG_B_GAIN_VALUE(gGain));

		last_read_fail = true;

		return FLYMAPLE_WARN_COMPASS_DATA_OVERFLOW;
	}


	// According to datasheet, values should range from -2048 to 2047
	if (x < COMPASS_DATA_OUT_MIN_VAL || x > COMPASS_DATA_OUT_MAX_VAL ||
	    y < COMPASS_DATA_OUT_MIN_VAL || y > COMPASS_DATA_OUT_MAX_VAL ||
	    z < COMPASS_DATA_OUT_MIN_VAL || z > COMPASS_DATA_OUT_MAX_VAL) {
		read_fail_count++;

		FLY_PRINT_ERR("WARNING: Compass read failure - value out of range");

		if (read_fail_count >= 5) {
			FLY_PRINT_ERR("ERROR: Compass failure! Last 5 values out of range");
			return FLYMAPLE_ERR_COMPASS_FAIL;
		}

		return FLYMAPLE_WARN_COMPASS_READ_FAIL;
	}

	floatx = gSign[0] * ((float)tmpx) * gScale[0];
	floaty = gSign[1] * ((float)tmpy) * gScale[1];
	floatz = gSign[2] * ((float)tmpz) * gScale[2];

	magnitude = sqrt((double)(floatx * floatx + floaty * floaty + floatz * floatz));

	/*
	 * Update the global values
	 *
	 * Need to do this in a critical section, just in case control transfers in the middle
	 * leaving one or two components updated and the others stale.
	 */
	taskENTER_CRITICAL();
	x = floatx / magnitude;
	y = floaty / magnitude;
	z = floatz / magnitude;
	taskEXIT_CRITICAL();

	last_read_fail = false;
	read_fail_count = 0;

	return FLYMAPLE_SUCCESS;
}

void Compass::calibrate()
{
	static float maxval = 0, maxx = 0, maxy = 0, maxz = 0;
#ifndef TEST_CALIBRATION
	// This code block for actually getting compass scale factors
	int16_t tmpx, tmpy, tmpz;

	getRawReading(&tmpx, &tmpy, &tmpz);
#else
	// This code block for testing that scale factors produce equal results
	float tmpx, tmpy, tmpz;

	getReading();
	tmpx = x;
	tmpy = y;
	tmpz = z;
#endif

	if (tmpx > maxx) maxx = tmpx;
	if (tmpy > maxy) maxy = tmpy;
	if (tmpz > maxz) maxz = tmpz;
	if (maxx > maxval) maxval = maxx;
	if (maxy > maxval) maxval = maxy;
	if (maxz > maxval) maxval = maxz;

	FLY_PRINT("scalex = ");
	FLY_PRINT(maxval / maxx);
	FLY_PRINT(" scaley = ");
	FLY_PRINT(maxval / maxy);
	FLY_PRINT(" scalez = ");
	FLY_PRINT(maxval / maxz);
	FLY_PRINTLN();

	delay(100);
}
