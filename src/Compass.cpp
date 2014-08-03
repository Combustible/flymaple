#include "wirish.h"
#include "Compass.h"

const float Compass::sign[3] = {1, 1, 1};
const float Compass::scale[3] = {1.18, 1, 1.10};

// Max sensitivity to start, will automatically decrease if it causes problems
unsigned char Compass::gain = 7;

// Reasonable default orientation
float Compass::x = 1;
float Compass::y = 0;
float Compass::z = 0;

bool init_done = false;

Compass::Compass()
{
	if (init_done != true) {
		write(COMPASS_I2C_ADDR, COMPASS_I2C_REG_CONFIG_A,
		      (COMPASS_CONFIG_A_AVG_SAMPLES_8 |
		       COMPASS_CONFIG_A_DATA_OUT_HZ_15 |
		       COMPASS_CONFIG_A_MEAS_MODE_NORMAL));
		write(COMPASS_I2C_ADDR, COMPASS_I2C_REG_CONFIG_B,
		      COMPASS_CONFIG_B_GAIN_VALUE(gain));
		write(COMPASS_I2C_ADDR, COMPASS_I2C_REG_MODE,
		      COMPASS_MODE_I2C_REGULAR_SPEED | COMPASS_MODE_CONTINUOUS);

		delay(100);

		getReading();

		init_done = true;
	}
}

void Compass::getRawReading(int16_t *x, int16_t *y, int16_t *z)
{
	uint8_t buffer[6];
	read(COMPASS_I2C_ADDR, COMPASS_I2C_REG_DATA_OUT, 6, buffer);
	*x = (((short)buffer[0]) << 8) | buffer[1];    // X axis
	*y = (((short)buffer[4]) << 8) | buffer[5];    // Y axis
	*z = (((short)buffer[2]) << 8) | buffer[3];    // Z axis
}

Compass::~Compass()
{
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

		if (gain == 0) {
#ifndef NDEBUG
			SerialUSB.println("ERROR: Compass Failure! Attempting to decrement gain below 0.");
#endif
			return FLYMAPLE_ERR_COMPASS_FAIL;
		}

		gain--;

#ifndef NDEBUG
		SerialUSB.print("WARNING: Compass overflow! Gain decreased to ");
		SerialUSB.print(gain);
		SerialUSB.println();
#endif

		write(COMPASS_I2C_ADDR, COMPASS_I2C_REG_CONFIG_B,
		      COMPASS_CONFIG_B_GAIN_VALUE(gain));

		last_read_fail = true;

		return FLYMAPLE_WARN_COMPASS_DATA_OVERFLOW;
	}


	// According to datasheet, values should range from -2048 to 2047
	if (x < COMPASS_DATA_OUT_MIN_VAL || x > COMPASS_DATA_OUT_MAX_VAL ||
	    y < COMPASS_DATA_OUT_MIN_VAL || y > COMPASS_DATA_OUT_MAX_VAL ||
	    z < COMPASS_DATA_OUT_MIN_VAL || z > COMPASS_DATA_OUT_MAX_VAL) {
		read_fail_count++;

#ifndef NDEBUG
		SerialUSB.println("WARNING: Compass read failure - value out of range");
#endif

		if (read_fail_count >= 5) {
#ifndef NDEBUG
			SerialUSB.println("ERROR: Compass failure! Last 5 values out of range");
#endif
			return FLYMAPLE_ERR_COMPASS_FAIL;
		}

		return FLYMAPLE_WARN_COMPASS_READ_FAIL;
	}

	floatx = sign[0] * ((float)tmpx) * scale[0];
	floaty = sign[1] * ((float)tmpy) * scale[1];
	floatz = sign[2] * ((float)tmpz) * scale[2];

	magnitude = sqrt((double)(floatx * floatx + floaty * floaty + floatz * floatz));

	x = floatx / magnitude;
	y = floaty / magnitude;
	z = floatz / magnitude;

	last_read_fail = false;
	read_fail_count = 0;

	return FLYMAPLE_SUCCESS;
}

void Compass::calibrate()
{
	static float maxval = 0, maxx = 0, maxy = 0, maxz = 0;
#ifndef TEST_CALIBRATION
	int16_t tmpx, tmpy, tmpz;

	getRawReading(&tmpx, &tmpy, &tmpz);
#else
	float tmpx, tmpy, tmpz;

	getReading();
	tmpx = x;
	tmpy = y;
	tmpz = z;
#endif

	if (tmpx > maxx) maxx = tmpx;
	if (maxx > maxval) maxval = maxx;
	if (tmpy > maxy) maxy = tmpy;
	if (maxy > maxval) maxval = maxy;
	if (tmpz > maxz) maxz = tmpz;
	if (maxz > maxval) maxval = maxz;

#ifndef NDEBUG
	SerialUSB.print("scalex = ");
	SerialUSB.print(maxval / maxx);
	SerialUSB.print(" scaley = ");
	SerialUSB.print(maxval / maxy);
	SerialUSB.print(" scalez = ");
	SerialUSB.print(maxval / maxz);
	SerialUSB.println();
#endif

	delay(100);
}
