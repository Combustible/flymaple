#include "wirish.h"
#include "Pressure.h"
#include "MapleFreeRTOS.h"
#include "flymaple_utils.h"

using namespace Sensor;

/********************** Locally Accessible **********************/

static bool gIsInit = false;

/** BMP085 Pressure over sampling setting; 0-3 */
static const uint8_t OSS = 2;

/** Datasheet specified delay after initiating a pressure measurement, in milliseconds */
static const uint8_t OSS_SAMPLE_DELAY_IN_MS[4] = {5, 8, 14, 26};

// EEPROM device constants
static int16_t ac1, ac2, ac3;
static uint16_t ac4, ac5, ac6;
static int16_t b1, b2, mb, mc, md;

// Computed value using temperature reading
static uint16_t b5;

// Raw temperature and pressure values from the sensor
static int32_t ut, up;

static inline void getRawReading(int32_t *new_ut, int32_t *new_up)
{
	uint8_t buffer[3];
	static uint8_t temp_counter = 0;

	/*********************** Raw Temperature ***********************/

	// Only read the temperature every few calls
	if (temp_counter == 0) {
		write(BMP085_I2C_ADDR, BMP085_I2C_REG_CONTROL, BMP085_CMD_READ_TEMPERATURE);

		// Wait 5 milliseconds
		vTaskDelay(5 / portTICK_RATE_MS);

		read(BMP085_I2C_ADDR, BMP085_I2C_REG_CONTROL_OUTPUT, 2, buffer);

		*new_ut = (((int32_t)buffer[0]) << 8) | ((int32_t)buffer[1]);

		temp_counter = PRESSURE_VS_TEMPERATURE_READ_RATIO;

		// 1 millisecond delay between temperature and pressure
		vTaskDelay(1 / portTICK_RATE_MS);
	} else {
		temp_counter --;
	}

	/*********************** Raw Pressure ***********************/

	write(BMP085_I2C_ADDR, BMP085_I2C_REG_CONTROL, (BMP085_CMD_READ_PRESSURE | (OSS << 6)));

	// Wait up to 26 milliseconds, depending on OSS
	vTaskDelay(OSS_SAMPLE_DELAY_IN_MS[OSS] / portTICK_RATE_MS);

	read(BMP085_I2C_ADDR, BMP085_I2C_REG_CONTROL_OUTPUT, 3, buffer);

	*new_up = ((((int32_t)buffer[0]) << 16) |
	           (((int32_t)buffer[1]) << 8) |
	           (((int32_t)buffer[2]))) >> (8 - OSS);
}

/********************** Globally Accessible **********************/

// Default to 25 degrees C, sea level
int32_t Pressure::temperature = 250;
int32_t Pressure::pressure = PRESSURE_AT_SEALEVEL_IN_PA;

status Pressure::init()
{
	status ret;
	uint8_t buffer[BMP085_I2C_REG_EEPROM_LEN];

	if (gIsInit != true) {
		Sensor::init();

		// Read all the EEPROM data at once
		read(BMP085_I2C_ADDR, BMP085_I2C_REG_EEPROM, sizeof(buffer), buffer);

		// Parse data into appropriate constants
		ac1 = BIG_ENDIAN_INT16_FROM_PTR(&buffer[0]);
		ac2 = BIG_ENDIAN_INT16_FROM_PTR(&buffer[2]);
		ac3 = BIG_ENDIAN_INT16_FROM_PTR(&buffer[4]);
		ac4 = BIG_ENDIAN_UINT16_FROM_PTR(&buffer[6]);
		ac5 = BIG_ENDIAN_UINT16_FROM_PTR(&buffer[8]);
		ac6 = BIG_ENDIAN_UINT16_FROM_PTR(&buffer[10]);
		b1 = BIG_ENDIAN_INT16_FROM_PTR(&buffer[12]);
		b2 = BIG_ENDIAN_INT16_FROM_PTR(&buffer[14]);
		mb = BIG_ENDIAN_INT16_FROM_PTR(&buffer[16]);
		mc = BIG_ENDIAN_INT16_FROM_PTR(&buffer[18]);
		md = BIG_ENDIAN_INT16_FROM_PTR(&buffer[20]);

		// Wait 100 milliseconds
		vTaskDelay(100 / portTICK_RATE_MS);

		// Get the first reading
		ret = getReading();
		if (ret) {
			FLY_PRINT_ERR("ERROR: Compass failure! First read returned error");
			return ret;
		}
	}

	return FLYMAPLE_SUCCESS;
}

status Pressure::getReading()
{
	int32_t x1, x2, x3, b3, b6, p, t;
	uint32_t b4, b7;

	static uint8_t read_fail_count = 0; /*!< Number of failed sequential measurements */

	getRawReading(&ut, &up);

	/***************** Compute true temperature *****************/

	x1 = ((ut - (int32_t)ac6) * (int32_t)ac5) >> 15;
	x2 = (((int32_t)mc) << 11) / ((int32_t)x1 + (int32_t)md);
	b5 = x1 + x2;

	t = ((b5 + 8) >> 4);

	/****************** Compute true pressure ******************/

	b6 = b5 - 4000;

	x1 = (b2 * ((b6 * b6) >> 12)) >> 11;
	x2 = ((int32_t)ac2 * b6) >> 11;
	x3 = x1 + x2;
	b3 = ((((int32_t)ac1 * 4 + x3) << OSS) + 2) >> 2;

	x1 = ((int32_t)ac3 * b6) >> 13;
	x2 = ((int32_t)b1 * ((b6 * b6) >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	b4 = ((uint32_t)ac4 * (uint32_t)(x3 + 32768)) >> 15;

	b7 = ((uint32_t)(up - b3) * (50000 >> OSS));
	if (b7 < 0x80000000)
		p = (b7 << 1) / b4;
	else
		p = (b7 / b4) << 1;

	x1 = (p >> 8) * (p >> 8);
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	p += (x1 + x2 + 3791) >> 4;

	// Sanity check the resulting values
	if (up == 0 || p < 30000 || p > 120000 || t < -500 || t > 700) {
		// Something is wrong, this would be > 9000 feet or < -1000 feet.
		//    or temperature < -50 degrees C or temperature > 70 degrees C
		read_fail_count++;

		FLY_PRINT_ERR("WARNING: Pressure sensor read failure - value out of range");
		FLY_PRINT("up = ");
		FLY_PRINTLN(up);
		FLY_PRINT("p = ");
		FLY_PRINTLN(p);
		FLY_PRINT("t = ");
		FLY_PRINTLN(t);

		if (read_fail_count >= 5) {
			FLY_PRINT_ERR("ERROR: Pressure sensor failure! Last 5 values out of range");
			return FLYMAPLE_ERR_PRESSURE_FAIL;
		}

		return FLYMAPLE_WARN_PRESSURE_READ_FAIL;
	}

	/*
	 * Update the global values
	 *
	 * Probably no need for a lock, since temperature and pressure values out of sync
	 *    are not likely to crash the drone
	 */
	temperature = t;
	pressure = p;

	read_fail_count = 0;
	return FLYMAPLE_SUCCESS;
}

float Pressure::computeAltitude()
{
	return (44330 * (1 - pow(((float)pressure / (float)PRESSURE_AT_SEALEVEL_IN_PA), 1.0 / 5.255)));
}
