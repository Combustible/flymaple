#include "wirish.h"
#include "Pressure.h"

using namespace Sensor;

/********************** Locally Accessible **********************/

static const unsigned char OSS = 0;
static const unsigned char BMP085_CONTROL = 0xf4;
static const unsigned char BMP085_CONTROL_OUTPUT = 0xf6;
static const unsigned char READ_TEMPERATURE = 0x2e;
static const unsigned char READ_PRESSURE = 0x34;
static const int MSLP = 101325;
static const int Altitude_cm_Offset = 0;

// EEPROM device constants
static int16_t ac1, ac2, ac3;
static uint16_t ac4, ac5, ac6;
static int16_t b1, b2, mb, mc, md;

// Computed value using temperature reading
static uint16_t b5;

static void getRawReading(unsigned int &up, unsigned short &ut)
{
	//读气压
	unsigned char buffer[3];
	up = 0;
	write(BMP085_I2C_ADDR, BMP085_CONTROL, (READ_PRESSURE + (OSS << 6)));
	delay(2 + (3 << OSS));
	read(BMP085_I2C_ADDR, BMP085_CONTROL_OUTPUT, 3, buffer);
	up = (((unsigned int) buffer[0] << 16) | ((unsigned int) buffer[1] << 8) | (unsigned int) buffer[2]) >> (8 - OSS);
	//读温度
	write(BMP085_I2C_ADDR, BMP085_CONTROL, READ_TEMPERATURE);
	delay(5);
	read(BMP085_I2C_ADDR, BMP085_CONTROL_OUTPUT, 2, buffer);
	ut = ((((short)buffer[0]) << 8) | buffer[1]);
}

static int rawToPressure(unsigned int up)
{
	int x1, x2, x3, b3, b6, p;
	unsigned int b4, b7;

	b6 = b5 - 4000;
	// Calculate B3
	x1 = (b2 * (b6 * b6) >> 12) >> 11;
	x2 = (ac2 * b6) >> 11;
	x3 = x1 + x2;
	b3 = (((((int)ac1) * 4 + x3) << OSS) + 2) >> 2;

	// Calculate B4
	x1 = (ac3 * b6) >> 13;
	x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	b4 = (ac4 * (unsigned int)(x3 + 32768)) >> 15;

	b7 = ((unsigned int)(up - b3) * (50000 >> OSS));
	if (b7 < 0x80000000)
		p = (b7 << 1) / b4;
	else
		p = (b7 / b4) << 1;

	x1 = (p >> 8) * (p >> 8);
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	p += (x1 + x2 + 3791) >> 4;

	return p;
}

static short rawToTemperature(unsigned short ut)
{
	int x1, x2;

	x1 = (((int)ut - (int)ac6) * (int)ac5) >> 15;
	x2 = ((int)mc << 11) / (x1 + md);
	b5 = x1 + x2;

	return ((b5 + 8) >> 4);
}

static double pressureToAltitude(int pressure)
{
	double meters = (44330 * (1 - pow(((float)pressure / (float)MSLP), 0.1903))) + Altitude_cm_Offset;
	return meters;
}

/********************** Globally Accessible **********************/

void Pressure::init()
{
	uint8_t buffer[BMP085_I2C_REG_EEPROM_LEN];

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
}

Vector<double> Pressure::getReading()
{
	unsigned int up;
	unsigned short ut;
	getRawReading(up, ut);
	int pres = rawToPressure(up);
	short temp = rawToTemperature(ut);
	double altitude = pressureToAltitude(pres);
	Vector<double> retVal(3);
	retVal(0) = pres;
	retVal(1) = temp;
	retVal(2) = altitude;
	return retVal;
}
