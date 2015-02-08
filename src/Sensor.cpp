#include "i2c.h"
#include "wirish.h"
#include "Sensor.h"
#include "MapleFreeRTOS.h"

/********************** Locally Accessible **********************/

static bool gIsInit = false;

/********************** Globally Accessible **********************/

void Sensor::init()
{
	if (gIsInit != true) {
		i2c_master_enable(I2C1, 0);
		gIsInit = true;
	}
}

void Sensor::read(unsigned char dev_addr, unsigned char read_addr,
                  unsigned char read_length, unsigned char *buffer)
{
#ifndef NDEBUG
	ASSERT(buffer);
#endif
	taskENTER_CRITICAL();

	//把要读的地址写入设备
	buffer[0] = read_addr;
	i2c_msg msgs[1];
	msgs[0].addr = dev_addr;
	msgs[0].flags = 0;
	msgs[0].length = 1;
	msgs[0].data = buffer;
	i2c_master_xfer(I2C1, msgs, 1, 0);
	//从设备读取获得的数据
	msgs[0].addr = dev_addr;
	msgs[0].flags = I2C_MSG_READ;
	msgs[0].length = read_length;
	msgs[0].data = buffer;
	i2c_master_xfer(I2C1, msgs, 1, 0);

	taskEXIT_CRITICAL();
}

void Sensor::write(unsigned char dev_addr, unsigned char write_addr, unsigned char value)
{
	unsigned char buffer[] = {write_addr, value};

	taskENTER_CRITICAL();

	i2c_msg msgs[1];
	msgs[0].addr = dev_addr;
	msgs[0].flags = 0;
	msgs[0].length = 2;
	msgs[0].data = buffer;
	i2c_master_xfer(I2C1, msgs, 1, 0);

	taskEXIT_CRITICAL();
}
