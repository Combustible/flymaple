/**
* @file   ADXL345.pde
* @author tonghuix <tonghuix@tonghuix-1011PX>
* @date   Thu Apr 12 16:30:46 2012
* 
* @brief  Acceleratemeter
* 
* 
*/

#include "ADXL345.h"
#include "wirish.h"
#include "config.h"
#include "misc.h"

static short a_offset[] = {0,0,0};

void initAcc() {
  	/* Read the DEVID from the accelerometer */
  	uint8 dev_id = 0;
  	readFrom(ACC, 0x00, 1, &dev_id);

#ifdef DEBUG
  	SerialUSB.print("Accelerometer DEVID: ");
  	printHex(dev_id);
  	SerialUSB.print("\r\n");
#endif

  	/* Make sure the DEVID is correct */
    if (dev_id != XL345_DEVID) {
		SerialUSB.println("Error, incorrect xl345 devid!");
		SerialUSB.println("Halting program, hit reset...");
		waitForButtonPress(0);
    }

    //调用 ADXL345
    writeTo(ACC, ADXLREG_POWER_CTL, 0x00); //清零 
    writeTo(ACC, ADXLREG_POWER_CTL, 0xff);//休眠
    writeTo(ACC, ADXLREG_POWER_CTL, 0x08); //仅开启工作模式
//    //设定在 +-2g 时的默认读数
//    float accumulator[] = {0,0,0};
//    for(int i = 0 ; i < 100 ; i++) {
//	    short acc[3];
//	    getAccelerometerData(acc);
//	    accumulator[0] += acc[0];
//	    accumulator[1] += acc[1];
//	    accumulator[2] += acc[2];
//    }
//    for(int i = 0 ; i < 3 ; i++) accumulator[i] /= 100;
//    accumulator[2] -= 248;
//    for(int i = 0 ; i < 3 ; i++) a_offset[i] = accumulator[i];
}

void getAccelerometerData(int16 * result) {
    int16 regAddress = ADXLREG_DATAX0;    //加速度传感器ADXL345第一轴的数据地址
    uint8 buff[A_TO_READ];

    readFrom(ACC, regAddress, A_TO_READ, buff); //读取加速度传感器ADXL345的数据

    //每个轴的读数有10位分辨率，即2个字节.  
    //我们要转换两个bytes为一个int变量
    result[0] = (((int16)buff[1]) << 8) | (buff[0] + a_offset[0]);   
    result[1] = (((int16)buff[3]) << 8) | (buff[2] + a_offset[1]);
    result[2] = (((int16)buff[5]) << 8) | (buff[4] + a_offset[2]);
}
