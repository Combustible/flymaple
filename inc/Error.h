/** TODO: License */

#ifndef __ERROR_H__
#define __ERROR_H__

enum status {
	FLYMAPLE_SUCCESS = 0,

	FLYMAPLE_ERR_COMPASS_FAIL = 0x01000000,
	FLYMAPLE_WARN_COMPASS_READ_FAIL,
	FLYMAPLE_WARN_COMPASS_DATA_OVERFLOW,

	FLYMAPLE_ERR_PRESSURE_FAIL = 0x02000000,
	FLYMAPLE_WARN_PRESSURE_READ_FAIL,

	FLYMAPLE_ERR_ACCELEROMETER_FAIL = 0x03000000,
	FLYMAPLE_WARN_ACCELEROMETER_READ_FAIL,

	FLYMAPLE_ERR_GYROSCOPE_FAIL = 0x04000000,
	FLYMAPLE_WARN_GYROSCOPE_READ_FAIL,


};


#endif
