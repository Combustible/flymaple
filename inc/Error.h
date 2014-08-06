/** TODO: License */

#ifndef __ERROR_H__
#define __ERROR_H__

enum status {
	FLYMAPLE_SUCCESS = 0,
	FLYMAPLE_ERR_COMPASS_FAIL,
	FLYMAPLE_WARN_COMPASS_READ_FAIL,
	FLYMAPLE_WARN_COMPASS_DATA_OVERFLOW,

	FLYMAPLE_ERR_PRESSURE_FAIL,
	FLYMAPLE_WARN_PRESSURE_READ_FAIL,

};


#endif