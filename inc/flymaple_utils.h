/*
 * flymaple_print.h
 *
 *  Created on: Aug 13, 2014
 *      Author: bmarohn
 */

#ifndef FLYMAPLE_PRINT_H_
#define FLYMAPLE_PRINT_H_


#define BIG_ENDIAN_INT16_FROM_PTR(ptr) \
	((int16_t)(((uint16_t)(*((uint8_t *)ptr)) << 8) | ((uint16_t)(*(((uint8_t *)ptr) + 1)))))
#define BIG_ENDIAN_UINT16_FROM_PTR(ptr) \
	(((uint16_t)(*((uint8_t *)ptr)) << 8) | ((uint16_t)(*(((uint8_t *)ptr) + 1))))
#define LITTLE_ENDIAN_INT16_FROM_PTR(ptr) \
	((int16_t)(((uint16_t)(*((uint8_t *)ptr))) | ((uint16_t)(*(((uint8_t *)ptr) + 1)) << 8)))
#define LITTLE_ENDIAN_UINT16_FROM_PTR(ptr) \
	(((uint16_t)(*((uint8_t *)ptr))) | ((uint16_t)(*(((uint8_t *)ptr) + 1)) << 8))


#ifdef NDEBUG

#define FLY_PRINT(...)
#define FLY_PRINTLN(...)
#define FLY_PRINT_ERR(...)

#else /* NDEBUG */

#define FLY_PRINT(...)         SerialUSB.print(__VA_ARGS__)
#define FLY_PRINTLN(...)       SerialUSB.println(__VA_ARGS__)

#define FLY_PRINT_ERR(...)             \
	do {                             \
		SerialUSB.print(__VA_ARGS__);          \
		SerialUSB.print(" | ");      \
		SerialUSB.print(__FILE__);   \
		SerialUSB.print(":");        \
		SerialUSB.println(__LINE__); \
	} while(0)

#endif /* NDEBUG */


#endif /* FLYMAPLE_PRINT_H_ */
