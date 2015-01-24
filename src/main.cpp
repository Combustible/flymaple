/**
 * @file   main.cpp
 * @author breadbread1984 <breadbread1984@163.com>
 * @date   Sat Jul 21 15:12:00 2012
 *
 * @section DESCRIPTION
 *
 * openDrone Quadcopter Main function File
 *
 * @section LICENSE
 *
 * GPLv3
 */


#include "wirish.h"
#include "MapleFreeRTOS.h"
//#include "Test.h"
//#include "GlobalXYZ.h"
#include "Pressure.h"
#include "Compass.h"
#include "Accelerometer.h"
#include "Sensor.h"
#include "flymaple_utils.h"

#define PWM_PIN 2
#define STACK_SIZE 300


void setup()
{
	/* Set up the LED to blink  */
	pinMode(BOARD_LED_PIN, OUTPUT);

	/* Turn on PWM on pin PWM_PIN */
	pinMode(PWM_PIN, PWM);
	pwmWrite(PWM_PIN, 0x8000);

	/* Send a message out USART2  */
	Serial2.begin(9600);
}

// TODO: Needs error handling somehow
void sensor_loop(void * pvParameters) {
	portTickType xLastWakeTime;
	const portTickType xFrequency = 100;

	/* Initialize sensors */
	Compass::init();
	Pressure::init();
	Accelerometer::init();

	FLY_PRINTLN("Sensor initialization complete");

	xLastWakeTime = xTaskGetTickCount ();

	while (1) {
		// Wait for the next cycle.
		vTaskDelayUntil( &xLastWakeTime, xFrequency );

		Compass::getReading();
		Pressure::getReading();
		Accelerometer::getReading();
	}
}

void print_loop(void * pvParameters) {
	portTickType xLastWakeTime;
	const portTickType xFrequency = 500;

	xLastWakeTime = xTaskGetTickCount ();

	while (1) {
		// Wait for the next cycle.
		vTaskDelayUntil( &xLastWakeTime, xFrequency );

		FLY_PRINT("Compass X: ");
		FLY_PRINTLN(Compass::x, 5);
		FLY_PRINT("Compass Y: ");
		FLY_PRINTLN(Compass::y, 5);
		FLY_PRINT("Compass Z: ");
		FLY_PRINTLN(Compass::z, 5);

		FLY_PRINT("Temperature: ");
		FLY_PRINTLN(Pressure::temperature);
		FLY_PRINT("Pressure: ");
		FLY_PRINTLN(Pressure::pressure);
		FLY_PRINT("Altitude: ");
		FLY_PRINTLN(Pressure::computeAltitude(), 5);

		FLY_PRINT("Accelerometer X:");
		FLY_PRINTLN(Accelerometer::x);
		FLY_PRINT("Accelerometer Y:");
		FLY_PRINTLN(Accelerometer::y);
		FLY_PRINT("Accelerometer Z:");
		FLY_PRINTLN(Accelerometer::z);
	}
}

void set_all_motors(HardwareTimer *timer, uint16_t compare) {
	timer->setCompare(TIMER_CH1, compare);
	timer->setCompare(TIMER_CH2, compare);
	timer->setCompare(TIMER_CH3, compare);
	timer->setCompare(TIMER_CH4, compare);
}


void fly_test(void) {
	pinMode(D12, PWM);
	pinMode(D11, PWM);
	pinMode(D27, PWM);
	pinMode(D28, PWM);
	HardwareTimer timer3(3);

	timer3.pause();
	timer3.setCount(0);

	timer3.setPeriod(20000);
	timer3.setMode(TIMER_CH1, TIMER_PWM);
	timer3.setMode(TIMER_CH2, TIMER_PWM);
	timer3.setMode(TIMER_CH3, TIMER_PWM);
	timer3.setMode(TIMER_CH4, TIMER_PWM);

	uint16_t overflow = timer3.getOverflow();
	uint16_t compare_min = overflow / 20;
	uint16_t compare_max = overflow / 10;
	uint16_t compare = compare_min;

	set_all_motors(&timer3, compare);

	timer3.refresh();
	timer3.resume();

	FLY_PRINT("overflow: ");
	FLY_PRINTLN(overflow);

	FLY_PRINT("compare_min, compare_max: ");
	FLY_PRINT(compare_min);
	FLY_PRINT(",");
	FLY_PRINTLN(compare_max);


	while (1) {
		while (SerialUSB.available()) {
			uint8 input = SerialUSB.read();
			FLY_PRINTLN((char)input);

			switch (input) {
			case '+':
				FLY_PRINTLN("MAXIMUM SPEED");

				compare = compare_max;
				set_all_motors(&timer3, compare);

				break;
			case 'a':
				compare += 100;
				if (compare > compare_max) {
					compare = compare_max;
				}

				FLY_PRINTLN(compare);
				set_all_motors(&timer3, compare);
				break;
			case ';':
				compare -= 100;
				if (compare < compare_min) {
					compare = compare_min;
				}

				FLY_PRINTLN(compare);
				set_all_motors(&timer3, compare);
				break;
			case ' ':
				FLY_PRINTLN("spacebar, nice!");

				compare = compare_min;
				set_all_motors(&timer3, compare);

				break;
			default: // -------------------------------
				FLY_PRINT("Unexpected byte: 0x");
				FLY_PRINT((int)input, HEX);
			}

			FLY_PRINT("> ");
		}
	}
}

/* Please Do Not Remove & Edit Following Code */
void loop(void * pvParameters)
{
	while (1) {
		while (SerialUSB.available()) {
			uint8 input = SerialUSB.read();
			FLY_PRINTLN((char)input);

			switch (input) {
			case 'a':
				fly_test();

				break;
			case ' ':
				FLY_PRINTLN("spacebar, nice!");

				xTaskCreate(sensor_loop, (const signed char *)"sensor_loop", STACK_SIZE, NULL, 1, NULL);
				xTaskCreate(print_loop, (const signed char *)"print_loop", STACK_SIZE, NULL, 1, NULL);

				vTaskStartScheduler();

				break;
			case 'c':
				while (1) {
					Compass::calibrate();
				}

				break;
			default: // -------------------------------
				FLY_PRINT("Unexpected byte: 0x");
				FLY_PRINT((int)input, HEX);
			}

			FLY_PRINT("> ");
		}
	}
#ifdef NOTYET
	xTaskCreate(vTaskUpdateXYZ, (const signed char *)"GlobalXYZ", STACK_SIZE, NULL, 1, NULL);
	xTaskCreate(vTaskTestOrientationFiltering2, (const signed char *)"testOrientationFiltering2", STACK_SIZE, NULL, 1, NULL);

	vTaskStartScheduler();
#endif
}

// -- premain() and main() ----------------------------------------------------

// Force init to be called *first*, i.e. before static object allocation.
// Otherwise, statically allocated objects that need libmaple may fail.
__attribute__((constructor)) void premain()
{
	init();
}

int main(void)
{
	setup();

	loop(NULL);

	return 0;
}

