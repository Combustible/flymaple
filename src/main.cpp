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
#include "Sensor.h"

#define PWM_PIN 2
#define STACK_SIZE 500


void setup()
{
	/* Set up the LED to blink  */
	pinMode(BOARD_LED_PIN, OUTPUT);

	/* Turn on PWM on pin PWM_PIN */
	pinMode(PWM_PIN, PWM);
	pwmWrite(PWM_PIN, 0x8000);

	/* Send a message out USART2  */
	Serial2.begin(9600);

	/* Initialize sensors */
	Compass::init();
	Pressure::init();
}

// TODO: Needs error handling somehow
void sensor_loop(void * pvParameters) {
	portTickType xLastWakeTime;
	const portTickType xFrequency = 100;

	xLastWakeTime = xTaskGetTickCount ();

	while (1) {
		// Wait for the next cycle.
		vTaskDelayUntil( &xLastWakeTime, xFrequency );

		Compass::getReading();
		Pressure::getReading();
	}
}

void print_loop() {
	portTickType xLastWakeTime;
	const portTickType xFrequency = 500;

	xLastWakeTime = xTaskGetTickCount ();

	while (1) {
		// Wait for the next cycle.
		vTaskDelayUntil( &xLastWakeTime, xFrequency );

		SerialUSB.print("X: ");
		SerialUSB.println(Compass::x, 5);
		SerialUSB.print("Y: ");
		SerialUSB.println(Compass::y, 5);
		SerialUSB.print("Z: ");
		SerialUSB.println(Compass::z, 5);

		SerialUSB.print("Temperature: ");
		SerialUSB.println(Pressure::temperature);
		SerialUSB.print("Pressure: ");
		SerialUSB.println(Pressure::pressure);
		SerialUSB.print("Altitude: ");
		SerialUSB.println(Pressure::computeAltitude(), 5);
		SerialUSB.print("ut: ");
		SerialUSB.println(Pressure::debug_get_ut());
		SerialUSB.print("up: ");
		SerialUSB.println(Pressure::debug_get_up());
	}
}

/* Please Do Not Remove & Edit Following Code */
void loop(void * pvParameters)
{
	while (1) {
		while (SerialUSB.available()) {
			uint8 input = SerialUSB.read();
			SerialUSB.println((char)input);

			switch (input) {
			case ' ':
				SerialUSB.println("spacebar, nice!");

				print_loop();

				break;
			case 'c':
				while (1) {
					Compass::calibrate();
				}

				break;
			default: // -------------------------------
				SerialUSB.print("Unexpected byte: 0x");
				SerialUSB.print((int)input, HEX);
			}

			SerialUSB.print("> ");
			taskYIELD();
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

	xTaskCreate(sensor_loop, (const signed char *)"sensor_loop", STACK_SIZE, NULL, 1, NULL);
	xTaskCreate(loop, (const signed char *)"loop", STACK_SIZE, NULL, 1, NULL);

	vTaskStartScheduler();
//	loop(NULL);

	return 0;
}

