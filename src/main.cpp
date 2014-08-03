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
#include "Test.h"
#include "GlobalXYZ.h"
#include "Pressure.h"
#include "Compass.h"

#define PWM_PIN 2
#define STACK_SIZE 200


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

/* Please Do Not Remove & Edit Following Code */
void loop(void)
{
	Vector<double> pressure;

	Compass compass;


	while (SerialUSB.available()) {
		uint8 input = SerialUSB.read();
		SerialUSB.println((char)input);

		switch (input) {
		case ' ':
			SerialUSB.println("spacebar, nice!");
//            pressure = Pressure::getReading();
//			SerialUSB.print("Pressure: ");
//			SerialUSB.println((double)pressure(0), 5);
//			SerialUSB.print("Temperature: ");
//			SerialUSB.println((double)pressure(1), 5);
//			SerialUSB.print("Altitude: ");
//			SerialUSB.println((double)pressure(2), 5);

			compass.getReading();
			SerialUSB.print("X: ");
			SerialUSB.println(compass.x, 5);
			SerialUSB.print("Y: ");
			SerialUSB.println(compass.y, 5);
			SerialUSB.print("Z: ");
			SerialUSB.println(compass.z, 5);
			break;

		case 'c':
			while (1) {
				compass.calibrate();
			}
		default: // -------------------------------
			SerialUSB.print("Unexpected byte: 0x");
			SerialUSB.print((int)input, HEX);
		}

		SerialUSB.print("> ");
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

	while (1) {
		loop();
	}
	return 0;
}

