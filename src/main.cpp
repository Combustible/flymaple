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
#include "GlobalXYZ.h"
#include "Pressure.h"
#include "Compass.h"
#include "Accelerometer.h"
#include "Gyroscope.h"
#include "Sensor.h"
#include "Motor.h"
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

int global_error = 0;

// TODO: Needs error handling somehow
void sensor_loop(void *pvParameters)
{
	portTickType xLastWakeTime;
	const portTickType xFrequency = 50;

	/* Initialize sensors */
	global_error |= Compass::init();
	global_error |= Pressure::init();
	global_error |= Accelerometer::init();
	global_error |= Gyroscope::init();

	FLY_PRINTLN("Sensor initialization complete");

	xLastWakeTime = xTaskGetTickCount();

	while (1) {
		// Wait for the next cycle.
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

		global_error |= Compass::getReading();
		global_error |= Pressure::getReading();
		global_error |= Accelerometer::getReading();
		global_error |= Gyroscope::getReading();

		GlobalXYZ::update();
	}
}

void print_loop(void *pvParameters)
{
	portTickType xLastWakeTime;
	const portTickType xFrequency = 150;

	// Wait a bit so any initialization messages can be read
	vTaskDelay(5000 / portTICK_RATE_MS);

	xLastWakeTime = xTaskGetTickCount();

	while (1) {
		// Wait for the next cycle.
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

		if (!global_error) {
			CLEAR_SCREEN();

			// Print data
			FLY_PRINT("Comp X: ");
			FLY_PRINTLN(Compass::x, 5);
			FLY_PRINT("Comp Y: ");
			FLY_PRINTLN(Compass::y, 5);
			FLY_PRINT("Comp Z: ");
			FLY_PRINTLN(Compass::z, 5);

			FLY_PRINT("Temp: ");
			FLY_PRINTLN(Pressure::temperature);
			FLY_PRINT("Pres: ");
			FLY_PRINTLN(Pressure::pressure);
			FLY_PRINT("Alt : ");
			FLY_PRINTLN(Pressure::computeAltitude(), 5);

			FLY_PRINT("Rel : ");
			FLY_PRINTLN(GlobalXYZ::rel_height);

			FLY_PRINT("Acc X:");
			FLY_PRINTLN(Accelerometer::x);
			FLY_PRINT("Acc Y:");
			FLY_PRINTLN(Accelerometer::y);
			FLY_PRINT("Acc Z:");
			FLY_PRINTLN(Accelerometer::z);

			FLY_PRINT("Gyr X:");
			FLY_PRINTLN(Gyroscope::x);
			FLY_PRINT("Gyr Y:");
			FLY_PRINTLN(Gyroscope::y);
			FLY_PRINT("Gyr Z:");
			FLY_PRINTLN(Gyroscope::z);

			FLY_PRINT("Up X:");
			FLY_PRINTLN(GlobalXYZ::up[0]);
			FLY_PRINT("Up Y:");
			FLY_PRINTLN(GlobalXYZ::up[1]);
			FLY_PRINT("Up Z:");
			FLY_PRINTLN(GlobalXYZ::up[2]);

			uint16_t motorspeed[4];
			Motor::getspeed(motorspeed);
			FLY_PRINT("Spd 0:");
			FLY_PRINTLN(motorspeed[0]);
			FLY_PRINT("Spd 1:");
			FLY_PRINTLN(motorspeed[1]);
			FLY_PRINT("Spd 2:");
			FLY_PRINTLN(motorspeed[2]);
			FLY_PRINT("Spd 3:");
			FLY_PRINTLN(motorspeed[3]);

			FLY_PRINTLN();
			FLY_PRINTLN("Commands: i e d a ; ' '");

			if (SerialUSB.available()) {
				uint8 input = SerialUSB.read();
				FLY_PRINTLN((char)input);

				switch (input) {
				case 'i':
					FLY_PRINTLN("Initializing motors");
					Motor::init();

					break;
				case 'e':
					FLY_PRINTLN("Enabling motors");
					Motor::enable();

					break;
				case 'd':
					FLY_PRINTLN("Disabling motors");
					Motor::disable();

					break;
				case 'a':
					Motor::update(MOTOR_COMPUTE_NEW_SPEED(motorspeed[0], 100));

					break;
				case ';':
					Motor::update(MOTOR_COMPUTE_NEW_SPEED(motorspeed[0], -100));

					break;
				case ' ':
					FLY_PRINTLN("Stopping motors");

					Motor::stop();

					break;
				default: // -------------------------------
					FLY_PRINT("Unexpected byte: 0x");
					FLY_PRINT((int)input, HEX);
				}

			}
			FLY_PRINT("> ");
		}
	}
}

/* Please Do Not Remove & Edit Following Code */
void loop(void *pvParameters)
{
	while (1) {
		while (SerialUSB.available()) {
			uint8 input = SerialUSB.read();
			FLY_PRINTLN((char)input);

			switch (input) {
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

