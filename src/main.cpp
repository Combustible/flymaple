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
#include "MotorControl.h"
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
bool global_sensor_init_flag = false;
int slow_ticks = 0;
int fast_ticks = 0;

extern double phi_x[NUM_HIDDEN_LAYER_NODES];
extern double phi_y[NUM_HIDDEN_LAYER_NODES];

extern double perf_index_x;
extern double perf_index_y;

void slow_sensor_loop(void *pvParameters)
{
	portTickType xLastWakeTime;
	const portTickType xFrequency = 66; // ~15 Hz

	xLastWakeTime = xTaskGetTickCount();

	while (1) {
		// Wait for the next cycle.
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

		if (global_sensor_init_flag) {
			Compass::getReading();
			Pressure::getReading();
		}

		slow_ticks++;
	}
}


// TODO: Needs error handling somehow
void sensor_loop(void *pvParameters)
{
	portTickType xLastWakeTime;
	const portTickType xFrequency = 10; // 100 Hz

	/* Initialize sensors */
	Compass::init();
	Pressure::init();
	Accelerometer::init();
	Gyroscope::init();

	global_sensor_init_flag = true;

	FLY_PRINTLN("Sensor initialization complete");

	xLastWakeTime = xTaskGetTickCount();

	while (1) {
		// Wait for the next cycle.
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

		if (global_sensor_init_flag) {
			Accelerometer::getReading();
			Gyroscope::getReading();

			GlobalXYZ::update();
			MotorControl::update();
		}

		fast_ticks++;
	}
}

int32_t ticks_to_stop = 0;

static inline void do_user_input(void)
{
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
			MotorControl::setbasespeed(MOTOR_COMPUTE_NEW_SPEED(MotorControl::getbasespeed(), 100));
			break;
		case ';':
			MotorControl::setbasespeed(MOTOR_COMPUTE_NEW_SPEED(MotorControl::getbasespeed(), -100));
		case '1':
			MotorControl::enablepid();
			//MotorControl::setheight(2.0);
			break;
		case '0':
			MotorControl::disablepid();
			//MotorControl::setheight(-5.0);
			break;
		case 'x':
			ticks_to_stop = 60 * 5;
			break;
		case ' ':
			FLY_PRINTLN("Stopping motors");

			MotorControl::unset_height();

			Motor::disable();

			break;
		default: // -------------------------------
			FLY_PRINT("Unexpected byte: 0x");
			FLY_PRINT((int)input, HEX);
		}
	}
}


void print_loop(void *pvParameters)
{
	portTickType xLastWakeTime;
	const portTickType xFrequency = 200;


	// Wait a bit so any initialization messages can be read
	vTaskDelay(5000 / portTICK_RATE_MS);

	xLastWakeTime = xTaskGetTickCount();

	while (1) {
		// Wait for the next cycle.
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

		// One tick closer to stopping
		ticks_to_stop --;

		CLEAR_SCREEN();

		// Print data
		FLY_PRINT("Ticks:(");
		FLY_PRINT(slow_ticks);
		FLY_PRINT(",");
		FLY_PRINT(fast_ticks);
		FLY_PRINTLN(")");
		slow_ticks = 0;
		fast_ticks = 0;

#if 0
		FLY_PRINT("Comp X: ");
		FLY_PRINTLN(Compass::x, 4);
		FLY_PRINT("Comp Y: ");
		FLY_PRINTLN(Compass::y, 4);
		FLY_PRINT("Comp Z: ");
		FLY_PRINTLN(Compass::z, 4);

		FLY_PRINT("Temp: ");
		FLY_PRINTLN(Pressure::temperature);
		FLY_PRINT("Pres: ");
		FLY_PRINTLN(Pressure::pressure);
		FLY_PRINT("Alt : ");
		FLY_PRINTLN(Pressure::altitude, 4);
#endif

		FLY_PRINT("Rel : ");
		FLY_PRINTLN(GlobalXYZ::rel_height, 4);

#if 0
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
#endif

		FLY_PRINT("Up X:");
		FLY_PRINTLN(GlobalXYZ::up[0], 4);
		FLY_PRINT("Up Y:");
		FLY_PRINTLN(GlobalXYZ::up[1], 4);
		FLY_PRINT("Up Z:");
		FLY_PRINTLN(GlobalXYZ::up[2], 4);

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
		FLY_PRINT("basespeed:");
		FLY_PRINTLN(MotorControl::getbasespeed());
		FLY_PRINT("ticks:");
		FLY_PRINTLN(ticks_to_stop);

		FLY_PRINTLN("phi_x");
		for (int i = 0; i < 6; i ++) {
			FLY_PRINTLN(phi_x[i]);
		}
		FLY_PRINTLN("phi_y");
		for (int i = 0; i < 6; i ++) {
			FLY_PRINTLN(phi_y[i]);
		}
		FLY_PRINT("perf_index_x:");
		FLY_PRINTLN(perf_index_x, 4);
		FLY_PRINT("perf_index_y:");
		FLY_PRINTLN(perf_index_y, 4);


		FLY_PRINTLN();
		FLY_PRINTLN("Commands: i e d 1 0 x a ; ' '");


		/* Notes:
		 *
		 * Takeoff @ around 3300
		 * Safe landing maybe 2900
		 *
		 */

		// Come back to the ground after a period of time
		if ((ticks_to_stop == 150) && (MotorControl::is_height_set() == true)) {
			MotorControl::setheight(-5.0);
		}
		if ((ticks_to_stop == 0) && (MotorControl::is_height_set() == true)) {
			MotorControl::unset_height();

			Motor::disable();
		}

		do_user_input();

		FLY_PRINT("> ");
	}
}

/* Please Do Not Remove & Edit Following Code */
void loop(void *pvParameters)
{
	// Be quiet at start time
	Motor::init();

	while (1) {
		while (SerialUSB.available()) {
			uint8 input = SerialUSB.read();
			FLY_PRINTLN((char)input);

			switch (input) {
			case ' ':
				FLY_PRINTLN("spacebar, nice!");

				xTaskCreate(sensor_loop, (const signed char *)"sensor_loop", STACK_SIZE, NULL, 1, NULL);
				xTaskCreate(slow_sensor_loop, (const signed char *)"slow_sensor_loop", STACK_SIZE, NULL, 1, NULL);
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
	xTaskCreate(vTaskTestOrientationFiltering2, (const signed char *)"testOrientationFiltering2",
	            STACK_SIZE, NULL, 1, NULL);

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

