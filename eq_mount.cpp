#include <Arduino.h>
#include "CA4998.hpp"

// Use CA4998.h micro step controller to control a camera mount
// stepper motor.
//   - CA4998.h uses Timer1 to step the motor

// Joystick analog position selects speed and direction.
//   - NORMAL operation is near neutral position.
//   - moving away from neutral position speeds motor in either direction.
// Joystick discrete switch toggles lower power sleep mode.

typedef enum {
	REVERSE_FAST_2,
	REVERSE_FAST_1,
	FORWARD_NORMAL,    // joystick in neutral position
	FORWARD_FAST_1,
	FORWARD_FAST_2
} OpModeType;
OpModeType current_mode = FORWARD_NORMAL;

// Joystick levels select OpMode, 0 to 1023 reported by analogRead
const int POT_NEUTRAL = 485; // as measured
const int DEAD_BAND  = 30;
const int POT_LOW_2  = 5;
const int POT_LOW_1  = POT_NEUTRAL - DEAD_BAND;
const int POT_HIGH_1 = POT_NEUTRAL + DEAD_BAND;
const int POT_HIGH_2 = 1020;

// The motor speed of each mode is set by PULSE_US and STEP_MODE
const int PULSE_NORMAL_US = 97389;
const int PULSE_FAST1_US = 400;
const int PULSE_FAST2_US = 50;

const CA4998::StepType NORMAL_STEP_MODE = CA4998::SIXTEENTH_STEP;
const CA4998::StepType FAST1_STEP_MODE  = CA4998::SIXTEENTH_STEP;
const CA4998::StepType FAST2_STEP_MODE  = CA4998::SIXTEENTH_STEP;

// Assign analog input channels
const int JOYSTICK_SWITCH = A0; // Analog to use internal pull up.
const int JOYSTICK_INPUT  = A6;

// Construct the driver object, inputs select DIO channels (or A0-A5)
CA4998 motor_driver(
		4, 5,            // pin7_step, pin8_dir
		6, 7, 8, // pin2_m1, pin3_m2, pin4_m3
		9, A1);                 // pin5_reset, pin6_sleep

// Convert analog read of potentiometer, to operating mode
OpModeType selectOpMode(unsigned int input_pot){
	if (input_pot <= POT_LOW_2) {
		return(REVERSE_FAST_2);  // pot <= LOW2
	} else if (input_pot <= POT_LOW_1) {
		return(REVERSE_FAST_1);  // LOW2 < pot <= LOW1
	} else if (input_pot <= POT_HIGH_1) {
		return(FORWARD_NORMAL);  // LOW1 < pot <= HIGH1
	} else if (input_pot <= POT_HIGH_2) {
		return(FORWARD_FAST_1);  // HIGH1 < pot < HIGH2
	} else {
		return(FORWARD_FAST_2);  // pot > HIGH2
	}
}

// Convert operating mode to motor step period
unsigned long step_period_us(OpModeType mode)
{
	if (mode == FORWARD_FAST_1 || mode == REVERSE_FAST_1){
		return PULSE_FAST1_US;
	} else if (mode == FORWARD_FAST_2 || mode == REVERSE_FAST_2) {
		return PULSE_FAST2_US;
	} else {
		return PULSE_NORMAL_US;
	}
}

// Convert operating mode to motor step direction
CA4998::DirectionType step_direction(OpModeType mode) {
	if (mode == FORWARD_NORMAL || mode == FORWARD_FAST_1 || mode == FORWARD_FAST_2) {
		return CA4998::CLOCKWISE;
	} else {
		return CA4998::COUNTER_CLOCK;
	}
}

// Convert operating mode to motor controller micro step mode
CA4998::StepType step_mode(OpModeType mode) {
	if (mode == FORWARD_FAST_1 || mode == REVERSE_FAST_1){
		return FAST1_STEP_MODE;
	} else if (mode == FORWARD_FAST_2 || mode == REVERSE_FAST_2) {
		return FAST2_STEP_MODE;
	} else {
		return NORMAL_STEP_MODE;
	}
}

void setup() {
	Serial.begin(9600);

	pinMode(JOYSTICK_SWITCH, INPUT_PULLUP);  // for joystick switch input

	Serial.print("Neutral Pot = ");
	Serial.println(analogRead(A6));

	current_mode = FORWARD_NORMAL;
	motor_driver.init(
			step_mode(current_mode),
			step_direction(current_mode));
	motor_driver.start(step_period_us(current_mode));
}

void loop() {

	OpModeType next_mode = FORWARD_NORMAL;

	int sleep = analogRead(A0);

	//Debounce the pushbutton
	static bool sleep_mode = false;
	const int debounce_ms = 50;
	static unsigned long push_ms;
	unsigned long release_ms;
	static bool push_active = false;

	if(sleep < 500)
	{
		push_ms = millis();
		push_active = true;
	}
	else
	{
		if(push_active){
			release_ms=millis();
			if(release_ms-push_ms > debounce_ms){
				push_active=false;
				sleep_mode=!sleep_mode;
			}
		}
	}

	if (sleep_mode)
	{
		motor_driver.sleep();
	}
	else // not sleep_mode
	{
		motor_driver.wake();

		// Select operating mode based on pot position
		next_mode = selectOpMode(analogRead(JOYSTICK_INPUT));

		// change modes
		if (next_mode != current_mode)
		{
			// stop() can be expensive, so only change modes if necessary
			if (step_mode(next_mode) != step_mode(current_mode) )
			{
				// waits up to 32 timer 1 pulses, allowing micro stepping
				// driver state to return to HOME position
				motor_driver.stop();
				// Only change modes in HOME position
				motor_driver.setStepMode(step_mode(next_mode));
			}
			motor_driver.setDirection(step_direction(next_mode));
			motor_driver.start(step_period_us(next_mode));
			current_mode = next_mode;
		}

	} // not sleep_mode
}

