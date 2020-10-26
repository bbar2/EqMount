#include <Arduino.h>
#include "CA4998.hpp"
#include "bbLocalLib.hpp"

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
OpModeType current_mode;

// The motor speed of each mode is set by PULSE_US and STEP_MODE
// 364/365*139*2*200     = 55,447.671 steps per day
// 55447.67 / (24*60*60) = 0.64176 steps per second
// For 16 micro step mode: 0.64176 * 16 = 10.268087 micro steps per second
// or 97,389.1 us per step - round down to 97389
const int PULSE_NORMAL_US = 97389;
const int PULSE_FAST1_US = 200; // 400 something faster
const int PULSE_FAST2_US = 50;  // 50 about as fast as I can make it go.

// I tried lower micro step modes for the FAST speeds.  In the end,
// speed was not limited by pulse rate, and speed changes are more
// reliable with higher micro stepping.  So I could go either:
// HALF_STEP at 400us, QUARTER_STEP at 200us, EIGHTH_STEP at 100us,
// or SIXTEENTH_STEP at 50us.
// Mode changes add complexity due to the need to stop() for HOME
// synchronization, so I settled for all SIXTEENTH_STEP modes.
const CA4998::StepType NORMAL_STEP_MODE = CA4998::SIXTEENTH_STEP;
const CA4998::StepType FAST1_STEP_MODE  = CA4998::SIXTEENTH_STEP;
const CA4998::StepType FAST2_STEP_MODE  = CA4998::SIXTEENTH_STEP;

// Assign analog input channels
const int JOYSTICK_SWITCH = A0; // Analog to use internal pull up.
const int JOYSTICK_AXIS  = A6;

// Construct the driver object, inputs select DIO channels (or A0-A5)
CA4998 motor_driver(
		4, 5,            // pin7_step, pin8_dir
		6, 7, 8, // pin2_m1, pin3_m2, pin4_m3
		9, A1);                 // pin5_reset, pin6_sleep

// Convert analog read of potentiometer, to operating mode
OpModeType selectOpMode(unsigned int input_pot){
	// Joystick levels select OpMode, 0 to 1023 reported by analogRead
	const int POT_NEUTRAL = 485; // as measured
	const int DEAD_BAND  = 30;
	const int POT_LOW_2  = 5;
	const int POT_LOW_1  = POT_NEUTRAL - DEAD_BAND;
	const int POT_HIGH_1 = POT_NEUTRAL + DEAD_BAND;
	const int POT_HIGH_2 = 1020;

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

// Convert OpModeType to motor step direction
CA4998::DirectionType step_direction(OpModeType mode) {
	if (mode == FORWARD_NORMAL || mode == FORWARD_FAST_1 || mode == FORWARD_FAST_2) {
		return CA4998::CLOCKWISE;
	} else {
		return CA4998::COUNTER_CLOCK;
	}
}

// Convert OpModeType to motor step period
unsigned long step_period_us(OpModeType mode)
{
	if (mode == FORWARD_FAST_1 || mode == REVERSE_FAST_1) {
		return PULSE_FAST1_US;
	} else if (mode == FORWARD_FAST_2 || mode == REVERSE_FAST_2) {
		return PULSE_FAST2_US;
	} else {
		return PULSE_NORMAL_US;
	}
}

// Convert OpModeType to motor controller micro step mode
CA4998::StepType step_mode(OpModeType mode) {
	if (mode == FORWARD_FAST_1 || mode == REVERSE_FAST_1) {
		return FAST1_STEP_MODE;
	} else if (mode == FORWARD_FAST_2 || mode == REVERSE_FAST_2) {
		return FAST2_STEP_MODE;
	} else {
		return NORMAL_STEP_MODE;
	}
}

void setup() {

	pinMode(JOYSTICK_SWITCH, INPUT_PULLUP);  // for joystick switch input
	pinMode(LED_BUILTIN, OUTPUT);

//	Serial.begin(9600);
//	Serial.print("Neutral Pot = ");
//	Serial.println(analogRead(A6));

	current_mode = FORWARD_NORMAL;
	motor_driver.init(
			step_mode(current_mode),
			step_direction(current_mode));
	motor_driver.start(step_period_us(current_mode));
}

void loop() {
	static debounceBool sleep_switch;

	// JOYSTICK_SWITCH is pulled up internally so it will return
	// about 1023 when open, and about 0 when closed.
	bool raw_input = (analogRead(JOYSTICK_SWITCH) < 500);
	bool sleep_mode = sleep_switch.toggleOnFallingEdge(raw_input);

	if (sleep_mode)
	{
		digitalWrite(LED_BUILTIN, LOW);
		motor_driver.sleep();
	}
	else // not sleep_mode
	{
		digitalWrite(LED_BUILTIN, HIGH);
		motor_driver.wake();

		// Select operating mode based on pot position
		OpModeType next_mode = selectOpMode(analogRead(JOYSTICK_AXIS));

		// change OpMode
		if (next_mode != current_mode)
		{
			// stop() can be expensive, so only change step mode if necessary
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

