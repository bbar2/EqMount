#include <Arduino.h>
#include "CA4998.hpp"
#include <math.h>
#include "bbLocalLib.hpp"
#include "CShutterControl.hpp"

// Use CA4998.h micro step controller to control a camera mount stepper motor.
//   - To match star motion, camera should rotate 364/365 revs per 24 hours.
//   - Motor is NEMA 17, 200step, with 139 to 1 reducer into a 2 to 1 drive gear.
//   - CA4998.h uses Timer1 to step the motor

// Joystick Y axis analog position selects speed and direction.
//   - NORMAL operation is near joystick neutral position.
//   - moving away from neutral position speeds motor in either direction.
// Joystick discrete switch toggles low power sleep mode.

// TODO - look at Direction constants and PPS.  Direction is now determined by pps sign.
// TODO - seems like Direction constants in here are OBE.

typedef enum {
	REVERSE_FAST_2,
	REVERSE_FAST_1,
	FORWARD_NORMAL,    // joystick in neutral position
	FORWARD_FAST_1,
	FORWARD_FAST_2
} OpModeType;
OpModeType current_mode;


// Motor speed of each mode set by PULSE_US and STEP_MODE
// NORMAL speed needs as many micro steps as possible, so SIXTEENTH_STEP
// FAST speeds up to 1000 RPM are limited by acceleration
// Pulse rate at SIXTEENTH_STEP limits RPM to about 500 RPM.
// Given the following combinations result in the same motor speed:
//   HALF_STEP at 400us,    QUARTER_STEP at 200us,
//   EIGHTH_STEP at 100us,  or SIXTEENTH_STEP at 50us;
// and mode changes have a time performance hit waiting for HOME
// synchronization, I selected SIXTEENTH_STEP modes for NORMAL and FAST1.
// Since FAST1 is so much faster than NORMAL, waiting for HOME does not take
// long, so FAST2 can tolerate a mode change, enabling higher RPMs.
static const CA4998::StepType NORMAL_STEP_MODE = CA4998::SIXTEENTH_STEP;
static const CA4998::StepType FAST1_STEP_MODE  = CA4998::SIXTEENTH_STEP;
static const CA4998::StepType FAST2_STEP_MODE  = CA4998::SIXTEENTH_STEP;

// Normal pulse rate is dependent on target speed and NORMAL_STEP_MODE.
static const float TARGET_REVS_PER_DAY = 364.0f/365.0f;
static const float STEPS_PER_DAY = TARGET_REVS_PER_DAY * 200.0f * 139.0f * 2.0f; // 55,447.6
static const float STEPS_PER_SEC = STEPS_PER_DAY / (24.0f * 60.0f * 60.0f);   // 0.642
static const float NORMAL_PPS = NORMAL_STEP_MODE * STEPS_PER_SEC; // 10.27

// FAST modes are limited by how fast the motor can turn.
// Theoretical NEMA 17 max: 600-1500 RPM: function of accel, controller, and voltage
// Arduino Nano timer ISR min pulse width ~20us, limiting max SIXTEENTH_STEP RPM to ~468
// With QUARTER_STEP and acceleration limits I can get 1000 RPM.
static const float FAST1_TARGET_RPM = 100.0f; //350.0f; //100.0f;
static const float FAST1_STEPS_PER_SEC = FAST1_TARGET_RPM * 200.0f / 60.0f;
static const float FAST1_PPS = FAST1_STEPS_PER_SEC * FAST1_STEP_MODE;

static const float FAST2_TARGET_RPM = 350.0f;
static const float FAST2_STEPS_PER_SEC = FAST2_TARGET_RPM * 200.0f / 60.0f;
static const float FAST2_PPS = FAST2_STEPS_PER_SEC * FAST2_STEP_MODE;

// Assign analog input channels
static const int JOYSTICK_SWITCH = A4; // Analog to use internal pull up.
static const int JOYSTICK_AXIS   = A5;

// Construct the motor driver object, inputs select DIO channels (or A0-A5)
CA4998 motor_driver(
		6, 5,                 // pin7_step, pin8_dir
		11, 10, 9,        // pin2_m1, pin3_m2, pin4_m3
		8, 7, 12); // pin5_reset, pin6_sleep, pin1_enable

CShutterControl shutter_control(3, 4);

// Convert analog read of potentiometer, to operating mode
OpModeType selectOpMode(unsigned int input_pot){
	// Joystick levels select OpMode, 0 to 1023 reported by analogRead
	const int POT_NEUTRAL = 485; // as measured
	const int DEAD_BAND   = 30;
	const int POT_LOW_2   = 5;
	const int POT_LOW_1   = POT_NEUTRAL - DEAD_BAND;
	const int POT_HIGH_1  = POT_NEUTRAL + DEAD_BAND;
	const int POT_HIGH_2  = 1020;

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
CA4998::DirectionType stepDirection(OpModeType mode) {
	// Looking up at sky: stars rotate counter clockwise around north star
	// Looking down at shaft: rotate clockwise, so looking up matches sky
	// Looking down at motor: rotate counter clock, since geared opposite shaft
	if (mode == FORWARD_NORMAL || mode == FORWARD_FAST_1 || mode == FORWARD_FAST_2) {
		return CA4998::FORWARD;
	} else {
		return CA4998::REVERSE;
	}
}

// Convert OpModeType to motor step period
float stepPps(OpModeType mode)
{
	switch (mode)
	{
		case FORWARD_NORMAL:
			return NORMAL_PPS;
		case FORWARD_FAST_1:
			return FAST1_PPS;
		case FORWARD_FAST_2:
			return FAST2_PPS;
		case REVERSE_FAST_1:
			return -FAST1_PPS;
		case REVERSE_FAST_2:
			return -FAST2_PPS;
		default:
			return NORMAL_PPS;
	}
}

// Convert OpModeType to motor controller micro step mode
CA4998::StepType stepMode(OpModeType mode) {
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

	#ifdef MY_DEBUG
	Serial.begin(9600);
	#endif

	DEBUG_PRINT("Neutral Pot = ");
	DEBUG_PRINTLN(analogRead(A6));

	DEBUG_PRINT("NORMAL_PPS = ");
	DEBUG_PRINT(NORMAL_PPS);
	DEBUG_PRINT(" x:");
	DEBUG_PRINTLN(NORMAL_STEP_MODE);

	DEBUG_PRINT("FAST1_PPS = ");
	DEBUG_PRINT(FAST1_PPS);
	DEBUG_PRINT(" x:");
	DEBUG_PRINTLN(FAST1_STEP_MODE);

	DEBUG_PRINT("FAST2_PPS = ");
	DEBUG_PRINT(FAST2_PPS);
	DEBUG_PRINT(" x:");
	DEBUG_PRINTLN(FAST2_STEP_MODE);
	delay(1000);

	current_mode = FORWARD_NORMAL;

	motor_driver.init(
			stepMode(current_mode),
			stepDirection(current_mode));
	motor_driver.start(stepPps(current_mode));

	shutter_control.init();
}


void loop() {
	static debounceBool sleep_switch;
	static bool timer_running = true;

	// JOYSTICK_SWITCH is pulled up internally so it will return
	// about 1023 when open, and about 0 when closed.
	bool raw_input = (analogRead(JOYSTICK_SWITCH) < 500);
	bool sleep_mode = sleep_switch.toggleOnFallingEdge(raw_input);

	if (sleep_mode)
	{
		if (timer_running) {  // need to go to sleep
			motor_driver.stop();
			motor_driver.sleep();
			timer_running = false;
			digitalWrite(LED_BUILTIN, HIGH);
			shutter_control.stop();
		}
	}
	else // not sleep_mode
	{
		if (!timer_running){  // need to wake up
			motor_driver.start(stepPps(current_mode));
			motor_driver.wake();
			timer_running = true;
			digitalWrite(LED_BUILTIN, LOW);
			shutter_control.begin();
		}

		// Select operating mode based on pot position
		OpModeType next_mode = selectOpMode(analogRead(JOYSTICK_AXIS));

		// change OpMode
		if (next_mode != current_mode)
		{
			// setStepMode() can be expensive, so only change step mode if necessary
			if (stepMode(next_mode) != stepMode(current_mode) )
			{
				// waits up to 64 pulses for A4998 translator to return to HOME position
				motor_driver.setStepMode(stepMode(next_mode));
			}
			motor_driver.changePPS(stepPps(next_mode));
			current_mode = next_mode;
		}

		// Take shots using schedule in shutter_control
		shutter_control.run();

	} // end not sleep_mode
}

