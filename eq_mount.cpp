#include <Arduino.h>
#include "CA4998.hpp"
#include <math.h>
#include "bbLocalLib.hpp"

// Use CA4998.h micro step controller to control a camera mount stepper motor.
//   - To match star motion, camera should rotate 364/365 revs per 24 hours.
//   - Motor is NEMA 17, 200step, with 139 to 1 reducer into a 2 to 1 drive gear.
//   - CA4998.h uses Timer1 to step the motor

// Joystick Y axis analog position selects speed and direction.
//   - NORMAL operation is near joystick neutral position.
//   - moving away from neutral position speeds motor in either direction.
// Joystick discrete switch toggles low power sleep mode.

typedef enum {
	REVERSE_FAST_2,
	REVERSE_FAST_1,
	FORWARD_NORMAL,    // joystick in neutral position
	FORWARD_FAST_1,
	FORWARD_FAST_2
} OpModeType;
OpModeType current_mode;

// Motor speed of each mode set by PULSE_US and STEP_MODE
// NORMAL speed needs as many micro steps as possible, so SIXTEENTH_STEP.
// FAST speeds up to 1000 RPM are limited by acceleration.
// Pulse rate at SIXTEENTH_STEP limits RPM to about 500 RPM. not limited by pulse rate, and speed changes more reliable
// with higher micro stepping.  Given the following combinations result
// in the same motor speed:
//   HALF_STEP at 400us,    QUARTER_STEP at 200us,
//   EIGHTH_STEP at 100us,  or SIXTEENTH_STEP at 50us;
// and mode changes add complexity due to the need to stop() for HOME
// synchronization, I selected all SIXTEENTH_STEP modes.
static const CA4998::StepType NORMAL_STEP_MODE = CA4998::SIXTEENTH_STEP;
static const CA4998::StepType FAST1_STEP_MODE  = CA4998::SIXTEENTH_STEP;
static const CA4998::StepType FAST2_STEP_MODE  = CA4998::SIXTEENTH_STEP;

// Normal pulse rate is dependent on target speed and NORMAL_STEP_MODE.
static const double TARGET_REVS_PER_DAY = 364.0/365.0;
static const double STEPS_PER_DAY = TARGET_REVS_PER_DAY * 200.0 * 139.0 * 2.0; // 55,447.6
static const double STEPS_PER_SEC = STEPS_PER_DAY / (24.0 * 60.0 * 60.0);   // 0.642
static const double MICRO_STEPS_PER_SEC = NORMAL_STEP_MODE * STEPS_PER_SEC; // 10.27

// FAST modes are limited by how fast the motor can turn.
// Theoretical NEMA 17 max: 600-1500 RPM: function of accel, controller, and voltage
// Arduino Nano timer ISR min pulse width ~20us, limiting max SIXTEENTH_STEP RPM to ~468
// With QUARTER_STEP and acceleration limits I can get 1000 RPM.
static const double FAST1_TARGET_RPM = 200.0; //350; //100.0;
static const double FAST1_STEPS_PER_SEC = FAST1_TARGET_RPM * 200.0 / 60.0;
static const double FAST1_MICRO_STEPS_PER_SEC = FAST1_STEPS_PER_SEC * FAST1_STEP_MODE;

static const double FAST2_TARGET_RPM = 500; //375.0;
static const double FAST2_STEPS_PER_SEC = FAST2_TARGET_RPM * 200.0 / 60.0;
static const double FAST2_MICRO_STEPS_PER_SEC = FAST2_STEPS_PER_SEC * FAST2_STEP_MODE;

static const uint32_t PULSE_NORMAL_US = (lround)(1E06 / MICRO_STEPS_PER_SEC); // 97,389
static const uint32_t PULSE_FAST1_US = (lround)(1E06 / FAST1_MICRO_STEPS_PER_SEC);
static const uint32_t PULSE_FAST2_US = (lround)(1E06 / FAST2_MICRO_STEPS_PER_SEC);

// Assign analog input channels
static const int JOYSTICK_SWITCH = A2; // Analog to use internal pull up.
static const int JOYSTICK_AXIS   = A1;

// Construct the driver object, inputs select DIO channels (or A0-A5)
CA4998 motor_driver(
		11, 12,                 // pin7_step, pin8_dir
		6, 7, 8,        // pin2_m1, pin3_m2, pin4_m3
		9, 10, 5); // pin5_reset, pin6_sleep, pin1_enable

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
CA4998::DirectionType step_direction(OpModeType mode) {
	// Looking up at sky: stars rotate counter clockwise around north star
	// Looking down at shaft: rotate clockwise, so looking up matches sky
	// Looking down at motor: rotate counter clock, since geared opposite shaft
	if (mode == FORWARD_NORMAL || mode == FORWARD_FAST_1 || mode == FORWARD_FAST_2) {
		return CA4998::COUNTER_CLOCK;
	} else {
		return CA4998::CLOCKWISE;
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

	#if 1
	Serial.begin(9600);
	Serial.print("Neutral Pot = ");
	Serial.println(analogRead(A6));

	Serial.print("PULSE_NORMAL_US = ");
	Serial.print(PULSE_NORMAL_US);
	Serial.print(" x:");
	Serial.println(NORMAL_STEP_MODE);

	Serial.print("PULSE_FAST1_US = ");
	Serial.print(PULSE_FAST1_US);
	Serial.print(" x:");
	Serial.println(FAST1_STEP_MODE);

	Serial.print("PULSE_FAST2_US = ");
	Serial.print(PULSE_FAST2_US);
	Serial.print(" x:");
	Serial.println(FAST2_STEP_MODE);
	delay(1000);
	#endif

	current_mode = FORWARD_NORMAL;

	motor_driver.init(
			step_mode(current_mode),
			step_direction(current_mode));

	motor_driver.start(step_period_us(current_mode));
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
		if (timer_running) {
			motor_driver.stop();
			timer_running = false;
			digitalWrite(LED_BUILTIN, HIGH);
			motor_driver.sleep();
		}
	}
	else // not sleep_mode
	{
		if (!timer_running){
			motor_driver.start(step_period_us(current_mode));
			timer_running = true;
			digitalWrite(LED_BUILTIN, LOW);
			motor_driver.wake();
		}

		// Select operating mode based on pot position
		OpModeType next_mode = selectOpMode(analogRead(JOYSTICK_AXIS));

		// change OpMode
		if (next_mode != current_mode)
		{
			// stop() can be expensive, so only change step mode if necessary
			if (step_mode(next_mode) != step_mode(current_mode) )
			{
				Serial.println("THE SLOW ONE");
				// waits up to 32 timer 1 pulses, allowing micro stepping
				// driver state to return to HOME position
				motor_driver.stop();
				// Only change modes in HOME position
				motor_driver.setStepMode(step_mode(next_mode));
				motor_driver.start(step_period_us(next_mode));
			}
			Serial.print("The Quick One: Current Timer Period = ");
			Serial.println(motor_driver.getTimerPeriod());
			motor_driver.setDirection(step_direction(next_mode));
			motor_driver.changePeriod(step_period_us(next_mode));
			current_mode = next_mode;
		}

	} // end not sleep_mode
}

