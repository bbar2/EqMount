#include <Arduino.h>
#include "CA4998.hpp"

CA4998 motor_driver(
		4, 5,           // pin7_step, pin8_dir
		6, 7, 8,   // pin2_m1, pin3_m2, pin4_m3
		9);//, PIN_A4, PIN_A3 ); // pin5_reset, pin1_enable, pin6_sleep

typedef enum {
	CCW_2,
	CCW_1,
	CW_NORMAL,
	CW_1,
	CW_2
} OpModeType;

OpModeType current_mode = CW_NORMAL;

const int PULSE_FAST2_US = 400;
const int PULSE_FAST1_US = 400;
const int PULSE_NORMAL_US = 10000;

const CA4998::StepType NORMAL_STEP_MODE = CA4998::SIXTEENTH_STEP;
const CA4998::StepType FAST1_STEP_MODE = CA4998::QUARTER_STEP;
const CA4998::StepType FAST2_STEP_MODE = CA4998::HALF_STEP;

static int motor_pulse_output = LOW;
void timerInterrupt() {
	if (motor_pulse_output == LOW) {

	}
}

unsigned long step_period_us(OpModeType mode)
{
	if (mode == CW_NORMAL) {
		return PULSE_NORMAL_US;
	} else if (mode == CW_1 || mode == CCW_1){
		return PULSE_FAST1_US;
	} else if (mode == CW_2 || mode == CCW_2) {
		return PULSE_FAST2_US;
	} else {
		return PULSE_NORMAL_US;
	}
}

CA4998::DirectionType step_direction(OpModeType mode) {
	if (mode == CW_NORMAL || mode == CW_1 || mode == CW_2) {
		return CA4998::CLOCKWISE;
	} else {
		return CA4998::COUNTER_CLOCK;
	}
}

CA4998::StepType step_mode(OpModeType mode) {
	if (mode == CW_NORMAL){
		return NORMAL_STEP_MODE;
	} else if (mode == CW_1 || mode == CCW_1){
		return FAST1_STEP_MODE;
	} else if (mode == CW_2 || mode == CCW_2) {
		return FAST2_STEP_MODE;
	} else {
		return NORMAL_STEP_MODE;
	}
}

void setup() {
	Serial.begin(9600);
	for (int i = 0; i < 100; i+=6)
	{
		digitalWrite(LED_BUILTIN, HIGH);
		delay(i);
		digitalWrite(LED_BUILTIN, LOW);
		delay(i);
	}



	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(A0, INPUT_PULLUP);
	pinMode(A1, OUTPUT);

	current_mode = CW_NORMAL;
	motor_driver.init(step_mode(current_mode), step_direction(current_mode));
}

void step200(int multiples=1)
{
	for (int num_steps=0; num_steps < 200*multiples; num_steps++){
		motor_driver.step();
		delay(10);
	}
}

void loop() {

	int pot = analogRead(A6);
	int sleep = analogRead(A0);

	//Debounce the pushbutton
	static bool sleep_mode = false;
	const int debounce_ms = 50;
	static unsigned long push_ms;
	unsigned long release_ms;
	static bool push_active = false;

	if(analogRead(A0) < 500)
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


	const int LOW2 = 5;   // CCW_2
	const int LOW1 = 400;   // CCW_1
	const int HIGH1 = 550;  // CW_1
	const int HIGH2 = 1020;  // CW_2

	static bool mode_change_required = true;


	if (sleep_mode)
	{
		digitalWrite(A1, LOW);
	}
	else // not sleep_mode
	{
		digitalWrite(A1, HIGH);

		// See if mode change required
		if (pot < LOW2)
		{
			if (current_mode != CCW_2)
			{
				current_mode = CCW_2;
				mode_change_required = true;
				Serial.print(pot);
				Serial.println("CCW_2");
			}
		} else if (pot >= LOW2 && pot < LOW1)
		{
			if (current_mode != CCW_1)
			{
				current_mode = CCW_1;
				mode_change_required = true;
				Serial.print(pot);
				Serial.println("CCW_1");
			}
		} else if (pot >= LOW2 && pot < HIGH1)
		{
			if (current_mode != CW_NORMAL)
			{
				current_mode = CW_NORMAL;
				mode_change_required = true;
				Serial.print(pot);
				Serial.println("CW_NORMAL");
			}
		} else if (pot >= HIGH1 && pot < HIGH2)
		{
			if (current_mode != CW_1)
			{
				current_mode = CW_1;
				mode_change_required = true;
				Serial.print(pot);
				Serial.println(" CW_1");
			}
		} else // pot >= HIGH2
		{
			if (current_mode != CW_2)
			{
				current_mode = CW_2;
				mode_change_required = true;
				Serial.print(pot);
				Serial.println("CW_2");
			}
		}

		if (mode_change_required)
		{
			motor_driver.init(step_mode(current_mode), step_direction(current_mode));
			mode_change_required = false;
		}

		// Step the motor
		motor_driver.step(step_period_us(current_mode));

	} // not sleep_mode
}

