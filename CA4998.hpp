//
// Created by barry on 10/13/20.
// Header only class to manage A4998 micro stepping driver.
//
// Construct identifying pins wired to Arduino
//   - must specify STEP pin at a minimum.  All others optional.
//
// Crude control of motor speed by calling singleStep() in a loop with delay().
// Better control via timer 1 with start() and stop().
//   singleStep(width_us) - One pulse HIGH for width_us/2 and LOW for width_us/2.
//   start(pulse_per_sec) - Use Timer 1 to send pulses on m_step_pin
//   changePPS(new_pps) - updates pps for already started timer.
//   stop() - stop sending pulses on m_step_pin
//
// setStepMode(1, 2, 4, 8, 16) - Select micro step mode.  1 is Full Steps.
//   Assures that mode changes take place at micro step # 0.
//   Issues a reset to the controller.
//
// sleep() if motor not in use, to minimize power.  Else set to wake();
//
// reset(LOW):
//   - resets state of micro stepper.
//   - Ignores STEP pulses
//
// To set A4998 current limiting trim pot voltage, measuring Vref on trim pot:
//   - Put A4988 in full step mode
//   - While not clocking STEP input
//   - Select Vref (volts) = Motor Current Limit (amps) / 2.5

#pragma once
#include <Arduino.h>
#include "bbLocalLib.hpp"
#include "TimerOne.h" // instantiates at Timer1 object.

class CA4998
{
public: // members
	typedef enum {
		FORWARD = HIGH,
		REVERSE = LOW
	} DirectionType;

	typedef  enum {
		FULL_STEP = 1,
		HALF_STEP = 2,
		QUARTER_STEP = 4,
		EIGHTH_STEP = 8,
		SIXTEENTH_STEP = 16
	} StepType;

private: // members
	int m_enable_pin;
	int m_m1_pin, m_m2_pin, m_m3_pin;
	int m_reset_pin;
	int m_sleep_pin;
	int m_step_pin;
	int m_dir_pin;
	bool m_timer1_running;
	volatile int m_micro_step_num; // volatile because ISR changes it.
	int m_micro_step_home; // 4 * micro step mode
	float m_target_pps;
	volatile float m_current_pps; // volatile because ISR changes it.
	volatile int32_t m_last_accel_ms;
	static CA4998* static_object;  // pointer so static ISR can find this object.

private: // methods

public: // methods
	explicit CA4998( // may be called before setup(), so no system calls
			int pin7_step,    // only pin you must specify
			int pin8_dir = 0, // floats - must be wired if not assigned to a pin.
			int pin2_m1 = 0, int pin3_m2 = 0, int pin4_m3 = 0, // pulled LOW internally
			int pin5_reset  = 0,   // active low: floats - can tie to sleep to pull high
			int pin6_sleep  = 0,   // active low: pulled HIGH internally.
    	int pin1_enable = 0) : // active low: pulled LOW internally
			m_step_pin(pin7_step),
			m_dir_pin(pin8_dir),
			m_m1_pin(pin2_m1), m_m2_pin(pin3_m2), m_m3_pin(pin4_m3),
			m_reset_pin(pin5_reset),
			m_sleep_pin(pin6_sleep),
			m_enable_pin(pin1_enable),
			m_timer1_running(false),
			m_micro_step_num(0),
			m_micro_step_home(0),
			m_target_pps(0),
			m_current_pps(0),
			m_last_accel_ms(0)
	{
		CA4998::static_object = this; // a non c++ target for the timer 1 interrupt
	}

	void init(StepType initial_step_mode = FULL_STEP,
					 DirectionType initial_dir   = FORWARD) {

		// Configure used arduino IO pins as outputs
		pinMode(m_step_pin, OUTPUT);
		if (m_dir_pin)pinMode(m_dir_pin, OUTPUT);
		if (m_m1_pin)pinMode(m_m1_pin, OUTPUT);
		if (m_m2_pin)pinMode(m_m2_pin, OUTPUT);
		if (m_m3_pin)pinMode(m_m3_pin, OUTPUT);
		if (m_enable_pin)pinMode(m_enable_pin, OUTPUT);
		if (m_reset_pin)pinMode(m_reset_pin, OUTPUT);
		if (m_sleep_pin)pinMode(m_sleep_pin, OUTPUT);

		// initialize outputs to active state
		this->setStepLevel(LOW);               // Ready for LOW to HIGH transition
		this->setDirection(initial_dir);
		this->setStepMode(initial_step_mode);
		this->reset();                         // Step translator to home state
		this->wake();
		this->enable();

		// Prepare for timer operation
		CA4998::static_object = this; // TODO Remove this, since constructor sets it.
		Timer1.attachInterrupt(CA4998::staticTimerFunc);
	};

	float currentPPS() const {return m_current_pps;};

	void disable() const { if(m_enable_pin) digitalWrite(m_enable_pin, HIGH); };

	void enable() const { if(m_enable_pin) digitalWrite(m_enable_pin, LOW); };

	void setStepMode(StepType step_mode)  // must be set 20ns before (& held 20ns after) step()
	{
		// If timer running, only change modes from HOME position - i.e. every 4 complete full steps
		while(m_timer1_running && m_micro_step_num != 0){};

		switch(step_mode)
		{
			case HALF_STEP:
				if(m_m1_pin)digitalWrite(m_m1_pin, HIGH);
				if(m_m2_pin)digitalWrite(m_m2_pin, LOW);
				if(m_m3_pin)digitalWrite(m_m3_pin, LOW);
				m_micro_step_home = 2 * 4; // *4 to complete a full cycle back to HOME state
				break;

			case QUARTER_STEP:
				if(m_m1_pin)digitalWrite(m_m1_pin, LOW);
				if(m_m2_pin)digitalWrite(m_m2_pin, HIGH);
				if(m_m3_pin)digitalWrite(m_m3_pin, LOW);
				m_micro_step_home = 4 * 4;
				break;

			case EIGHTH_STEP:
				if(m_m1_pin)digitalWrite(m_m1_pin, HIGH);
				if(m_m2_pin)digitalWrite(m_m2_pin, HIGH);
				if(m_m3_pin)digitalWrite(m_m3_pin, LOW);
				m_micro_step_home = 8 * 4;
				break;

			case SIXTEENTH_STEP:
				if(m_m1_pin)digitalWrite(m_m1_pin, HIGH);
				if(m_m2_pin)digitalWrite(m_m2_pin, HIGH);
				if(m_m3_pin)digitalWrite(m_m3_pin, HIGH);
				m_micro_step_home = 16 * 4;
				break;

			default:  // Full Step
				if(m_m1_pin)digitalWrite(m_m1_pin, LOW);
				if(m_m2_pin)digitalWrite(m_m2_pin, LOW);
				if(m_m3_pin)digitalWrite(m_m3_pin, LOW);
				m_micro_step_home = 1 * 4;
		}
	};

	void setDirection(DirectionType dir_level) const {
		if(m_dir_pin)digitalWrite(m_dir_pin, dir_level);
		DEBUG_PRINT("dir_level = ");
		DEBUG_PRINTLN(dir_level);
	};

	void reset() {
		if(m_reset_pin) {
			digitalWrite(m_reset_pin, LOW);
			delayMicroseconds(2);
			digitalWrite(m_reset_pin, HIGH);
		  delayMicroseconds(2);
		}
		m_micro_step_num = 0; // Home state
	};

	void sleep() const { if(m_sleep_pin)digitalWrite(m_sleep_pin, LOW); };

	void wake() const { if(m_sleep_pin)digitalWrite(m_sleep_pin, HIGH);	};

	void setStepLevel(int step_level) const {
		digitalWrite(m_step_pin, step_level);
	};

	// Use this when not using the start() and stop() timer approach.
	// This does not assure mode changes take place at micro step #0.
	void singleStep(uint32_t pulse_width_us = 200) const {

		uint32_t half_width = pulse_width_us / 2;

		digitalWrite(m_step_pin, HIGH);
		myDelayUs(half_width);
		digitalWrite(m_step_pin, LOW);
		myDelayUs(half_width);
	}

	// Timer function toggles the pulse input
	static void staticTimerFunc();

	void start(float pps)
	{
		// 2 ticks per period for rising and falling edge of 50% duty cycle pulse
		m_target_pps = pps;
		m_current_pps = pps;
		// TODO should probably do a setDirection here.  Then remove it from initIO()
		uint32_t half_period_us = (1e6/2) / pps; // TODO rewrite this as 1e6  / pps / 2
		Timer1.initialize(half_period_us);
		Timer1.start();
		m_last_accel_ms = millis();
		m_timer1_running = true;
	}

	void changePPS(float new_pps)
	{
		m_target_pps = new_pps;
	}

	void stop()
	{
		Timer1.stop();
		m_timer1_running = false;
	}
};

/// Following static member and methods are per class, and must be defined outside
/// of class declaration.
CA4998* CA4998::static_object = nullptr;

// Timer function toggles the step output to the motor
void CA4998::staticTimerFunc()
{
	static bool pulse_level_high = false;

	if(static_object) {
		if (pulse_level_high == false)
		{
			digitalWrite(static_object->m_step_pin, HIGH); // motor outputs change now
			pulse_level_high = true;

			// Count rising pulses to sync mode changes with the HOME state
			if (++static_object->m_micro_step_num >= static_object->m_micro_step_home)
			{
				static_object->m_micro_step_num = 0;
			}

		} else // pulse_level_high == true
		{
			digitalWrite(static_object->m_step_pin, LOW);
			pulse_level_high = false;

			const int32_t accel_dt_ms = 50;
			const float accel_pps = 15000; // Acceleration ramp value
			const float accel_ppdt = accel_pps * accel_dt_ms / 1000;
			const float pps_epsilon = 0.001;

			// Acceleration math on falling edges every accel_dt_ms
			int32_t current_time_ms = millis();
			if (current_time_ms - static_object->m_last_accel_ms > accel_dt_ms)
			{
				static_object->m_last_accel_ms = current_time_ms;

				float current_pps = static_object->m_current_pps;
				float target_pps = static_object->m_target_pps;
				if (fabs(current_pps - target_pps) < pps_epsilon)
				{
					// no acceleration required.
					digitalWrite(LED_BUILTIN, LOW);
				}
				else
				{
					// acceleration required
					digitalWrite(LED_BUILTIN, HIGH);
					if (current_pps < target_pps) {
						current_pps = min(target_pps, current_pps + accel_ppdt);
					}
					else {
						current_pps = max(target_pps, current_pps - accel_ppdt);
					}

					// Update timer period
					uint32_t half_period_us = (1e6/2) / fabs(current_pps);
					Timer1.initialize(half_period_us);

					// Set Direction TODO - state logic to reduce unnecessary IO
					if (current_pps > 0) {
						static_object->setDirection(CA4998::FORWARD);
					} else {
						static_object->setDirection(CA4998::REVERSE);
					}

					static_object->m_current_pps = current_pps;
				}
			} // end if time > accel_dt_ms
		} // end if pulse_level_high == true
	}
};

