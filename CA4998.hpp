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
//   start(period_us) - Use Timer 1 to send pulses on m_step_pin
//   stop() - stop sending pulses on m_step_pin
//   start() and stop() assure that mode changes take place at micro step # 0.
//
// setStepMode(1, 2, 4, 8, 16) - Select micro step mode.  1 is Full Steps.
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
#include "TimerOne.h" // instantiates at Timer1 object.

class CA4998
{
public: // members
	typedef enum {
		CLOCKWISE     = HIGH,
		COUNTER_CLOCK = LOW
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
	volatile int m_micro_step_num;  // volatile because ISR changes it.
	int m_num_steps_current_mode;
	static CA4998* static_object;

private: // methods
	void myDelayUs(unsigned long request_delay_us) const {
		unsigned long delay_us = request_delay_us % 1000;
		unsigned long delay_ms = request_delay_us / 1000;
		if (delay_us) delayMicroseconds(delay_us);
		if (delay_ms) delay(delay_ms);
	}

public: // methods
	explicit CA4998( // can be called before setup(), so no system calls
			int pin7_step,    // only pin you must specify
			int pin8_dir = 0, // floats - must be wired if not assigned to a pin.
			int pin2_m1 = 0, int pin3_m2 = 0, int pin4_m3 = 0, // pulled LOW internally
			int pin5_reset  = 0,   // floats - can tie to sleep to pull high
			int pin6_sleep  = 0,   // pulled HIGH internally.
    	int pin1_enable = 0) : // pulled LOW internally
			m_step_pin(pin7_step),
			m_dir_pin(pin8_dir),
			m_m1_pin(pin2_m1), m_m2_pin(pin3_m2), m_m3_pin(pin4_m3),
			m_reset_pin(pin5_reset),
			m_sleep_pin(pin6_sleep),
			m_enable_pin(pin1_enable),
			m_micro_step_num(0),
			m_num_steps_current_mode(0)
	{
		CA4998::static_object = this; // a non c++ target for the timer 1 interrupt
	}

	void init(StepType initial_step_mode = FULL_STEP,
					 DirectionType start_dir = CLOCKWISE) {

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
		this->setStepMode(initial_step_mode); // Sets reset() too.
		this->setStepLevel(LOW);  // Ready for LOW to HIGH transition.
		this->setDirection(start_dir); // HIGH clockwise, LOW counter
		this->wake();             // Not sleeping in low power mode.
		this->enable();           // all set, so enable

		// Prepare for timer operation
		CA4998::static_object = this;
		Timer1.attachInterrupt(CA4998::staticTimerFunc);
	};

	void disable() const { digitalWrite(m_enable_pin, HIGH); };

	void enable() const { digitalWrite(m_enable_pin, LOW); };

	void setStepMode(StepType step_mode)  // must be set 20ns prior to step()
	{
		switch(step_mode)
		{
			case HALF_STEP:
				if(m_m1_pin)digitalWrite(m_m1_pin, HIGH);
				if(m_m2_pin)digitalWrite(m_m2_pin, LOW);
				if(m_m3_pin)digitalWrite(m_m3_pin, LOW);
				m_num_steps_current_mode = 2*4; // *4 to complete a full cycle back to HOME state
				break;

			case QUARTER_STEP:
				if(m_m1_pin)digitalWrite(m_m1_pin, LOW);
				if(m_m2_pin)digitalWrite(m_m2_pin, HIGH);
				if(m_m3_pin)digitalWrite(m_m3_pin, LOW);
				m_num_steps_current_mode = 4*4;
				break;

			case EIGHTH_STEP:
				if(m_m1_pin)digitalWrite(m_m1_pin, HIGH);
				if(m_m2_pin)digitalWrite(m_m2_pin, HIGH);
				if(m_m3_pin)digitalWrite(m_m3_pin, LOW);
				m_num_steps_current_mode = 8*4;
				break;

			case SIXTEENTH_STEP:
				if(m_m1_pin)digitalWrite(m_m1_pin, HIGH);
				if(m_m2_pin)digitalWrite(m_m2_pin, HIGH);
				if(m_m3_pin)digitalWrite(m_m3_pin, HIGH);
				m_num_steps_current_mode = 16*4;
				break;

			default:  // Full Step
				if(m_m1_pin)digitalWrite(m_m1_pin, LOW);
				if(m_m2_pin)digitalWrite(m_m2_pin, LOW);
				if(m_m3_pin)digitalWrite(m_m3_pin, LOW);
				m_num_steps_current_mode = 1*4;
		}

		// Issue a reset
		this->reset();
	};

	void setDirection(DirectionType dir_level) const {
		if(m_dir_pin)digitalWrite(m_dir_pin, dir_level);
	};

	void reset() const {
		if(m_reset_pin)digitalWrite(m_reset_pin, LOW);
		delayMicroseconds(5);
		if(m_reset_pin)digitalWrite(m_reset_pin, HIGH);
		delayMicroseconds(5);
	};

	void sleep() const { if(m_sleep_pin)digitalWrite(m_sleep_pin, LOW); };

	void wake() const { if(m_sleep_pin)digitalWrite(m_sleep_pin, HIGH);	};

	void setStepLevel(int step_level) const {
		digitalWrite(m_step_pin, step_level);
	};

	// Use this when not using the start() and stop() timer approach.
	// This does not assure mode changes take place at micro step # 0.
	void singleStep(unsigned long pulse_width_us = 1000) const {

		unsigned long half_width = pulse_width_us / 2;

		digitalWrite(m_step_pin, HIGH);
		myDelayUs(half_width);
		digitalWrite(m_step_pin, LOW);
		myDelayUs(half_width);
	}

	// Timer function toggles the pulse input
	static void staticTimerFunc();

	void start(unsigned long period_us)
	{
		// 2 ticks per period for rising and falling edge of 50% duty cycle pulse
		Timer1.initialize(period_us/2);
		Timer1.start();
	}

	void stop()
	{
		// Wait for controller to return to HOME state
		while(m_micro_step_num != 0){};
		Timer1.stop();
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
		digitalWrite(static_object->m_step_pin, pulse_level_high? HIGH : LOW);
		pulse_level_high = !pulse_level_high;

		// Count pulses to sync mode changes with the HOME state
		int last_edge = static_object->m_num_steps_current_mode * 2;
		if (++(static_object->m_micro_step_num) >= last_edge) {
			static_object->m_micro_step_num = 0;
		}
	}
};

