//
// Created by barry on 10/26/20.
//
#pragma once

#include <Arduino.h>

//#define MY_DEBUG
#ifdef MY_DEBUG
#define DEBUG_INIT  Serial.begin(9600);
#define DEBUG_PRINT(X)   Serial.print(X);
#define DEBUG_PRINTLN(X) Serial.println(X);
#define DEBUG_PRINTLN2(X, Y) Serial.print(X); Serial.println(Y);
#else
#define DEBUG_INIT //
#define DEBUG_PRINT(X)  // Do Nothing
#define DEBUG_PRINTLN(X) // Do Nothing
#define DEBUG_PRINTLN2(X, Y) // Do Nothing
#endif

/// Debounce logical inputs. instantiate static.  Examples:
//   static debounceBool analogButton;
//   bool toggle1_result = analogButton.toggleOnFallingEdge(analogRead(A0) < 500);
//   static debounceBool digitalButton;
//   bool toggle2_result = digitalButton.toggleOnFallingEdge(digitalRead(5) == HIGH);

class debounceBool{
public: // members

private: // members
	bool current_output;
	bool push_active;
	const unsigned long debounce_ms = 50;
	unsigned long push_ms;
	unsigned long release_ms;

public: // methods
	debounceBool():
			current_output(false),
			push_ms(0),
			release_ms(0),
			push_active(false){};

	// output changes state on input transition to false
	// (i.e. button release)
	bool toggleOnFallingEdge(bool input){
		if (input)
		{
			push_ms = millis();
			push_active = true;
		} else
		{
			if(push_active)
			{
				release_ms=millis();
				if(release_ms-push_ms > debounce_ms)
				{
					push_active=false;
					current_output=!current_output;
				}
			}
		}
		return current_output;
	};


};

/// My delay function that combines delay and delayMicros
static void myDelayUs(uint32_t request_delay_us) {
	uint32_t delay_us = request_delay_us % 1000;
	uint32_t delay_ms = request_delay_us / 1000;
	if (delay_us) delayMicroseconds(delay_us);
	if (delay_ms) delay(delay_ms);
}

