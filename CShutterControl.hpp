//
// Created by barry on 5/7/21.
//

#pragma once
#include <Arduino.h>
#include "bbLocalLib.hpp"

// todo - bug in startup delay
// todo - make a struct or tuple for each press.
// todo - add a num shots of a current tuple
// todo - state is then a list of :
//    (full press, gap), num_reps.
//    (full press, gap), num_reps.
// todo - should half press be an element, or constant shared by all.


const float START_DELAY_SEC = 5.0f; // delay before starting first shot

typedef struct {
	int half_press_sec;
	int full_press_sec;
	int pause_sec;
	int num_shots;
} ShotGroup;

const ShotGroup shot_list[3] ={
		{2, 5,10,5},
		{2, 1,1,5}
};

class CShutterControl
{
private: // members
	int m_half_press_pin;
	int m_full_press_pin;

	const int num_shot_groups = sizeof(shot_list) / sizeof(ShotGroup);
	int current_shot_group; // which ShotGroup is running
	int current_shot;       // shot withing group

	typedef enum {
		CAMERA_OFF_STATE = 0,
		START_DELAY_STATE,
		HALF_PRESS_STATE,
		FULL_PRESS_STATE,
		POST_PRESS_STATE
	} CameraStateType;
	CameraStateType camera_state;
	float state_start_sec;

public: // members

private: // methods
	inline void releaseHalfPress() const { if(m_half_press_pin) digitalWrite(m_half_press_pin, LOW); }
	inline void releaseFullPress() const { if(m_full_press_pin) digitalWrite(m_full_press_pin, LOW); }
	inline void halfPress() const { if(m_half_press_pin) digitalWrite(m_half_press_pin, HIGH); }
	inline void fullPress() const { if(m_full_press_pin) digitalWrite(m_full_press_pin, HIGH); }
	static inline float nowSec() { return float(millis()) / 1000.f;}

public: // methods
	explicit CShutterControl(  // may be called before setup(), so no system calls
		int half_press_pin,
		int full_press_pin):
		m_half_press_pin(half_press_pin),
		m_full_press_pin(full_press_pin)
	{
		camera_state = CAMERA_OFF_STATE;
		state_start_sec = 0;
		current_shot = 0;
	}

	void init() {
		if(m_full_press_pin) pinMode(m_full_press_pin, OUTPUT);
		if(m_half_press_pin) pinMode(m_half_press_pin, OUTPUT);
		releaseFullPress();
		releaseHalfPress();
		camera_state = CAMERA_OFF_STATE;
		state_start_sec = 0;
		current_shot = 0;
	}

	void stop() {
		releaseFullPress();
		releaseHalfPress();
		camera_state = CAMERA_OFF_STATE;
		DEBUG_PRINTLN("CAMERA_OFF_STATE");
	}

	void begin(){
		camera_state = START_DELAY_STATE;
		DEBUG_PRINTLN2("START_DELAY_STATE", state_start_sec);
	}

	void run(){
		float time_in_state_sec = nowSec() - state_start_sec;
		CameraStateType next_camera_state = CAMERA_OFF_STATE;
		switch (camera_state)
		{
			case CAMERA_OFF_STATE:
				next_camera_state = CAMERA_OFF_STATE;
				break;
			case START_DELAY_STATE:
				if (time_in_state_sec < START_DELAY_SEC)
				{
					next_camera_state = START_DELAY_STATE;
				} else
				{
					next_camera_state = HALF_PRESS_STATE;
					state_start_sec = nowSec();
					current_shot = 0;
					halfPress();
					DEBUG_PRINTLN("HALF_PRESS_STATE");
				}
				break;
			case HALF_PRESS_STATE:
				if (time_in_state_sec < HALF_PRESS_SEC)
				{
					next_camera_state = HALF_PRESS_STATE;
				} else
				{
					next_camera_state = FULL_PRESS_STATE;
					state_start_sec = nowSec();
					fullPress();
					DEBUG_PRINTLN2("FULL_PRESS_STATE ", current_shot);
				}
				break;
			case FULL_PRESS_STATE:
				if (time_in_state_sec < SHOT_LEN_SEC[current_shot])
				{
					next_camera_state = FULL_PRESS_STATE;
				} else
				{
					next_camera_state = POST_PRESS_STATE;
					state_start_sec = nowSec();
					releaseFullPress();
					releaseHalfPress();
					DEBUG_PRINTLN("POST_PRESS_STATE");
				}
				break;
			case POST_PRESS_STATE:
				if (time_in_state_sec < SHOT_GAP_SEC[current_shot])
				{
					next_camera_state = POST_PRESS_STATE;
				} else
				{
					current_shot++;
					if (current_shot < NUM_SHOTS)
					{
						next_camera_state = HALF_PRESS_STATE;
						state_start_sec = nowSec();
						halfPress();
						DEBUG_PRINTLN("HALF_PRESS_STATE");
					} else
					{
						next_camera_state = CAMERA_OFF_STATE;
						state_start_sec = 0.0f;
						DEBUG_PRINTLN("CAMERA_OFF_STATE");
					}
				}
				break;
			default:
				next_camera_state = CAMERA_OFF_STATE;
				releaseFullPress();
				releaseHalfPress();
		}
		camera_state = next_camera_state;
	}

};
