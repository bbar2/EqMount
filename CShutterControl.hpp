//
// Created by barry on 5/7/21.
//

#pragma once
#include <Arduino.h>
#include "bbLocalLib.hpp"

const float START_DELAY_SEC = 5.0f; // delay before starting first shot

typedef struct {
	int half_press_sec;
	int full_press_sec;
	int release_sec;
	int num_shots;
} ShotGroup;

const ShotGroup shot_list[] ={
		{2, 5,30,30}
};

class CShutterControl
{
private: // members
	int m_half_press_pin;
	int m_full_press_pin;

	const int num_shot_groups = sizeof(shot_list) / sizeof(ShotGroup);
	int current_shot_group; // Which ShotGroup is running
	int current_shot;       // Shot within current ShotGroup

	typedef enum {
		CAMERA_OFF_STATE = 0,
		START_DELAY_STATE,
		HALF_PRESS_STATE,
		FULL_PRESS_STATE,
		RELEASE_STATE
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
		current_shot_group = 0;
		current_shot = 0;
	}

	void initIO() { // called in or after setup to initialize the IO channels
		if(m_full_press_pin) pinMode(m_full_press_pin, OUTPUT);
		if(m_half_press_pin) pinMode(m_half_press_pin, OUTPUT);
		releaseFullPress();
		releaseHalfPress();
	}

	void stop() {
		releaseFullPress();
		releaseHalfPress();
		camera_state = CAMERA_OFF_STATE;
		DEBUG_PRINTLN("CAMERA_OFF_STATE");
	}

	void begin(){
		camera_state = START_DELAY_STATE;
		state_start_sec = nowSec();
		current_shot_group = 0;
		current_shot = 0;
		DEBUG_PRINTLN("START_DELAY_STATE");
	}

	void run(){
		float time_in_state_sec = nowSec() - state_start_sec;
		CameraStateType next_camera_state = CAMERA_OFF_STATE;

		switch (camera_state)
		{
			case CAMERA_OFF_STATE:
				next_camera_state = CAMERA_OFF_STATE; // only begin() changes this.
				break;

			case START_DELAY_STATE:
				if (time_in_state_sec < START_DELAY_SEC) {
					next_camera_state = START_DELAY_STATE;
				} else {
					next_camera_state = HALF_PRESS_STATE;
					state_start_sec = nowSec();
					current_shot_group = 0;
					current_shot = 0;
					halfPress();
					DEBUG_PRINTLN2("HALF_PRESS_STATE ", current_shot);
				}
				break;

			case HALF_PRESS_STATE:
				if (time_in_state_sec < shot_list[current_shot_group].half_press_sec){
					next_camera_state = HALF_PRESS_STATE;
				} else {
					next_camera_state = FULL_PRESS_STATE;
					state_start_sec = nowSec();
					fullPress();
					DEBUG_PRINTLN2("FULL_PRESS_STATE ", current_shot);
				}
				break;

			case FULL_PRESS_STATE:
				if (time_in_state_sec < shot_list[current_shot_group].full_press_sec) {
					next_camera_state = FULL_PRESS_STATE;
				} else {
					next_camera_state = RELEASE_STATE;
					state_start_sec = nowSec();
					releaseFullPress();
					releaseHalfPress();
					DEBUG_PRINTLN2("RELEASE_STATE ", current_shot);
				}
				break;

			case RELEASE_STATE:
				if (time_in_state_sec < shot_list[current_shot_group].release_sec) {
					next_camera_state = RELEASE_STATE;
				} else {  // shot complete
					current_shot++;
					if (current_shot < shot_list[current_shot_group].num_shots) { // next shot in shot_group
						next_camera_state = HALF_PRESS_STATE;
						state_start_sec = nowSec();
						halfPress();
						DEBUG_PRINTLN("HALF_PRESS_STATE - next shot in shot_group");
					} else {  // shot group complete
						if(++current_shot_group < num_shot_groups){ // start next shot_group
							DEBUG_PRINTLN2("num_shot_group     = ", num_shot_groups)
							DEBUG_PRINTLN2("current_shot_group = ", current_shot_group)
							next_camera_state = HALF_PRESS_STATE;
							state_start_sec = nowSec();
							current_shot = 0;
							halfPress();
							DEBUG_PRINTLN("HALF_PRESS_STATE - start next shot_group");
						} else { // last shot group complete
							next_camera_state = CAMERA_OFF_STATE;
							state_start_sec = 0.0f;
							DEBUG_PRINTLN("CAMERA_OFF_STATE - last group");
						}
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
