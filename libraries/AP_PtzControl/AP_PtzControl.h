// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AP_PtzControl.h

#ifndef AP_PtzControl_H
#define AP_PtzControl_H

#include "SRV_Channel/SRV_Channel.h"


/// @class	PTZ
class AP_PtzControl {

public:
    /// Constructor
    ///
    AP_PtzControl()
    {
		_initialised = false;
    }
	void init();
	~AP_PtzControl();
	void set_servo_pwm(uint16_t pitch,uint16_t yaw,uint16_t photograph,uint16_t focus);        // Servo operated camera
	//	modified by zhangyong to disable shutter when in rtl mode 20180612
	void update(uint8_t channel_bit_mask);
	
private:
	bool            _initialised; 
	uint8_t 		_srv_aux_channel[NUM_SERVO_CHANNELS];

	struct _PTZ_Control{
		uint16_t _pitch = 1500;
		uint16_t _yaw = 1500;
		uint16_t _photograph = 1100;
		uint16_t _focus = 1500;
	}PTZ_Control;
};

#endif /* AP_CAMERA_H */
