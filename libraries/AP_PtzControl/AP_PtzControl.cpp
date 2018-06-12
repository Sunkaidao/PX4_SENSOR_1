// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_PtzControl/AP_PtzControl.h>
#include <stdio.h>

#define BY_DEBUGGING 0

#if BY_DEBUGGING
# include <stdio.h>
 # define Debug(fmt, args ...)  do {::printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
 # define Debug(fmt, args ...)
#endif



void AP_PtzControl::init()
{	
    
	memset(_srv_aux_channel, 0, NUM_SERVO_CHANNELS);

	/*
	//
	k_ptz_pitch             = 95,
	k_ptz_yaw               = 96,
	k_ptz_photograph        = 97,
	k_ptz_focus             = 98,
	//
	*/
	
	if(false == SRV_Channels::find_channel(SRV_Channel::k_ptz_pitch, _srv_aux_channel[0]))
	{
		_initialised = false;
		return;
	}
	else if(false == SRV_Channels::find_channel(SRV_Channel::k_ptz_yaw, _srv_aux_channel[1]))
	{
		_initialised = false;
		return;
	}
	else if(false == SRV_Channels::find_channel(SRV_Channel::k_ptz_photograph, _srv_aux_channel[2]))
	{
		_initialised = false;
		return;
	}
	else if(false == SRV_Channels::find_channel(SRV_Channel::k_ptz_focus, _srv_aux_channel[3]))
	{
		_initialised = false;
		return;
	}

	_initialised = true;

}

AP_PtzControl::~AP_PtzControl()
{}

/// set Servo output pwm
void                          
AP_PtzControl::set_servo_pwm(uint16_t pitch,uint16_t yaw,uint16_t photograph,uint16_t focus)
{
	if(!_initialised){
       return;
	}
  
	PTZ_Control._pitch = pitch;
	PTZ_Control._yaw = yaw;
	PTZ_Control._photograph = photograph;
	PTZ_Control._focus = focus;

}

void AP_PtzControl::update(uint8_t channel_bit_mask)
{
	if(!_initialised){
       return;
	}
	
	SRV_Channels::set_output_pwm(SRV_Channel::k_ptz_pitch,PTZ_Control._pitch);
	SRV_Channels::set_output_pwm(SRV_Channel::k_ptz_yaw,PTZ_Control._yaw);
	SRV_Channels::set_output_pwm(SRV_Channel::k_ptz_photograph,PTZ_Control._photograph);
	SRV_Channels::set_output_pwm(SRV_Channel::k_ptz_focus,PTZ_Control._focus);
}

