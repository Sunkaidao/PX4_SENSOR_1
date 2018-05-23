/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * AP_NewBroadcast.cpp
 *
 *      Author: Breeder Bai
 */

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>


#include "AP_NewBroadcast.h"
#include <GCS_MAVLink/GCS.h>
#include "./../../ArduCopter/Copter.h"
#include "./../../ArduCopter/config.h"


#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>


#if NEWBROADCAST == ENABLED

extern const AP_HAL::HAL& hal;

// table of user settable New Broadcast parameters
const AP_Param::GroupInfo AP_NewBroadcast::var_info[] = {

    // @Param: REG_NO
    // @DisplayName: NBC_REG_NO
    // @Description:  UAV registration number
    // @User: Advanced
    //AP_GROUPINFO("REG_NO", 0 , AP_NewBroadcast, _reg_no, 0),

    // @Param: FLI_SEQ
    // @DisplayName: NBC_FLI_SEQ
    // @Description:  Number of flights
    // @User: Advanced
    AP_GROUPINFO("FLI_SEQ", 1 , AP_NewBroadcast, _flight_seq, 0),

    // @Param: ENABLE
    // @DisplayName: NBC_ENABLE
    // @Description:  Enable new Broadcast
    // @User: Advanced
    AP_GROUPINFO("ENABLE", 2 , AP_NewBroadcast, _enable, 1),

	AP_GROUPINFO("REG_NO0", 3 , AP_NewBroadcast, _reg_no[0], 0),
	AP_GROUPINFO("REG_NO1", 4 , AP_NewBroadcast, _reg_no[1], 0),
	AP_GROUPINFO("REG_NO2", 5 , AP_NewBroadcast, _reg_no[2], 0),
	AP_GROUPINFO("REG_NO3", 6 , AP_NewBroadcast, _reg_no[3], 0),
	AP_GROUPINFO("REG_NO4", 7 , AP_NewBroadcast, _reg_no[4], 0),
	AP_GROUPINFO("REG_NO5", 8 , AP_NewBroadcast, _reg_no[5], 0),
	AP_GROUPINFO("REG_NO6", 9 , AP_NewBroadcast, _reg_no[6], 0),
	AP_GROUPINFO("REG_NO7", 10 , AP_NewBroadcast, _reg_no[7], 0),
	AP_GROUPINFO("REG_NO8", 11 , AP_NewBroadcast, _reg_no[8], 0),
	AP_GROUPINFO("REG_NO9", 12 , AP_NewBroadcast, _reg_no[9], 0),
	AP_GROUPINFO("REG_NO10", 13 , AP_NewBroadcast, _reg_no[10], 0),
	AP_GROUPINFO("REG_NO11", 14 , AP_NewBroadcast, _reg_no[11], 0),
	AP_GROUPINFO("REG_NO12", 15 , AP_NewBroadcast, _reg_no[12], 0),
	AP_GROUPINFO("REG_NO13", 16 , AP_NewBroadcast, _reg_no[13], 0),
	AP_GROUPINFO("REG_NO14", 17 , AP_NewBroadcast, _reg_no[14], 0),
	AP_GROUPINFO("REG_NO15", 18 , AP_NewBroadcast, _reg_no[15], 0),
	AP_GROUPINFO("REG_NO16", 19 , AP_NewBroadcast, _reg_no[16], 0),
	AP_GROUPINFO("REG_NO17", 20 , AP_NewBroadcast, _reg_no[17], 0),
	AP_GROUPINFO("REG_NO18", 21 , AP_NewBroadcast, _reg_no[18], 0),
	AP_GROUPINFO("REG_NO19", 22 , AP_NewBroadcast, _reg_no[19], 0),

    AP_GROUPEND
};


AP_NewBroadcast::AP_NewBroadcast(AC_Sprayer& _sprayer):
    sprayer(_sprayer)
{
	AP_Param::setup_object_defaults(this, var_info);
	_initialized = false;
}

AP_NewBroadcast::~AP_NewBroadcast()
{
}

void AP_NewBroadcast::init()
{
	if(_enable == NewBroadcast_TYPE_NONE)
		return;
	
	send_flag = COMPLETE;
	flight_seq_pre = _flight_seq.get();
	flight_area_m2_curr = 0;
	flight_area_m2_pre = 0;
	memset( & view, 0, sizeof(view));
	memset( & payload, 0, sizeof(payload));
	update_view_flight_control();
	update_view_reg_no();
	
	if(detect_backends())
	{
		if(drive->init())
		{
			_initialized = true;
			printf("New Broadcast init success\n");
		}
		else
		{
			delete drive;
			drive = nullptr;
			_initialized = false;
			printf("New Broadcast init false\n");
		}
	}
	else
	{
		_initialized = false;
	}
}

uint8_t AP_NewBroadcast::detect_backends()
{
	uint8_t res = 0;
	AP_NewBroadcast_Backend *new_backend = nullptr;
	
	switch(_enable)
	{
		case NewBroadcast_TYPE_NONE:
			break;
		case NewBroadcast_TYPE_GK_CAN:
			new_backend = new AP_NewBroadcast_CAN(payload);
			break;
		case NewBroadcast_TYPE_GK_UAVCAN:
			new_backend = new AP_NewBroadcast_UAVCAN(payload);
			break;
		default:
			break;
	}

	if (new_backend != nullptr) 
	{
		drive = new_backend;
		res = 1;
	}

	return res;
}

 /*
   handle an msg_newbroadcast_str
  */
MAV_RESULT AP_NewBroadcast::handle_msg_newbroadcast_str(const mavlink_newbroadcast_str_t &packet,mavlink_channel_t chan)
{
	 MAV_RESULT result = MAV_RESULT_FAILED;
 
	 switch (packet.type) {
		 case REG_NO: {

			 update_reg_no(packet,chan);
			 result = MAV_RESULT_ACCEPTED; 
			 break;
		 }
		 case REG_NO_REQUEST: {

			 send_reg_no(chan);
			 result = MAV_RESULT_ACCEPTED; 
			 break;
		 }
			 
		 case TASK_ID: {
		 	
		 	 update_view_task_id(packet,chan);
			 result = MAV_RESULT_ACCEPTED; 
			 break;
		 }
		 case APP_VER_NO: {

			 update_view_app_ver_no(packet,chan);
			 result = MAV_RESULT_ACCEPTED;
			 break;
		 }
		 case TP_REG_NO: {

			 update_view_tp_reg_no(packet,chan);
			 result = MAV_RESULT_ACCEPTED;
			 break;
		 }
		 default:
		 	break;
	 }

	 return result;
}

void AP_NewBroadcast ::send_reg_no(mavlink_channel_t chan)
{
	int8_t reg_no[REG_NO_STRING_LEN];
		
	for(int i = 0; i < REG_NO_STRING_LEN; i++)
    {
    	reg_no[i] = _reg_no[i];  //May be wrong
    }
	
	mavlink_msg_newbroadcast_str_send(
		chan,
		REG_NO_REQUEST,
		0,
		reg_no);
}

void AP_NewBroadcast ::send_flight_status(mavlink_channel_t chan)
{

	mavlink_msg_newbroadcast_flight_sta_send(
		chan,
		view.flight_area,
		view.flight_length,
		view.flight_seq,
		view.state);

}

void AP_NewBroadcast :: update_view_action()
{
    view.action = 0;
}

void AP_NewBroadcast :: update_reg_no(const mavlink_newbroadcast_str_t &packet,mavlink_channel_t chan)
{
   	for(int i = 0; i < REG_NO_STRING_LEN; i++)
   	{
		_reg_no[i].set_and_save_ifchanged(packet.string[i]);
   	}

	mavlink_msg_newbroadcast_str_send_struct(chan,&packet);
}

void AP_NewBroadcast :: update_view_reg_no()
{
   	for(int i = 0; i < REG_NO_STRING_LEN; i++)
   	{
    	view.reg_no[i] = _reg_no[i];
	}
}

void AP_NewBroadcast ::update_view_task_id(const mavlink_newbroadcast_str_t &packet,mavlink_channel_t chan)
{
	for(int i = 0; i < TASK_ID_STRING_LEN; i++)
   	{
    	view.task_id[i] = packet.string[i];
   	}

	mavlink_msg_newbroadcast_str_send_struct(chan,&packet);
}

void AP_NewBroadcast ::update_view_app_ver_no(const mavlink_newbroadcast_str_t &packet,mavlink_channel_t chan)
{
	for(int i = 0; i < APP_VER_NO_STRING_LEN; i++)
   	{
    	view.app_ver_no[i] = packet.string[i];
   	}

	mavlink_msg_newbroadcast_str_send_struct(chan,&packet);
}

void AP_NewBroadcast ::update_view_tp_reg_no(const mavlink_newbroadcast_str_t &packet,mavlink_channel_t chan)
{
   	view.tp_reg_no = packet.tp_reg_no;
	mavlink_msg_newbroadcast_str_send_struct(chan,&packet);
}

void AP_NewBroadcast :: update_view_flight_seq()
{
    static int8_t view_step = 0;

    switch(view_step)
    {
        case 0:
            if(copter.motors->armed())
            {
                timer = AP_HAL::millis64();
                view_step = 1;
            }
            else
            {
                view_step = 0;
            }
            break;
        case 1:
            if(copter.motors->armed())
            {
                if(AP_HAL::millis64()-timer>=5000)
                {
                    _flight_seq += 1;
                    _flight_seq.set_and_save(_flight_seq);
                    view_step = 2;
                }
            }
            else
            {
                view_step = 0;
            }
            break;
        case 2:
             if(!copter.motors->armed())
             {
                view_step = 0;
             }
             break;
    }

    view.flight_seq = _flight_seq.get();
}

void AP_NewBroadcast :: update_view_now_time()
{
    view.now_time = hal.util->get_system_clock_ms()/1000;
}

void AP_NewBroadcast :: update_view_state()
{
    static int8_t state_step = 0;

    switch(state_step)
    {
        case 0:
            if(copter.motors->armed())
            {
                timer = AP_HAL::millis64();
                view.state = 1;
                state_step = 1;
            }
            else
            {
                view.state = 3;
                state_step = 0;
            }
            break;
        case 1:
            if(copter.motors->armed())
            {
                if(AP_HAL::millis64()-timer>=5000)
                {
                    view.state = 2;
                    state_step = 2;
                }
            }
            else
            {
                view.state = 3;
                state_step = 0;
            }
            break;
        case 2:
             if(!copter.motors->armed())
             {
                view.state = 3;
                state_step = 0;
             }
             break;
    }

}

void AP_NewBroadcast :: update_view_flight_time()
{
    view.flight_time = copter.local_flight_time_sec*1000;
}

void AP_NewBroadcast :: update_view_longitude()
{
    view.longitude = copter.inertial_nav.get_longitude();
}

void AP_NewBroadcast :: update_view_latitude()
{
    view.latitude = copter.inertial_nav.get_latitude();
}

void AP_NewBroadcast :: update_view_height()
{
    view.height = copter.inertial_nav.get_position().z*10;
}

void AP_NewBroadcast :: update_view_altitude()
{
    view.altitude = copter.gps.location().alt*100;
}

void AP_NewBroadcast :: update_view_path_angle()
{
    view.path_angle = copter.ahrs.yaw_sensor/10;
}

void AP_NewBroadcast :: update_view_pitch_angle()
{
    view.pitch_angle = copter.ahrs.pitch_sensor/10;
}

void AP_NewBroadcast :: update_view_roll_angle()
{
    view.roll_angle = copter.ahrs.roll_sensor/10;
}

void AP_NewBroadcast :: update_view_horizontal_velocity()
{
    view.horizontal_velocity = copter.gps.ground_speed()*10;
}

void AP_NewBroadcast :: update_view_is_nozzle_work()
{
    view.is_nozzle_work = sprayer.get_spraying();
}

void AP_NewBroadcast :: update_view_nozzle_diameter()
{
    view.nozzle_diameter = 0;
}
void AP_NewBroadcast :: update_view_nozzle_angle()
{
    view.nozzle_angle = 0;
}

void AP_NewBroadcast :: update_view_nozzle_pressure()
{
    view.nozzle_pressure = 0;
}

void AP_NewBroadcast :: update_view_spray_range()
{
    view.spray_range = (sprayer.get_unspray_dist()-50)/10;
}

void AP_NewBroadcast :: update_view_flight_control()
{	
	for(int i = 0; i < FLIGHT_CONTROL_STRING_LEN; i++)
	{
#if FXTX_AUTH == ENABLED 
		view.flight_control[i] = copter.auth_msg[i+18]; 
#else 
		view.flight_control[i] = 0; 
#endif 
	}
}
void AP_NewBroadcast :: update_view_remain_dose()
{
	view.remain_dose = 0;
}
void AP_NewBroadcast :: update_view_used_dose()
{
	view.used_dose = 0;
}
void AP_NewBroadcast :: update_view_cur_flow()
{
	view.cur_flow = 0;
}
void AP_NewBroadcast :: update_view_flight_area()
{
	static uint8_t step = 0;
	//float distance; //uint: meter
	
	switch(step)
	{
		case 0:
			if(sprayer.get_spraying()&&copter.motors->armed())
			{
				spraying_start.lat = copter.inertial_nav.get_latitude();
				spraying_start.lng = copter.inertial_nav.get_longitude();
				
				step = 1;
				flight_area_m2_pre += flight_area_m2_curr;
			}
			break;
		case 1:
			if(sprayer.get_spraying()&&copter.motors->armed())
			{
				spraying_end.lat = copter.inertial_nav.get_latitude();
				spraying_end.lng = copter.inertial_nav.get_longitude();
				
				flight_area_m2_curr = get_distance(spraying_start,spraying_end)*((sprayer.get_unspray_dist()-50)/100);
				
				view.flight_area = (flight_area_m2_curr + flight_area_m2_pre)*0.015f;
			}
			else if(!copter.motors->armed())
			{
				flight_area_m2_pre = 0;
				flight_area_m2_curr = 0;
			}
			else
			{
				step = 0;
			}
			break;
	}
}
void AP_NewBroadcast :: update_view_flight_length()
{
	static uint8_t step = 0;
	
	switch(step)
	{
		case 0:
			if(flight_seq_pre != _flight_seq.get())
			{
				flight_seq_originate.lat = copter.inertial_nav.get_latitude();
				flight_seq_originate.lng = copter.inertial_nav.get_longitude();
				
				step = 1;
				flight_seq_pre = _flight_seq.get();
			}
			break;
		case 1:
			if(flight_seq_pre == _flight_seq.get())
			{
				flight_seq_current.lat = copter.inertial_nav.get_latitude();
				flight_seq_current.lng = copter.inertial_nav.get_longitude();
				
				view.flight_length = get_distance(flight_seq_originate,flight_seq_current);

				if(!copter.motors->armed())
				{
					step = 0;
					flight_seq_pre = _flight_seq.get();
				}
			}
			else
			{
				step = 0;
				flight_seq_pre = _flight_seq;
			}
			break;
	}
}

void AP_NewBroadcast :: update_view()
{
    update_view_action();
    update_view_reg_no();
    update_view_now_time();
    update_view_state();
    update_view_flight_time();
    update_view_longitude();
    update_view_latitude();
    update_view_height();
    update_view_altitude();
    update_view_path_angle();
    update_view_pitch_angle();
    update_view_roll_angle();
    update_view_horizontal_velocity();
    update_view_is_nozzle_work();
    update_view_nozzle_diameter();
    update_view_nozzle_angle();
    update_view_nozzle_pressure();
    update_view_spray_range();
	update_view_remain_dose();
	update_view_used_dose();
	update_view_cur_flow();
	update_view_flight_seq();
   	update_view_flight_length();
   	update_view_flight_area();
}

void AP_NewBroadcast :: update_payload()
{
	payload._payload_s.frame_header0 = 0xAA;
	payload._payload_s.frame_header1 = 0x55;
	payload._payload_s.action = view.action;

	for(int i = 0; i < REG_NO_STRING_LEN; i++)
	{		
		payload._payload_s.reg_no[i] = view.reg_no[i];
	}
	
	payload._payload_s.flight_seq = view.flight_seq;
	
	payload._payload_s.now_time = view.now_time;
	payload._payload_s.state = view.state;
	payload._payload_s.flight_time = view.flight_time;
	payload._payload_s.longitude = view.longitude;
	payload._payload_s.latitude = view.latitude;
	payload._payload_s.height = view.height;
	payload._payload_s.altitude = view.altitude;
	payload._payload_s.path_angle = view.path_angle;
	payload._payload_s.pitch_angle = view.pitch_angle;
	payload._payload_s.roll_angle = view.roll_angle;
	payload._payload_s.horizontal_velocity = view.horizontal_velocity;
	payload._payload_s.is_nozzle_work = view.is_nozzle_work;
	payload._payload_s.nozzle_diameter = view.nozzle_diameter;
	payload._payload_s.nozzle_angle= view.nozzle_angle;
	payload._payload_s.nozzle_pressure = view.nozzle_pressure;
	payload._payload_s.spray_range = view.spray_range;

	for(int i = 0; i < FLIGHT_CONTROL_STRING_LEN; i++)
	{		
		payload._payload_s.flight_control[i] = view.flight_control[i];
	}
	for(int i = 0; i < TASK_ID_STRING_LEN; i++)
	{		
		payload._payload_s.task_id[i] = view.task_id[i];
	}
	for(int i = 0; i < APP_VER_NO_STRING_LEN; i++)
	{		
		payload._payload_s.app_ver_no[i] = view.app_ver_no[i];
	}

	payload._payload_s.tp_reg_no = view.tp_reg_no;
	payload._payload_s.remain_dose = view.remain_dose;
	payload._payload_s.used_dose = view.used_dose;
	payload._payload_s.cur_flow = view.cur_flow;
	payload._payload_s.flight_area = view.flight_area;
	payload._payload_s.flight_length = view.flight_length;
	
	for(int i = 0; i < RESERVED_NUM; i++)
	{
		payload._payload_s.reserved[i] = 0;
	}
}

void AP_NewBroadcast ::update()
{	
   if(!_initialized)
		return;

   if(!_enable)
   {
       return;
   }

   if(copter.gps.status() < AP_GPS::GPS_OK_FIX_3D) 
   {
   		send_last_time = AP_HAL::micros();
		return;
   }


	//TOOL:timeout detect,if true,do something,needn't use return.
	//
	//if(AP_HAL::micros()-send_last_time > 3000000)
   		

	//1Hz send 
   if(AP_HAL::micros()-send_last_time < 1000000)
   		return;
   	
   if(send_flag == COMPLETE)
   {
      update_payload();
   }

   if(drive == nullptr)
   		return;
   
   drive->send(send_flag,send_last_time);
   
}

#endif

