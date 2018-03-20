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
	AP_GROUPINFO("REG_NO20", 23 , AP_NewBroadcast, _reg_no[20], 0),
	AP_GROUPINFO("REG_NO21", 24 , AP_NewBroadcast, _reg_no[21], 0),
	AP_GROUPINFO("REG_NO22", 25 , AP_NewBroadcast, _reg_no[22], 0),
	AP_GROUPINFO("REG_NO23", 26 , AP_NewBroadcast, _reg_no[23], 0),
	AP_GROUPINFO("REG_NO24", 27 , AP_NewBroadcast, _reg_no[24], 0),
	AP_GROUPINFO("REG_NO25", 28 , AP_NewBroadcast, _reg_no[25], 0),
	AP_GROUPINFO("REG_NO26", 29 , AP_NewBroadcast, _reg_no[26], 0),
	AP_GROUPINFO("REG_NO27", 30 , AP_NewBroadcast, _reg_no[27], 0),
	AP_GROUPINFO("REG_NO28", 31 , AP_NewBroadcast, _reg_no[28], 0),
	AP_GROUPINFO("REG_NO29", 32 , AP_NewBroadcast, _reg_no[29], 0),
	AP_GROUPINFO("REG_NO30", 33 , AP_NewBroadcast, _reg_no[30], 0),
	AP_GROUPINFO("REG_NO31", 34 , AP_NewBroadcast, _reg_no[31], 0),
	AP_GROUPINFO("REG_NO_COMP", 35 , AP_NewBroadcast, _reg_no_complete, 0),
				
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
	_parent_can_mgr = (PX4CANManager *)hal.can_mgr[0];
	timer = AP_HAL::millis();
	send_last_time = AP_HAL::micros();

	send_flag = COMPLETE;
	send_index = 0;
	memset( & view, 0, sizeof(view));
	memset( & payload, 0, sizeof(payload));

	if(_parent_can_mgr != NULL)
	{
		_initialized = true;
		printf("New Broadcast init success\n");
	}
	else
	{
		_initialized = false;
		printf("New Broadcast init false\n");
	}
}

void AP_NewBroadcast :: sendPayload(char *pArray)
{
    sendPayload(pArray,PAYLOAD_ARRAY_LEN);
}

//Send paylod array and crc16
void AP_NewBroadcast :: sendPayload(char *pArray,uint16_t len)
{
    uint16_t frame_num = len / 8;
    uint8_t last_frame_num_b = len % 8;

    for(int16_t i = send_index; i<(frame_num+2); i++)
    {
    	 send_flag = PROCESSING;
		 send_index = i+1;
		
        if(!_parent_can_mgr->getIface(0)->tx_pending())
        {
            if(i<frame_num)
            {
                 if(!sendDataStream(EXTENDID,pArray,i,8))
                 {
                    send_index = i;
                    i--;
                 }
            }
            else if(i == frame_num)
            {
                if(!sendDataStream(EXTENDID,pArray,i,last_frame_num_b))
                {
                    send_index = i;
                    i--;
                }
            }
            else
            {
                if(sendCrc16(EXTENDID))
                {
                    send_flag = COMPLETE;
					  send_index = 0;
					  send_last_time = AP_HAL::micros();
                }
				  else
				  {
				      send_index = i;
				  	  i--;
				  }
            }
        }
        else
        {
            send_index = i;
            i--;
			 break;
        }
    }
}

bool AP_NewBroadcast :: sendDataStream(frameType type,char *pArray,uint16_t index,uint8_t length)
{

    bool result = false;
    uint16_t j = index*8;

    uavcan::CanFrame frame;

    if (type == STANDARDID) {
        frame.id = uavcan::CanFrame::MaskStdID & 0x12;
    }
    else if(type == EXTENDID)
    {
        frame.id = uavcan::CanFrame::MaskExtID & 0x12;
        frame.id |= uavcan::CanFrame::FlagEFF;
    }

    frame.dlc = length & 0x0F;
    for(uint16_t i = 0;i<length;i++)
    {
        frame.data[i] = uint8_t(0XFF & pArray[j]);
        j++;
    }

    result =  _parent_can_mgr->getIface(0)->send_rf(frame, uavcan::MonotonicTime::fromUSec(1), uavcan::CanIOFlagAbortOnError);

    return result;
}

bool AP_NewBroadcast :: sendCrc16(frameType type)
{
    bool result = false;
	uint16_t crc16;

    uavcan::CanFrame frame;

    if (type == STANDARDID) {
        frame.id = uavcan::CanFrame::MaskStdID & 0x12;
    }
    else if(type == EXTENDID)
    {
        frame.id = uavcan::CanFrame::MaskExtID & 0x12;
        frame.id |= uavcan::CanFrame::FlagEFF;
    }

	crc16 = crc16_ccitt(payload.paylod_array,PAYLOAD_ARRAY_LEN);

    frame.dlc = 2;

    frame.data[0] = crc16 & 0x00ff;
	frame.data[1] = (crc16 & 0xff00) >> 8;

    result =  _parent_can_mgr->getIface(0)->send_rf(frame, uavcan::MonotonicTime::fromUSec(1), uavcan::CanIOFlagAbortOnError);

    return result;

}

 void AP_NewBroadcast :: update_view_action()
{
    view.action = 0;
}

void AP_NewBroadcast :: update_view_reg_no()
{
    //uint8_t length = sprintf((char*)view.number_buffer, "%d", _reg_no.get());
    //view.number_buffer[length] = '\0';
    for(int i = 0; i < REG_NO_STRING_LEN; i++)
    {
    	view.reg_no[i] = _reg_no[i];
    }
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
                if(AP_HAL::millis64()-timer>=15000)
                {
                    _flight_seq += 1;
                    _flight_seq.set_and_save_ifchanged(_flight_seq.get());
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
                if(AP_HAL::millis64()-timer>=15000)
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

	for(int i = 0; i < 14; i++)
	{
		payload._payload_s.reserved[i] = 0;
	}
}

void AP_NewBroadcast ::update()
{	
   update_view_flight_seq();

   if(!_initialized)
		return;

   if(!_enable)
   {
       return;
   }

   if(!_reg_no_complete)
   		return;

   
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
      update_view();
      update_payload();
   }
	   
   sendPayload(payload.paylod_array);
   
}

const uint16_t AP_NewBroadcast ::crc16tab[] = {
    0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
    0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
    0x1231,0x0210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,
    0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
    0x2462,0x3443,0x0420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,
    0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
    0x3653,0x2672,0x1611,0x0630,0x76d7,0x66f6,0x5695,0x46b4,
    0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
    0x48c4,0x58e5,0x6886,0x78a7,0x0840,0x1861,0x2802,0x3823,
    0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
    0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0x0a50,0x3a33,0x2a12,
    0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
    0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0x0c60,0x1c41,
    0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
    0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0x0e70,
    0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
    0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,
    0x1080,0x00a1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
    0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,
    0x02b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
    0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,
    0x34e2,0x24c3,0x14a0,0x0481,0x7466,0x6447,0x5424,0x4405,
    0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,
    0x26d3,0x36f2,0x0691,0x16b0,0x6657,0x7676,0x4615,0x5634,
    0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,
    0x5844,0x4865,0x7806,0x6827,0x18c0,0x08e1,0x3882,0x28a3,
    0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,
    0x4a75,0x5a54,0x6a37,0x7a16,0x0af1,0x1ad0,0x2ab3,0x3a92,
    0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,
    0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0x0cc1,
    0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,
    0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0x0ed1,0x1ef0
};

uint16_t AP_NewBroadcast :: crc16_ccitt(const char *buf, int len)
{
    int16_t counter = 0;
    uint16_t crc = 0;
    for( counter = 0; counter < len; counter++)
        crc = (crc<<8) ^ crc16tab[((crc>>8) ^ *(char *)buf++)&0x00FF];
    return crc;
} 

#endif

