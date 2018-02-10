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
    AP_GROUPINFO("REG_NO", 0 , AP_NewBroadcast, _reg_no, 0),

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

void AP_NewBroadcast :: sendJson(char *pJString)
{
    uint16_t length = strlen(pJString);

    sendJson(pJString,length);
}

//Send JSON strings and delimiters
void AP_NewBroadcast :: sendJson(char *pJString,uint16_t len)
{
    uint16_t frame_num = len / 8;
    uint8_t last_frame_num_b = len % 8;
    uint32_t tstart = AP_HAL::micros();

    for(uint16_t i = 0; i<(frame_num+2); i++)
    {
        if(!_parent_can_mgr->getIface(0)->tx_pending())
        {
            if(i<frame_num)
            {
                 if(!sendString(EXTENDID,pJString,i,8))
                 {
                    i--;
                 }
            }
            else if(i == frame_num)
            {
                if(!sendString(EXTENDID,pJString,i,last_frame_num_b))
                {
                    i--;
                }
            }
            else
            {
                if(!sendSeparatorSymbol(EXTENDID))
                {
                    i--;
                }
            }
        }
        else
        {
            i--;
        }

        if(AP_HAL::micros() - tstart > 10000)
        {
            break;
        }
    }
}

bool AP_NewBroadcast :: sendString(frameType type,char *pJString,uint16_t index,uint8_t length)
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
        frame.data[i] = uint8_t(0XFF & pJString[j]);
        j++;
    }

    result =  _parent_can_mgr->getIface(0)->send_rf(frame, uavcan::MonotonicTime::fromUSec(1), uavcan::CanIOFlagAbortOnError);

    return result;
}

bool AP_NewBroadcast :: sendSeparatorSymbol(frameType type)
{
    bool result = false;

    uavcan::CanFrame frame;

    if (type == STANDARDID) {
        frame.id = uavcan::CanFrame::MaskStdID & 0x12;
    }
    else if(type == EXTENDID)
    {
        frame.id = uavcan::CanFrame::MaskExtID & 0x12;
        frame.id |= uavcan::CanFrame::FlagEFF;
    }

    frame.dlc = 1;

    frame.data[0] = '\n';

    result =  _parent_can_mgr->getIface(0)->send_rf(frame, uavcan::MonotonicTime::fromUSec(1), uavcan::CanIOFlagAbortOnError);

    return result;

}

char * AP_NewBroadcast :: makeViewJson()
{
     cJSON * pJsonRoot = NULL;

     pJsonRoot = cJSON_CreateObject();
     if(NULL == pJsonRoot)
     {
         //error happend here
         printf("Create cJSON false");
         return NULL;
     }

     cJSON_AddStringToObject(pJsonRoot, "reg_no", (const char *)view.number_buffer);
     cJSON_AddNumberToObject(pJsonRoot, "action", view.action);
     cJSON_AddNumberToObject(pJsonRoot, "now_time", view.now_time);
     cJSON_AddNumberToObject(pJsonRoot, "spray_range", view.spray_range);
     cJSON_AddNumberToObject(pJsonRoot, "altitude",view.altitude);
     cJSON_AddNumberToObject(pJsonRoot, "latitude",view.latitude);
     cJSON_AddNumberToObject(pJsonRoot, "pitch_angle",view.pitch_angle);
     cJSON_AddNumberToObject(pJsonRoot, "flight_time",view.flight_time);
     cJSON_AddNumberToObject(pJsonRoot, "nozzle_diameter",view.nozzle_diameter);
     cJSON_AddNumberToObject(pJsonRoot, "horizontal_velocity",view.horizontal_velocity);
     cJSON_AddNumberToObject(pJsonRoot, "flight_seq",view.flight_seq);
     cJSON_AddNumberToObject(pJsonRoot, "roll_angle",view.roll_angle);
     cJSON_AddNumberToObject(pJsonRoot, "is_nozzle_work",view.is_nozzle_work);
     cJSON_AddNumberToObject(pJsonRoot, "nozzle_angle",view.nozzle_angle);
     cJSON_AddNumberToObject(pJsonRoot, "state",view.state);
     cJSON_AddNumberToObject(pJsonRoot, "nozzle_pressure",view.nozzle_pressure);
     cJSON_AddNumberToObject(pJsonRoot, "longitude",view.longitude);
     cJSON_AddNumberToObject(pJsonRoot, "height",view.height);
     cJSON_AddNumberToObject(pJsonRoot, "path_angle",view.path_angle);

     char * p = cJSON_PrintUnformatted(pJsonRoot);
     if(NULL == p)
     {
         //convert json list to string faild, exit
         //because sub json pSubJson han been add to pJsonRoot, so just delete pJsonRoot, if you also delete pSubJson, it will coredump, and error is : double free
         cJSON_Delete(pJsonRoot);
         return NULL;
     }

     cJSON_Delete(pJsonRoot);

     return p;
}

 void AP_NewBroadcast :: update_view_action()
{
    view.action = 0;
}

void AP_NewBroadcast :: update_view_reg_no()
{
    uint8_t length = sprintf((char*)view.number_buffer, "%d", _reg_no.get());
    view.number_buffer[length] = '\0';
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
                if(AP_HAL::millis64()-timer>=3000)
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
    view.is_nozzle_work = 0;
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
    update_view_flight_seq();
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

void AP_NewBroadcast ::update()
{
	if(!_initialized)
		return;

    if(!_enable)
    {
        return;
    }

	if(copter.gps.status() >= AP_GPS::GPS_OK_FIX_3D)
    {
        update_view();

        char * p = makeViewJson();

        if(NULL == p)
        {
            return;
        }

        sendJson(p);
        free(p);
    }
}

#endif

