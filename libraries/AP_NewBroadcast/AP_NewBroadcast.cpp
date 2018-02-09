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

//	printf("Send status %d\n",result);
	//printf("Send Date: OK!\n");
	//printf("tx pending %d\n",_parent_can_mgr->getIface(0)->tx_pending());

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

	//printf("Send status %d\n",result);
	//printf("Send Date: OK!\n");
	//printf("tx pending %d\n",_parent_can_mgr->getIface(0)->tx_pending());

    return result;

}

void AP_NewBroadcast :: send_msg(uint8_t id_mode,uint8_t frame_mode,uint8_t length)
{

	uavcan::CanFrame frame;

    if (id_mode == 0) {
        frame.id = uavcan::CanFrame::MaskStdID & 0x12;
    } else {
        frame.id = uavcan::CanFrame::MaskExtID & 0x12;
        frame.id |= uavcan::CanFrame::FlagEFF;
    }

    if (frame_mode != 0) {
        frame.id |= uavcan::CanFrame::FlagRTR;
    }

    frame.dlc = length & 15;

    frame.data[0] = uint8_t(0xFF & 0);
    frame.data[1] = uint8_t(0xFF & 1);
    frame.data[2] = uint8_t(0xFF & 2);
    frame.data[3] = uint8_t(0xFF & 3);
    frame.data[4] = uint8_t(0xFF & 4);
    frame.data[5] = uint8_t(0xFF & 5);
    frame.data[6] = uint8_t(0xFF & 6);
    frame.data[7] = uint8_t(0xFF & 7);

	printf("Send status %d\n",_parent_can_mgr->getIface(0)->send_rf(frame, uavcan::MonotonicTime::fromUSec(1), uavcan::CanIOFlagAbortOnError));
	printf("Send Date: OK!\n");
	printf("tx pending %d\n",_parent_can_mgr->getIface(0)->tx_pending());
}

/*
void AP_NewBroadcast ::handle_msg()
{
	int32_t num;
	num = _parent_can_mgr->getIface(0)->available();

	if(num <= 0)
		return;

	uavcan::CanFrame out_frame;
	uavcan::MonotonicTime out_ts_monotonic;
    uavcan::UtcTime out_ts_utc;
	uavcan::CanIOFlags out_flags;

	for(int i = 0;i<num;i++)
	{
		_parent_can_mgr->getIface(0)->receive_rf(out_frame,out_ts_monotonic,out_ts_utc,out_flags);
		printf("ExtID %2X\n",out_frame.id);
		for(int j = 0;j<out_frame.dlc;j++)
		{
			printf("%2X \n",out_frame.data[j]);

		}
		//printf("****************\n");
	}
	printf("Receive: OK!\n");
	timer = AP_HAL::millis();
}
*/

bool AP_NewBroadcast ::read()
{
	int32_t num;
	num = _parent_can_mgr->getIface(0)->available();

	if(num <= 0)
		return false;

	uavcan::CanFrame out_frame;
	uavcan::MonotonicTime out_ts_monotonic;
    uavcan::UtcTime out_ts_utc;
	uavcan::CanIOFlags out_flags;

	for(int i = 0;i<num;i++)
	{
		_parent_can_mgr->getIface(0)->receive_rf(out_frame,out_ts_monotonic,out_ts_utc,out_flags);
		for(int j = 0;j<out_frame.dlc;j++)
		{
			//printf("%c",out_frame.data[j]);
            handle_msg(decode(out_frame.data[j]));
		}
	}
//	printf("Receive: OK!\n");
//	timer = AP_HAL::millis();
	return true;
}

void AP_NewBroadcast:: handle_msg(int8_t result)
{
    switch(result)
    {
        case -1:
            break;
        case 0:
            break;
        case 1:
            parseJson(JString);
            break;
    }
}

void AP_NewBroadcast::parseJson(char * pMsg)
{
    if(NULL == pMsg)
    {
        return;
    }
    cJSON * pJson = cJSON_Parse(pMsg);
    if(NULL == pJson)
    {
        // parse faild, return
        return ;
    }

    // get string from json
    cJSON * pSub = cJSON_GetObjectItem(pJson, "action");
    if(NULL == pSub)
    {
        //get object named "hello" faild
    }
    //printf("\n\n");
    //printf("P:action : %d\n", pSub->valueint);

    // get number from json
    pSub = cJSON_GetObjectItem(pJson, "reg_no");
    if(NULL == pSub)
    {
         //get number from json faild
    }
    //printf("P:reg_no : %s\n", pSub->valuestring);

    cJSON_Delete(pJson);
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

//JSON format output test
char * AP_NewBroadcast:: makeJson()
{
     cJSON * pJsonRoot = NULL;

     pJsonRoot = cJSON_CreateObject();
     if(NULL == pJsonRoot)
     {
         //error happend here
         printf("Create cJSON false");
         return NULL;
     }

     cJSON_AddStringToObject(pJsonRoot, "reg_no", "1110023452");
     cJSON_AddNumberToObject(pJsonRoot, "action", 0);
     cJSON_AddNumberToObject(pJsonRoot, "now_time", 1516345485);
     cJSON_AddNumberToObject(pJsonRoot, "spray_range", 6);
     cJSON_AddNumberToObject(pJsonRoot, "altitude",10000);
     cJSON_AddNumberToObject(pJsonRoot, "uav_id",0);
     cJSON_AddNumberToObject(pJsonRoot, "latitude",2255325507);
     cJSON_AddNumberToObject(pJsonRoot, "pitch_angle",500);
     cJSON_AddNumberToObject(pJsonRoot, "index",0);
     cJSON_AddNumberToObject(pJsonRoot, "flight_time",0);
     cJSON_AddNumberToObject(pJsonRoot, "nozzle_diameter",10);
     cJSON_AddNumberToObject(pJsonRoot, "horizontal_velocity",300);
     cJSON_AddNumberToObject(pJsonRoot, "flight_seq",1);
     cJSON_AddNumberToObject(pJsonRoot, "roll_angle",600);
     cJSON_AddNumberToObject(pJsonRoot, "is_nozzle_work",0);
     cJSON_AddNumberToObject(pJsonRoot, "nozzle_angle",60);
     cJSON_AddNumberToObject(pJsonRoot, "state",1);
     cJSON_AddNumberToObject(pJsonRoot, "nozzle_pressure",1000);
     cJSON_AddNumberToObject(pJsonRoot, "longitude",1139460592);
     cJSON_AddNumberToObject(pJsonRoot, "height",20000);
     cJSON_AddNumberToObject(pJsonRoot, "path_angle",400);

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

/*
 *char * AP_NewBroadcast:: makeJson()
 *{
 *     cJSON * pJsonRoot = NULL;
 *
 *     pJsonRoot = cJSON_CreateObject();
 *     if(NULL == pJsonRoot)
 *     {
 *         //error happend here
 *         return NULL;
 *     }
 *     cJSON_AddStringToObject(pJsonRoot, "hello", "hello world");
 *     //cJSON_AddNumberToObject(pJsonRoot, "number", 10010);
 *     //cJSON_AddNumberToObject(pJsonRoot, "bool", 1);
 *     cJSON * pSubJson = NULL;
 *     pSubJson = cJSON_CreateObject();
 *     if(NULL == pSubJson)
 *     {
 *         // create object faild, exit
 *         cJSON_Delete(pJsonRoot);
 *         return NULL;
 *     }
 *     cJSON_AddStringToObject(pSubJson, "subjsonobj", "a sub json string");
 *     cJSON_AddItemToObject(pJsonRoot, "subobj", pSubJson);
 *
 *     //char * p = cJSON_Print(pJsonRoot);
 *   // else use :
 *     char * p = cJSON_PrintUnformatted(pJsonRoot);
 *     if(NULL == p)
 *     {
 *         //convert json list to string faild, exit
 *         //because sub json pSubJson han been add to pJsonRoot, so just delete pJsonRoot, if you also delete pSubJson, it will coredump, and error is : double free
 *         cJSON_Delete(pJsonRoot);
 *         return NULL;
 *     }
 *     //free(p);
 *
 *     cJSON_Delete(pJsonRoot);
 *
 *     return p;
 *}
 */



int8_t AP_NewBroadcast ::decode(char data)
{
    int8_t result = 0;

    switch(data)
    {
        case '\r':
        case '\n':
            if(payload_offset != 0)
            {
                JString[payload_offset] = '\0';
                payload_offset = 0;
                result = 1;
            }
            break;
        default:
            if(payload_offset >= CJSON_STRING_LEN_MAX)
            {
                payload_offset = 0;
                result = -1;
            }
            else
            {
                JString[payload_offset++] = data;
            }
            break;

    }

    return result;
}

/*
 *
 *void AP_NewBroadcast::decode(uint8_t data)
 *{
 *  msg_complete = false;
 *
 *  switch (step) {
 *  case 0:
 *    //init_data();
 *    if (data == 0xAA) {
 *      CRC = 0xFFFF;
 *      step = 1;
 *      valcheck = CRC16Value(data);
 *    } else {
 *      step = 0;
 *    }
 *    printf("%2X \n",data);
 *    break;
 *  case 1:
 *    if (data == 0x44) {
 *      step = 2;
 *      valcheck = CRC16Value(data);
 *    } else {
 *      step = 0;
 *    }
 *    printf("%2X \n",data);
 *    break;
 *  case 2:
 *    if (data == 0x18) {
 *      valcheck = CRC16Value(data);
 *      step = 3;
 *    } else {
 *      step = 0;
 *    }
 *    printf("%2X \n",data);
 *    break;
 *  case 3:
 *    msg_rx[msg_index].msgid = 0x00FF & data;
 *    valcheck = CRC16Value(data);
 *    step = 4;
 *    printf("%2X \n",data);
 *    break;
 *  case 4:
 *    msg_rx[msg_index].msgid = ((uint16_t)data << 8) + msg_rx[msg_index].msgid;
 *    valcheck = CRC16Value(data);
 *    step = 5;
 *    printf("%2X \n",data);
 *    break;
 *  case 5:
 *    msg_rx[msg_index].version = data;
 *    valcheck = CRC16Value(data);
 *    step = 6;
 *    printf("%2X \n",data);
 *    break;
 *  case 6:
 *    msg_rx[msg_index].length = 0x00FF & data;
 *    valcheck = CRC16Value(data);
 *    step = 7;
 *    printf("%2X \n",data);
 *    break;
 *  case 7:
 *    msg_rx[msg_index].length = ((uint16_t)data << 8) + msg_rx[msg_index].length;
 *    msg_rx[msg_index].payload_offset = 0;
 *    valcheck = CRC16Value(data);
 *    step = 8;
 *    printf("%2X \n",data);
 *    break;
 *  case 8:
 *    if (msg_rx[msg_index].payload_offset < sizeof(msg_rx[msg_index].payload) - 1)
 *    //msg_rx.length -= 8;
 *    //if ((msg_rx.length < sizeof(msg_rx.payload) - 1) && (msg_rx.length --)>=0)
 *    {
 *        msg_rx[msg_index].payload[msg_rx[msg_index].payload_offset++] = data;
 *        valcheck = CRC16Value(data);
 *    }
 *
 *    if(msg_rx[msg_index].payload_offset >= msg_rx[msg_index].length - 8)
 *    {
 *        step = 9;
 *    }
 *    printf("%2X l:%d\n",data,msg_rx[msg_index].length);
 *    break;
 *  case 9:
 *    if((valcheck >>8) == data)
 *    {
 *        step = 10;
 *    }
 *    else
 *    {
 *        step = 0;
 *    }
 *    printf("%2X val:%X\n",(uint8_t)(valcheck>>8),valcheck);
 *    break;
 *  case 10:
 *    if((valcheck & 0x00FF) == data)
 *    {
 *        msg_complete = true;
 *        msg_rx[msg_index].is_presence = true;
 *        msg_index++;
 *
 *        if(msg_index >= 3)
 *        {
 *            msg_index = 0;
 *        }
 *    }
 *    else
 *    {
 *        msg_complete = false;
 *    }
 *    step = 0;
 *    printf("%2X \n",(uint8_t)(valcheck&0x00FF));
 *    break;
 *  default:
 *    break;
 *  }
 *}
 */

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
    view.height = copter.inertial_nav.get_position().z;
}

void AP_NewBroadcast :: update_view_altitude()
{
    view.altitude = copter.gps.location().alt;
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

    update_view();

    char * p = makeViewJson();

    if(NULL == p)
    {
        return;
    }

	if(copter.g.can_test_rt)
    {
		read();
    }
	else
    {
        sendJson(p);
        free(p);
    }
}


/*Calculate a CRC value to be used by CRC calculation functions. */
/*
 *#define CRC16_POLYNOMIAL   0xA001
 *uint16_t
 *AP_NewBroadcast ::CRC16Value(uint8_t data)
 *{
 *
 *    CRC ^= data;
 *    for (uint8_t i = 0; i < 8; i++)
 *    {
 *        if ( CRC & 1 )
 *            CRC = ( CRC >> 1 ) ^ CRC16_POLYNOMIAL;
 *        else
 *            CRC >>= 1;
 *    }
 *
 *    return CRC;
 *}
 */
#endif

