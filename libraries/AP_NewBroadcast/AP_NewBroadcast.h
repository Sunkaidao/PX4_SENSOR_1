/*
 *
 *      Author: Breeder Bai
 */
#pragma once


#include <uavcan/uavcan.hpp>

#include <string.h>
#include "stdio.h"
#include <AP_HAL/CAN.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL_PX4/CAN.h>
#include <AC_Sprayer/AC_Sprayer.h>
//#include <AP_GPS/AP_GPS.h>
#include <AP_Param/AP_Param.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>
#include "./../../ArduCopter/config.h"
#include <uavcan/helpers/heap_based_pool_allocator.hpp>
#include "cJSON.h"

#if NEWBROADCAST == ENABLED

using namespace PX4;

/*
 *#define PAYLOAD_LEN_MAX 100
 *#define SEND_BUFF_LEN_MAX 100
 *#define MSG_INDEX_MAX 3
 */
#define CJSON_STRING_LEN_MAX  500

/*
 *typedef struct
 *{
 *    uint16_t msgid;
 *    uint8_t send_buff[SEND_BUFF_LEN_MAX];
 *}send_info;
 *
 *typedef struct
 *{
 *    uint16_t msgid;
 *    bool is_presence;
 *    uint8_t version;
 *    uint8_t length;
 *    uint8_t payload_offset;
 *    uint8_t payload[PAYLOAD_LEN_MAX];
 *}receive_info;
 *
 */

typedef struct
{
    int32_t action;
    char number_buffer[26]; /* temporary buffer to print the number into */
    int32_t flight_seq;
    int32_t now_time;
    //int32_t uav_id;
    int32_t state;
    int32_t flight_time;
    int32_t longitude;
    int32_t latitude;
    int32_t height;
    int32_t altitude;
    int32_t path_angle;
    int32_t pitch_angle;
    int32_t roll_angle;
    int32_t horizontal_velocity;
    int32_t is_nozzle_work;
    int32_t nozzle_diameter;
    int32_t nozzle_angle;
    int32_t nozzle_pressure;
    int32_t spray_range;
}Message_info;

enum frameType
{
    STANDARDID,
    EXTENDID
};

class AP_NewBroadcast {
public:
    AP_NewBroadcast(AC_Sprayer& _sprayer);
    ~AP_NewBroadcast();

    //void decode(uint8_t data);

    int8_t decode(char data);
    void parseJson(char * pMsg);

    char * makeViewJson();
    //JSON format output test
    char * makeJson();

    void sendJson(char *pJString);
    void sendJson(char *pJString,uint16_t len);
    bool sendString(frameType type,char *pJString,uint16_t index,uint8_t length);

    //separator symbol is \n\r
    bool sendSeparatorSymbol(frameType type);

	uint16_t CRC16Value(uint8_t data);

	static const struct AP_Param::GroupInfo var_info[];

private:
    bool _initialized;
	PX4CANManager* _parent_can_mgr;
    AC_Sprayer & sprayer;

    AP_Int32 _reg_no;
    AP_Int32 _flight_seq;
	uint64_t timer;

    /*
     *uint8_t msg_index;
	 *uint8_t step;
     *uint8_t send_step;
	 *uint16_t CRC;
	 *uint16_t valcheck;
	 *bool msg_complete;
	 *uint8_t payload[PAYLOAD_LEN_MAX];
	 *send_info msg_tx;
	 *receive_info msg_rx[MSG_INDEX_MAX];
     *
     */

    uint16_t payload_offset;
    char JString[CJSON_STRING_LEN_MAX];
    Message_info view;
    static char p[];
public:
	void send_msg(uint8_t id_mode,uint8_t frame_mode,uint8_t data_length);
	void handle_msg(int8_t);
    bool read();
    void send();

	//void send_info();
	void update();
	void init();

    void update_view();
    void update_view_action();
    void update_view_reg_no();
    void update_view_flight_seq();
    void update_view_now_time();
    void update_view_state();
    void update_view_flight_time();
    void update_view_longitude();
    void update_view_latitude();
    void update_view_height();
    void update_view_altitude();
    void update_view_path_angle();
    void update_view_pitch_angle();
    void update_view_roll_angle();
    void update_view_horizontal_velocity();
    void update_view_is_nozzle_work();
    void update_view_nozzle_diameter();
    void update_view_nozzle_angle();
    void update_view_nozzle_pressure();
    void update_view_spray_range();
};
#endif

