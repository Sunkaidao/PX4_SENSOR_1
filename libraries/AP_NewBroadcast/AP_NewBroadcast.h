/*
 *
 *      Author: Breeder Bai
 */
#pragma once


#include <uavcan/uavcan.hpp>

#include <string.h>
#include "stdio.h"
#include <AP_HAL/CAN.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL_PX4/CAN.h>
#include <AC_Sprayer/AC_Sprayer.h>
#include <AP_Param/AP_Param.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>
#include "./../../ArduCopter/config.h"
#include <uavcan/helpers/heap_based_pool_allocator.hpp>

#if NEWBROADCAST == ENABLED 

using namespace PX4;

#define REG_NO_STRING_LEN 20
#define PAYLOAD_ARRAY_LEN 148
#define FLIGHT_CONTROL_STRING_LEN 16
#define TASK_ID_STRING_LEN 16
#define APP_VER_NO_STRING_LEN 16
#define RESERVED_NUM 14

#define PROCESSING 0
#define COMPLETE 1 

typedef struct
{
    uint8_t action;
    char reg_no[REG_NO_STRING_LEN]; /* temporary buffer to print the number into */
    uint16_t flight_seq;
    uint32_t now_time;  //uint: s
    uint8_t state;
    uint32_t flight_time;
    int32_t longitude;
    int32_t latitude;
    int32_t height;
    int32_t altitude;
    int16_t path_angle;
    int16_t pitch_angle;
    int16_t roll_angle;
    int16_t horizontal_velocity;
    int16_t is_nozzle_work;
    int16_t nozzle_diameter;
    int16_t nozzle_angle;
    int16_t nozzle_pressure;
    int16_t spray_range;
	char flight_control[FLIGHT_CONTROL_STRING_LEN];
	char task_id[TASK_ID_STRING_LEN];
	char app_ver_no[APP_VER_NO_STRING_LEN];
	uint64_t tp_reg_no;
	uint16_t remain_dose;
	uint16_t used_dose;
	uint16_t cur_flow;
	uint16_t flight_area;    //uint: mu*10
	uint16_t flight_length;  //uint: m
}Message_info;

#pragma  pack (push,1)
typedef struct
{
	uint8_t frame_header0;
	uint8_t frame_header1;
    uint8_t action;
    char reg_no[REG_NO_STRING_LEN]; /* temporary buffer to print the number into */
    uint16_t flight_seq;
    uint32_t now_time;      //uint: s
    uint8_t state;
    uint32_t flight_time;
    int32_t longitude;
    int32_t latitude;
    int32_t height;
    int32_t altitude;
    int16_t path_angle;
    int16_t pitch_angle;
    int16_t roll_angle;
    int16_t horizontal_velocity;
    int16_t is_nozzle_work;
    int16_t nozzle_diameter;
    int16_t nozzle_angle;
    int16_t nozzle_pressure;
    int16_t spray_range;
	char flight_control[FLIGHT_CONTROL_STRING_LEN];
	char task_id[TASK_ID_STRING_LEN];
	char app_ver_no[APP_VER_NO_STRING_LEN];
	uint64_t tp_reg_no;
	uint16_t remain_dose;
	uint16_t used_dose;
	uint16_t cur_flow;
	uint16_t flight_area;    //uint: mu*10
	uint16_t flight_length;  //uint: m
	uint8_t reserved[RESERVED_NUM];
}Payload_s;

typedef union
{
	Payload_s _payload_s;
	char paylod_array[PAYLOAD_ARRAY_LEN];
}Message_send_union;

#pragma pack(pop)  

enum frameType
{
    STANDARDID,
    EXTENDID
};

class AP_NewBroadcast {
public:
    AP_NewBroadcast(AC_Sprayer& _sprayer);
    ~AP_NewBroadcast();

	static const struct AP_Param::GroupInfo var_info[];

private:
    bool _initialized;
	PX4CANManager* _parent_can_mgr;
    AC_Sprayer & sprayer;

    AP_Int8  _enable;
	AP_Int8  _reg_no[REG_NO_STRING_LEN];
	//char  _reg_no[REG_NO_STRING_LEN];
    AP_Int32 _flight_seq;
	//AP_Int8 _reg_no_complete;        //0:UAV registration number is not written to complete,1:complete
	//bool _reg_no_complete;  
	int32_t flight_seq_pre;
	Location spraying_start;
	Location spraying_end;
	Location flight_seq_originate;
	Location flight_seq_current;
	uint64_t flight_area_m2_curr;	
	uint64_t flight_area_m2_pre;
	uint64_t timer;

    Message_info view;
	Message_send_union payload;

	bool send_flag;
	int16_t send_index;	
	uint64_t send_last_time;
		
	static const uint16_t crc16tab[];
public:
	void update();
	void init();

	void send_reg_no(mavlink_channel_t chan);
	void send_flight_status(mavlink_channel_t chan);
	uint16_t get_view_flight_area(){return view.flight_area;}
	uint16_t get_view_flight_length(){return view.flight_length;}
	
	MAV_RESULT handle_msg_newbroadcast_str(const mavlink_newbroadcast_str_t &packet,mavlink_channel_t chan);
    void update_view();
    void update_view_action();
    //void update_view_reg_no();
    void update_view_reg_no(const mavlink_newbroadcast_str_t &packet,mavlink_channel_t chan);
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
	void update_view_flight_control();
	void update_view_task_id(const mavlink_newbroadcast_str_t &packet,mavlink_channel_t chan);
	void update_view_app_ver_no(const mavlink_newbroadcast_str_t &packet,mavlink_channel_t chan);
	void update_view_tp_reg_no(const mavlink_newbroadcast_str_t &packet,mavlink_channel_t chan);
	void update_view_remain_dose();
	void update_view_used_dose();
	void update_view_cur_flow();
	void update_view_flight_area();
	void update_view_flight_length();
	void update_payload();

	void sendPayload(char *pArray);
	void sendPayload(char *pArray,uint16_t len);
	bool sendDataStream(frameType type,char *pArray,uint16_t index,uint8_t length);
	bool sendCrc16(frameType type);
	uint16_t crc16_ccitt(const char *buf, int len);


};
#endif

