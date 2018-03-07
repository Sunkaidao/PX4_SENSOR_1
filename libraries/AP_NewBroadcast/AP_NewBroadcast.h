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
#include <AP_Param/AP_Param.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>
#include "./../../ArduCopter/config.h"
#include <uavcan/helpers/heap_based_pool_allocator.hpp>
#include "cJSON.h"

#if NEWBROADCAST == ENABLED 

using namespace PX4;

#define REG_NO_STRING_LEN 32
#define PAYLOAD_ARRAY_LEN 98

typedef struct
{
    int32_t action;
    char reg_no[REG_NO_STRING_LEN]; /* temporary buffer to print the number into */
    int32_t flight_seq;
    uint64_t now_time;
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

#pragma  pack (push,1)
typedef struct
{
	uint8_t frame_header0;
	uint8_t frame_header1;
    uint8_t action;
    char reg_no[REG_NO_STRING_LEN]; /* temporary buffer to print the number into */
    uint16_t flight_seq;
    uint64_t now_time;
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
	uint8_t reserved[14];
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
    AP_Int32 _flight_seq;
	AP_Int8 _reg_no_complete;        //0:UAV registration number is not written to complete,1:complete
	uint64_t timer;

    Message_info view;
	Message_send_union payload;

	static const uint16_t crc16tab[];
public:
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
	void update_payload();

	//Big-endian to Little-endian
	uint16_t endian_big_to_little16u(uint16_t date16u);
	int16_t endian_big_to_little16(int16_t date16);
	uint32_t endian_big_to_little32u(uint32_t date32u);
	int32_t endian_big_to_little32(int32_t date32);
	uint64_t endian_big_to_little64u(uint64_t date64u);

	void sendPayload(char *pArray);
	void sendPayload(char *pArray,uint16_t len);
	bool sendDataStream(frameType type,char *pArray,uint16_t index,uint8_t length);
	bool sendCrc16(frameType type);
	uint16_t crc16_ccitt(const char *buf, int len);


};
#endif

