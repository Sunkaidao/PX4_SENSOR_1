#pragma once

#include <stdio.h>
#include "TaskDevice.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AC_WPNav/AC_WPNav.h>
#include <DataFlash/DataFlash.h>
#include "AP_Mission/AP_Mission.h"
#include <AP_InertialNav/AP_InertialNav.h>
#include <AP_SerialManager/AP_SerialManager.h>


#define YAW          0
#define POSITION     1

#if CHARGINGSTATION == ENABLED

class AP_ChargingStation : public TaskDevice
{
public:
	enum ChargingStation_CMD {
		Open_Communication_H = 0x00, //闁俺顔嗗锟介崥锟?x00
		Open_Communication_L = 0x00,
		Landing_Request_H = 0x00,	//鐠囬攱鐪伴梽宥堟儰0x01
		Landing_Request_L = 0x01,
		Landing_Ready_H = 0x80,		//闂勫秷鎯ら崙鍡楊槵鐏忚京鍗?x02
		Landing_Ready_L = 0x02,
		Landed_H = 0x00,			//瀹告煡妾烽拃锟?x03
		Landed_L = 0x03,
		Launch_Request_H = 0x00,	//鐠囬攱鐪伴崡鍥┾敄0x04
		Launch_Request_L = 0x04,
		Launch_Ready_H = 0x80,		//閸楀洨鈹栭崙鍡楊槵鐏忚京鍗?x05
		Launch_Ready_L = 0x05,
		Launched_H = 0x00,			//瀹告彃宕岀粚锟?x06
		Launched_L = 0x06,
		State_Request_H = 0x00,		//鐠囬攱鐪伴張铏圭崼閻樿埖锟斤拷0x07
		State_Request_L = 0x07,

		Coordinator_Address_H = 0x00,  	//閸楀繗鐨熼崳銊ユ勾閸э拷
		Coordinator_Address_L = 0x00,
		Broadcast_Address_H = 0x00, 	//楠炴寧鎸遍崷鏉挎絻
		Broadcast_Address_L = 0x00,

		Channel = 0x00
	};
	enum error_ChargingStation {
		Right_data = 0,
		Head_error = 1,
		Invalid_data = 2,
		Parity_error = 3
	};

	enum Flight_status {
		flight_default = 0,
		landing_complete = 1,
		flying_complete = 2,
		land_spare_position = 55,
		startland = 56,
		land = 57,
		hover = 58,
		back_to_station = 59,
		startcommunication = 60,
		end = 127
	};

	enum Communication_status {
		flight_other = 0,
		comm_end = -1,
		
		comm_success = 1,
		comm_failure = 2,
		comm_refuse  = 3,
		comm_noresponse = 4,

		reqL_success = 5,
		reqL_failure = 6,
		reqL_refuse  = 7,
		reqL_noresponse = 8,

		reaL_success = 9,
		reaL_failure = 10,
		reaL_noresponse = 12,

		comL_success = 13,
		comL_failure = 14,
		re_landed	 = 15,
		comL_noresponse = 16,

		reqF_success = 17,
		reqF_failure = 18,
		reqF_refuse  = 19,
		reqF_noresponse = 20,

		reaF_success = 21,
		reaF_failure = 22,
		reaF_noresponse = 24,

		comF_success	= 25,
		comF_noresponse = 28,

		comL_other = 29,
		comF_other = 30,

	};

	struct _CMD_MSG
	{
	    uint8_t command;           //The contents of this operation(0:nothing,1:Open the charging station cover)
		uint8_t command_status;    //This command status(0:nothing,1:Communicating,2:Request to open the cover,3:Opening the cover,4:Has been opened,5:The takeoff is complete)
		uint8_t command_result;    //The result of this command(0:processing,1:success,2:failure,3:time out)</field>
		uint8_t station_status;    //station ststus,open or close
	}CMD_MSG;
	
	AP_ChargingStation() ;
	void init_data(void);
	void init_station_pos();
	void sendCMD(uint8_t CMD_Data_H,uint8_t CMD_Data_L);
	void receiveData(uint8_t Data);
	int data_deal_S();
	int data_deal_R();
	void update();
	bool init();
	void landed(){fli_status = landing_complete;}
	void fly(){fli_status = flying_complete;}
	void reset_flight_status();
	void set_blastoff_flag();
	void set_receive_takeoff();
	void start_communication(){blastoff_flag = 2;}
	int32_t get_lat_station(){return lat_station.get();}
	int32_t get_lng_station(){return lng_station.get();}
	void set_lat_station(int32_t lat){lat_station.set_and_save_ifchanged(lat);}
    void set_lng_station(int32_t lng){lng_station.set_and_save_ifchanged(lng);}
	bool get_flight_permit();
	uint8_t get_repeat(){return repeat;}
	void reset(){blastoff_flag = -1;}//printf("%d\n",6);
	void reset_CMD_MSG();
	void set_CMD_MSG(Communication_status _status);
	void set_cmd(uint8_t type);
//	bool back_to_stationMidair();
	bool do_gotostation();
	bool land_alternate_positionMidair();
	bool do_takeoff();
	int8_t get_Bstation_use(){return Bstation_use.get();}
	int analytic_protocol();
	void time_check();
	int16_t get_timeout(){return timeout;}
	~AP_ChargingStation();

	static const struct AP_Param::GroupInfo     var_info[];
protected:
	AP_HAL::UARTDriver *_port;
		
	bool _initialised;

private:
	struct AP_Mission::Mission_Command cmd = {};
		
	AP_Int8 Timeout;
    AP_Int32 comm_alt;
	AP_Int8 comm_repetition;
	AP_Int8 Reql_repetition;
	AP_Int8 Real_repetition;
	AP_Int8 coml_repetition;
	AP_Int8 reqf_repetition;
	AP_Int8 reaf_repetition;
	AP_Int8 comf_repetition;
	AP_Int8 agal_repetition;
	AP_Int32 lng_station;
	AP_Int32 lat_station;
	AP_Int32 distance;
	AP_Int8 direction;
	AP_Int8 Bstation_use;
	AP_Int32 station_range;
	AP_Int8 arm_check;
	AP_Int16 sta_time_out;
	AP_Int16 comm_fly_alt;
	uint8_t Rx_Buff[20];
	uint8_t MSG;
    int8_t repeat_num[9];
	uint64_t tasktime;

	bool readyland_flag;
	bool readyfly_flag;
	bool againland_flag;
	bool back_to_station_midair;
	bool land_alternate_posMidair;
	bool startlanded;
	bool chargingStation_open;
	bool receive_takeoff;
	int8_t blastoff_flag;
  int8_t landtimes;
	int8_t fli_status;
	int16_t timeout;
	
	struct _DATA_Struct
	{
	    uint8_t source;
		uint8_t Instruction;
		uint8_t D8;
	}_chargingStation_data;
	struct Time_chargingStation_error
	{
		uint8_t Time_Head_error;
		uint8_t Time_Invalid_data;
		uint8_t Time_Parity_error;
		uint8_t Fail_time;
		uint8_t Timeout_time;
	}_num_error;
	uint8_t cmd_station;
	uint8_t send_cmd;
	uint8_t repeat;
	uint8_t state_machine;
	uint8_t receive_complete_frag;
	uint8_t sumchkm,length;
};
#endif
