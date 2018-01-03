#pragma once

#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <stdio.h>
#include <AP_Param/AP_Param.h>




#define AP_FLOWMETER_EXHAUSTED_SHRESHOLD		3

class AP_Flowmeter
{
public:
	enum Flowmeter_CMD {
		Read_Data = 0x00,
		LIQ_RESET = 0x01,
		Enter_Calib = 0x02,
		Quit_Calib = 0x03
	};
	enum error_Flowmeter {
		Right_data = 0,
		Head_error = 1,
		Invalid_data = 2,
		Parity_error = 3
	};
	AP_Flowmeter();

	int8_t enabled() {return _enabled.get();}
	
	void SendCMD(uint8_t CMD);
	int GetDate();
	void get_Flowmeter_data();
	void update(const AP_SerialManager& serial_manager);
	~AP_Flowmeter();
	void init(const AP_SerialManager& serial_manager);

	uint8_t get_warning();
	uint8_t get_packet_cnt();

	bool exhausted();

	static const struct AP_Param::GroupInfo var_info[];


protected:
	AP_HAL::UARTDriver *_port;
	
	bool _initialised;

private:
	// parameters
    AP_Int8         _enabled;               // top level enable/disable control
	
	uint8_t Tx_Buff[4];
	uint8_t Rx_Buff[11];

	struct Flowmeter_data
	{
		double high;
		double volume;
		uint8_t	warning;
		uint8_t	mode;
		uint8_t packet_cnt;
	}_Flo_data;


	
	struct Time_Flowmeter_error
	{
		uint8_t Time_Head_error;
		uint8_t Time_Invalid_data;
		uint8_t Time_Parity_error;
	}_num_error;
	uint8_t state_tim;

	
};


