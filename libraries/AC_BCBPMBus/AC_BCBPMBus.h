/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef AC_BCBPMBUS_H
#define AC_BCBPMBUS_H

#include <inttypes.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>                // ArduPilot Mega Vector/Matrix math Library

//	added by ZhangYong 20160414
#include <AP_SerialManager/AP_SerialManager.h>
#include <stdio.h>
//#include <../././ArduCopter/defines.h>



// Other values normally set directly by mission planner

#define AC_BATT_CAPACITY_DEFAULT            3300
#define AC_BATT_LOW_VOLT_TIMEOUT_MS         10000   // low voltage of 10 seconds will cause battery_exhausted to return true


#define BCBPMBUS_STATUS_HEADA					0xAA
#define BCBPMBUS_STATUS_HEADB					0x55

#define BCBPMBUS_STATUS_ENDA					0xA5
#define BCBPMBUS_STATUS_ENDB					0x5A



#define AC_BCBPMBUS_STATUS_TIMEOUT_MICROS		3000

#define AC_BCBPMBUS_STATUS_TIMEOUT				250   

#define AC_BCBPMBUS_MODULE_MAX_COMPONENT		4

#define AC_BCBPMBUS_FRAME_LENGTH_OFFSET			3
#define AC_BCBPMBUS_FRAME_PAYLOAD_OFFSET		10
#define AC_BCBPMBUS_FRAME_CRC_OFFSET			28

#define AC_BCBPMBUS_FRAME_VALIDE_LENGTH			31
#define AC_BCBPMBUS_FRAME_INVALIDE_LENGTH		13

#define AC_BCBPMBUS_FRAME_SOURCE_ID				0xFF
#define AC_BCBPMBUS_FRAME_SOURCE_COMPONENT1		0x32
#define AC_BCBPMBUS_FRAME_SOURCE_COMPONENT2		0x33
#define AC_BCBPMBUS_FRAME_SOURCE_COMPONENT3		0x34


#define AC_BCBPMBUS_FRAME_DEST_ID				0x01
#define AC_BCBPMBUS_FRAME_DEST_COMPONENT		0x00

#define AC_BCB_PMBUS_FRAME_STATUS_VALIDE		0X01
#define AC_BCB_PMBUS_FRAME_STATUS_INVALIDE		0X00







class AC_BCBPMBus
{
public:

    /// Constructor
    AC_BCBPMBus(AP_SerialManager &_serial_manager);

    /// Initialize the battery monitor
    //modified by ZhangYong 20160413
 //   void init();
	//	modifed end

	
	enum Log_id
	{
		LOG_PMBUS0_MSG = 247,
		LOG_PMBUS1_MSG,
		LOG_PMBUS2_MSG,
		LOG_PMBUS3_MSG
	};
		
	typedef struct PACKED BCBPMBus_modules_struct {
		uint8_t occupied;
		uint32_t last_read;
		uint8_t seq;
		uint8_t component_id;

		uint8_t status_byte;					//	10
		uint8_t status_word;					//	11
		uint8_t status_iout;					//	12
		uint8_t status_input;					//	13
		uint8_t status_temperature;				//	14
		uint8_t status_cml;						//	15
		
		uint16_t read_vin;
		uint16_t read_iin;
		uint16_t read_vout;
		uint16_t read_iout;
		uint16_t read_temperature;
		uint16_t read_pout;

		bool  should_log;
		bool  new_data;
		bool  should_report[MAVLINK_COMM_NUM_BUFFERS];
}BCBPMBUS_MODULES;
/*
	typedef struct PACKED BCBPMBus_component_info_struct {
		uint8_t component_id;

		uint8_t status_byte;					//	10
		uint8_t status_word;					//	11
		uint8_t status_iout;					//	12
		uint8_t status_input;					//	13
		uint8_t status_temperature;				//	14
		uint8_t status_cml;						//	15
		
		uint16_t read_vin;
		uint16_t read_iin;
		uint16_t read_vout;
		uint16_t read_iout;
		uint16_t read_temperature;
		uint16_t read_pout;
}BCBPMBUS_COMPONENT_INFO;

*/
	


	void init();

    /// Read the battery voltage and current.  Should be called at 10hz
    void read();

	void parse_bcbpmbus_status(uint8_t data);
	uint8_t crc_calculate_bcbpmbus(uint8_t* pBuffer, uint8_t length);
	uint32_t bcbpmbus_status_lasttime();

	uint8_t pmbus_is_OK();

	uint8_t component_is_OK(uint8_t slot, uint8_t *reason);

	//	uint8_t *slot which module is not ok
	//	uint8_t *reason
	//	0: OK
	//	1: input voltage lower than 352 (middle priority)
	//	2: output voltage lower than 44 (low priority)
	//	3: temprature higher than 80	(high priority)
	//	4: iout lower than 1			(higher priority)
	//	5: timeout						(highest priority)


	uint8_t components_is_OK(uint8_t *slot, uint8_t *reason);

	uint32_t get_last_read(uint8_t idx);
	uint8_t get_component_seq(uint8_t idx);
	uint8_t get_component_id(uint8_t idx);
	uint8_t get_status_byte(uint8_t idx);
	uint8_t get_status_word(uint8_t idx);
	uint8_t get_status_iout(uint8_t idx);
	uint8_t get_status_input(uint8_t idx);
	uint8_t get_status_temperature(uint8_t idx);
	uint8_t get_status_cml(uint8_t idx);

	/// Battery voltage.  Initialized to 0
    uint16_t get_read_vin(uint8_t idx);

	/// Battery voltage.  Initialized to 0
    uint16_t get_read_iin(uint8_t idx);

	uint16_t get_read_vout(uint8_t idx);

	uint16_t get_read_iout(uint8_t idx);

	uint16_t get_read_temperature(uint8_t idx);

    uint16_t get_read_pout(uint8_t idx);

	int16_t get_fs_iout_thr_min() {return _fs_iout_thr_min.get();}
	uint8_t get_fs_enable() {return (uint8_t)_fs_enable.get();}

	int8_t enabled(){return _enable;};
	uint8_t initialised() {return _initialised; };

	uint8_t log_id_2_component_id(uint8_t log_id);

	uint8_t component_id_2_log_id(uint8_t component_id);

//	bool get_component_slot_info(uint8_t index);

	bool find_component_slot(uint8_t componnet_id, uint8_t *idx);

	bool find_available_component_slot(uint8_t *idx);

	bool set_component_slot_data(uint8_t idx);

	bool printf_component_slot_data(uint8_t idx);
	bool printf_component_slot_info(uint8_t idx);

	bool should_log_componengt_slot_info(uint8_t idx);

	
	bool should_report_componengt_slot_info(uint8_t idx, uint8_t chan);

	void componengt_slot_info_logged(uint8_t idx);

	void componengt_slot_info_reported(uint8_t idx, uint8_t chan);

	bool log_component_slot_info(uint8_t idx);

	bool set_available_component_slot_occupy(uint8_t idx);

	


	static const struct AP_Param::GroupInfo var_info[];
	
protected:

    /// parameters
 
    /// internal variables
 //   float       _voltage;                   /// last read voltage
    
 //   float       _current_amps;              /// last read current drawn
 //   float       _current_total_mah;         /// total current drawn since startup (Amp-hours)

	//	added by ZhangYong 20150601
//	float		_power_w;					///	last power by last read voltage * last read current drawn
//	uint32_t	_power_mw;					///	last power by last read voltage * last read current drawn
//	uint32_t	_capacity_consumed;			///	
	//	added end
	
    uint32_t    _last_time_micros;          /// time when current was last read
    uint32_t    _low_voltage_start_ms;      /// time when voltage dropped below the minimum



	struct PACKED BCBPMBus_frame_struct {
		uint8_t headA;							//	0
		uint8_t headB;							//	1

		uint8_t seq;							//	2

		uint8_t length;							//	3

		uint8_t msg_id;							//	4
		
		uint8_t source_id;						//	5
		uint8_t source_component;				//	6


		uint8_t dest_id;						//	7
		uint8_t dest_component;					//	8

		uint8_t status;							//	9

		uint8_t status_byte;					//	10
		uint8_t status_word;					//	11
		uint8_t status_iout;					//	12
		uint8_t status_input;					//	13
		uint8_t status_temperature;				//	14
		uint8_t status_cml;						//	15

		uint8_t read_vin_h;						//	16
		uint8_t read_vin_l;						//	17

		uint8_t read_iin_h;						//	18
		uint8_t read_iin_l;						//	19
		
		uint8_t read_vout_h;					//	20
		uint8_t read_vout_l;					//	21

		uint8_t read_iout_h;					//	22
		uint8_t read_iout_l;					//	23

		uint8_t read_temperatur_1_h;			//	24
		uint8_t read_temperatur_1_l;			//	25

		uint8_t read_pout_h;					//	26
		uint8_t read_pout_l;					//	27
		
		uint8_t crc;							//	28

		uint8_t endA;							//	29
		uint8_t endB;							//	30
	};

	union PACKED BCBPMBus_frame_status {
		BCBPMBus_frame_struct data;
		uint8_t bytes[];
	} _bcbpmbus_buffer;

//	uint32_t reserved;

	BCBPMBUS_MODULES _bcbpmbus_status[AC_BCBPMBUS_MODULE_MAX_COMPONENT];

//	BCBPMBUS_COMPONENT_INFO _bcbpmbus_component_info, *_bcbpmbus_component_info_ptr;

	struct PACKED BCBPMBus_statistics_err_struct {
		uint32_t bcbpmbus_err_cnt;							//	0
		uint32_t bcbpmbus_ok_cnt;
	}_bcbpmbus_sttc_err, *_bcbpmbus_sttc_err_ptr;



	uint32_t _bcbpmbus_status_timeout;
	uint32_t _bcbpmbus_status_timeout_cnt;
	uint32_t _bcbpmbus_status_lasttime;
	bool 	 _bcbpmbus_status_timeout_switch;

	uint8_t _initialised;

	uint8_t _step;

	bool _status_passed;


	void read_incoming();

	AP_HAL::UARTDriver *_port;

	uint8_t test_var;

	
	
private:
	
	// parameters
	AP_Int8 _enable;
    AP_Int16 _fs_vin;
	AP_Int16 _fs_vout;
	AP_Int16 _fs_temp;
	AP_Int16 _fs_iout;
//	AP_Int8  _fs_enable;
	AP_Int16 _fs_threshold;
	AP_Int8 _fs_enable;
	AP_Int16 _fs_iout_thr_min;
	
	AP_SerialManager &serial_manager;
};
#endif  // AC_BCBMonitor_H
