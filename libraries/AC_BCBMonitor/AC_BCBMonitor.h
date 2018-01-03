/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef AC_BCBMonitor_H
#define AC_BCBMonitor_H

#include <inttypes.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>                // ArduPilot Mega Vector/Matrix math Library

//	added by ZhangYong 20160414
#include <AP_SerialManager/AP_SerialManager.h>
#include <stdio.h>


// Other values normally set directly by mission planner

#define AP_BATT_CAPACITY_DEFAULT            3300
#define AP_BATT_LOW_VOLT_TIMEOUT_MS         10000   // low voltage of 10 seconds will cause battery_exhausted to return true


#define BCBMONITOR_STATUS_HEADA					0xAA
#define BCBMONITOR_STATUS_HEADB					0x55

#define BCBMONITOR_STATUS_ENDA					0xA5
#define BCBMONITOR_STATUS_ENDB					0x5A



#define AP_BCBMONITOR_STATUS_TIMEOUT_MICROS		5000

#define AP_BCBMONITOR_STATUS_TIMEOUT			250   




class AC_BCBMonitor
{
public:

    /// Constructor
    AC_BCBMonitor();

    /// Initialize the battery monitor
    //modified by ZhangYong 20160413
 //   void init();
	//	modifed end

	void init(const AP_SerialManager& serial_manager);

    /// Read the battery voltage and current.  Should be called at 10hz
    void read();

	void parse_bcbmonitor_status(uint8_t data);
	uint8_t crc_calculate_bcbmonitor(uint8_t* pBuffer, uint8_t length);
	uint32_t bcbmonitor_status_lasttime();

	bool battery_is_OK();

	/// Battery voltage.  Initialized to 0
    uint16_t voltageA();

	/// Battery voltage.  Initialized to 0
    uint16_t voltageB();

    

    /// Battery pack instantaneous currrent draw in amperes
    float current_amps();

	float power() const { return _power_w; }

    /// Total current drawn since start-up (Amp-hours)
    float current_total_mah() const { return _current_total_mah; }

    /// capacity_remaining_pct - returns the % battery capacity remaining (0 ~ 100)
    uint8_t capacity_remaining_pct();

    /// exhausted - returns true if the voltage remains below the low_voltage for 10 seconds or remaining capacity falls below min_capacity
    bool exhausted(float low_voltage, float min_capacity_mah);

	

	
	
protected:

    /// parameters
 
    /// internal variables
    float       _voltage;                   /// last read voltage
    
    float       _current_amps;              /// last read current drawn
    float       _current_total_mah;         /// total current drawn since startup (Amp-hours)

	//	added by ZhangYong 20150601
	float		_power_w;					///	last power by last read voltage * last read current drawn
	uint32_t	_power_mw;					///	last power by last read voltage * last read current drawn
	uint32_t	_capacity_consumed;			///	
	//	added end
	
    uint32_t    _last_time_micros;          /// time when current was last read
    uint32_t    _low_voltage_start_ms;      /// time when voltage dropped below the minimum



	struct PACKED BCBMonitor_status_struct {
		uint8_t headA;							//	0
		uint8_t headB;							//	1
		
		uint8_t source_id;						//	2
		
		uint8_t data_valid;						//	3
		
		uint8_t voltage_AL;						//	4
		uint8_t voltage_AH;						//	5

		uint8_t current_AL;						//	6
		uint8_t current_AH;						//	7

		uint8_t temp_AL;						//	8
		uint8_t temp_AH;						//	9

		uint8_t voltage_BL;						//	10
		uint8_t voltage_BH;						//	11

		uint8_t current_BL;						//	12
		uint8_t current_BH;						//	13

		uint8_t temp_BL;						//	14
		uint8_t temp_BH;						//	15

		uint8_t crc;							//	16

		uint8_t endA;							//	17
		uint8_t endB;							//	18
	};

	union PACKED BCBMonitor_status {
		BCBMonitor_status_struct data;
		uint8_t bytes[];
	} _bcbmonitor_buffer;

	struct PACKED BCBMonitor_batt_status_struct {
		
		uint16_t voltage_A;
		uint16_t current_A;
		uint16_t temp_A;
		uint16_t voltage_B;
		uint16_t current_B;
		uint16_t temp_B;
	}_bcbmonitor_batt_status, *_bcbmonitor_batt_status_ptr;

	struct PACKED BCBMonitor_statistics_err_struct {
		uint32_t bcbmonitor_err_cnt;							//	0
	}_bcbmonitor_sttc_err, *_bcbmonitor_sttc_err_ptr;


	uint8_t  _bcbmonitor_status_counter;
	uint32_t _bcbmonitor_status_timeout;
	uint32_t _bcbmonitor_status_timeout_cnt;
	uint32_t _bcbmonitor_status_lasttime;
	bool 	 _bcbmonitor_status_timeout_switch;

	uint8_t _initialised;

	uint8_t _step;

	bool _status_passed;


	void read_incoming();

	AP_HAL::UARTDriver *_port;

	uint8_t test_var;

	
	
private:


	
};
#endif  // AC_BCBMonitor_H
