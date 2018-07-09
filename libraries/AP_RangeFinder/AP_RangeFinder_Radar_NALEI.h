#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"




class AP_RangeFinder_Radar_NALEI: public AP_RangeFinder_Backend
{


public:
    // constructor
    AP_RangeFinder_Radar_NALEI(RangeFinder::RangeFinder_State &_state,
                                   AP_SerialManager &serial_manager);

    // static detection function
    static bool detect(AP_SerialManager &serial_manager);

    // update state
    void update(void);
//added by xusiming 20180503 and used for collect log
	uint8_t get_no_target();
	uint8_t get_error_number();
//added end
protected:

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }
	
private:
    // get a reading
    bool get_reading(uint16_t &reading_cm);
    uint32_t last_reading_ms = 0;
    AP_HAL::UARTDriver *uart = nullptr;
	struct Time_Radar_NALEI_error
	{
		uint8_t Time_Head_error;
		uint8_t Time_Invalid_data;
		uint8_t Time_Payload_error;
		uint8_t Time_Tail_error;
	}_num_error;
	uint8_t checksum = 0x00;
	uint8_t message_state=0;
	uint8_t distance0=0;
	uint8_t distance1=0;
	uint8_t message_status=0;
	uint16_t message_id=0;
	uint8_t count=0;
	uint8_t check_number=0;
	uint8_t target_num=0;
//added by xusiming in 20180620 and used for nalei radar
	bool status_radar=false;
//added end
};

