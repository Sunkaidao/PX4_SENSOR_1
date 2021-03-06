#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"




class AP_RangeFinder_Radar_GKXN: public AP_RangeFinder_Backend
{


public:
    // constructor
    AP_RangeFinder_Radar_GKXN(RangeFinder::RangeFinder_State &_state,
                                   AP_SerialManager &serial_manager);

    // static detection function
    static bool detect(AP_SerialManager &serial_manager);

    // update state
    void update(void);
protected:

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }
	
private:
    // get a reading
    bool get_reading(uint16_t &reading_cm);
	int Getdata();
    uint32_t last_reading_ms = 0;
    AP_HAL::UARTDriver *uart = nullptr;
	struct Time_radar_GKXN_error
	{
		uint8_t Time_Head_error;
		uint8_t Time_Invalid_data;
		uint8_t Time_Checksum_error;
	}_num_error;
	enum error_radar_GKXN {
		Right_data = 0,
		Invalid_data = 1,
	};
	uint8_t message_state=0;
	uint16_t checksum = 0x00;
	int      k=0;
	uint8_t address_code=0x00;
	uint8_t frame_longth=0x00;
	uint16_t Height =0;
	uint8_t message_status=0;
	uint16_t d1=0x00;
	uint16_t d2=0x00;
};

