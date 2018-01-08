#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"




class AP_RangeFinder_Radar_ZHIBO: public AP_RangeFinder_Backend
{


public:
    // constructor
    AP_RangeFinder_Radar_ZHIBO(RangeFinder::RangeFinder_State &_state,
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
    uint32_t last_reading_ms = 0;
    AP_HAL::UARTDriver *uart = nullptr;
	struct Time_Radar_ZHIBO_error
	{
		uint8_t Time_Head_error;
		uint8_t Time_Invalid_data;
		uint8_t Time_Payload_error;
		uint8_t Time_Tail_error;
	}_num_error;

};

