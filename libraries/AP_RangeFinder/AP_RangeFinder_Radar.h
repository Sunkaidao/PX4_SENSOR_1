#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"




class AP_RangeFinder_Radar: public AP_RangeFinder_Backend
{


public:
    // constructor
    AP_RangeFinder_Radar(RangeFinder &ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state,
                                   AP_SerialManager &serial_manager);

    // static detection function
    static bool detect(RangeFinder &ranger, uint8_t instance, AP_SerialManager &serial_manager);

    // update state
    void update(void);
	
private:
    // get a reading
    bool get_reading(uint16_t &reading_cm);
    uint32_t last_reading_ms = 0;
    AP_HAL::UARTDriver *uart = nullptr;
	struct Time_Radar_error
	{
		uint8_t Time_Head_error;
		uint8_t Time_Invalid_data;
		uint8_t Time_Payload_error;
		uint8_t Time_Tail_error;
	}_num_error;

};

