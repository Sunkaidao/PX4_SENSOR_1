#include <AP_HAL/AP_HAL.h>
#include "AP_RangeFinder_Radar_ZHIBO.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>



extern const AP_HAL::HAL& hal;

AP_RangeFinder_Radar_ZHIBO::AP_RangeFinder_Radar_ZHIBO(RangeFinder::RangeFinder_State &_state,
                                   AP_SerialManager &serial_manager) :
    AP_RangeFinder_Backend(_state)
{
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Radar_ZHIBO, 0);
    if (uart != nullptr) {
       uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Radar_ZHIBO, 0));
    }
}
bool AP_RangeFinder_Radar_ZHIBO::detect(AP_SerialManager &serial_manager)
{
    return serial_manager.find_serial(AP_SerialManager::SerialProtocol_Radar_ZHIBO, 0) != nullptr;
}
bool AP_RangeFinder_Radar_ZHIBO::get_reading(uint16_t &reading_cm)

{	bool valid=false;
	uint8_t nbytes = uart->available();
	if (uart == nullptr) {
		   return false;
	   }
	
	while (nbytes-- > 0)
	{

		uint8_t data=uart->read();
		switch(message_state)
		{
			case 0:
			if(data==0xfe)
			{
				_num_error.Time_Invalid_data=0;
				_num_error.Time_Tail_error=0 ; 
				_num_error.Time_Head_error=0;
				count=0;
				checksum=0x00;
				distance0=distance1=0;
				checksum+=data; 
				message_state=1;
			}
			else if(data==0xf0)
			{
				message_state=0;
				state.RangeFinder_no_target_count++;
				valid=true;
			}
			else
			{
			message_state=0;
			_num_error.Time_Head_error++;
			} 
		break;
		case 1:
			if((data>0)&&(data<4))
			{
			checksum+=data;
			message_state=2;
			}
			else 
			{
			message_state=0;
			_num_error.Time_Invalid_data++;
			}
		break;
		case 2:
			distance0=data;
			checksum+=data;
			message_state=3;
		break;
		case 3:
			distance1=data;
			checksum+=data;
			message_state=4;
		break;
		case 4:
			if(count<4)
			{
			checksum+=data;
			count++;
			message_state=4;
			}	
			else if(count==4)
			{
			if(data==(checksum&0x007f))
				{
				reading_cm=distance0*0x80+distance1;
				message_state=0; 
				valid=true;
				}
				else
				{
				message_state=0;
				_num_error.Time_Tail_error++;
				} 
			}
			else 
			{
				message_state=0;
				_num_error.Time_Invalid_data++;
			}
		break;
		default:
		break;
		}	
		
	}	
	if(valid==true)
		{
		state.RangeFinder_message_condition=1;
		return true;
		}
	else
		{
		   state.RangeFinder_unvalid_num++;
		state.RangeFinder_message_condition=0;
		if((_num_error.Time_Invalid_data!=0)||(_num_error.Time_Head_error!=0)||(_num_error.Time_Head_error!=0))
			{
				state.RangeFinder_error_count++;
			}
		return false;
		}
	}	
	

void AP_RangeFinder_Radar_ZHIBO::update(void)
{
    if (get_reading(state.distance_cm)) {
        // update range_valid state based on distance measured
        last_reading_ms = AP_HAL::millis();
        update_status();
	} else if (AP_HAL::millis() - last_reading_ms > 200) {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}

