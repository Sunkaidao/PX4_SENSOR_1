#include <AP_HAL/AP_HAL.h>
#include "AP_RangeFinder_Radar_NALEI.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>



extern const AP_HAL::HAL& hal;

AP_RangeFinder_Radar_NALEI::AP_RangeFinder_Radar_NALEI(RangeFinder::RangeFinder_State &_state,
                                                               AP_SerialManager &serial_manager) :
    AP_RangeFinder_Backend(_state)
{
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Radar_NALEI, 0);
    if (uart != nullptr) {
       uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Radar_NALEI, 0));
    }
}
bool AP_RangeFinder_Radar_NALEI::detect(AP_SerialManager &serial_manager)
{
    return serial_manager.find_serial(AP_SerialManager::SerialProtocol_Radar_NALEI, 0) != nullptr;
}
bool AP_RangeFinder_Radar_NALEI::get_reading(uint16_t &reading_cm)
{
	if (uart == nullptr) {
        return false;
    }
	_num_error.Time_Head_error=0;
	_num_error.Time_Tail_error=0;
	_num_error.Time_Invalid_data=0;
	_num_error.Time_Payload_error=0;
	
	bool   valid=false;
	int16_t nbytes = uart->available();
	while (nbytes-- > 0)
		{
		uint16_t data=uart->read();
		switch(message_state)
			{
			case 0:
				if(data==0xAA)
					{
					checksum = 0x00;
					message_state=1;
					count=0;
					distance0=0;
					distance1=0;
					message_id=0;
					count=0;
					target_num=0;
					check_number=0;
					}
				else
					{
					_num_error.Time_Head_error++;
					message_state=0;
					}
			break;
			case 1:
				if(data==0xAA)
					{
					message_state=2;
					}
				else
					{
					_num_error.Time_Head_error++;
					message_state=0;
					}
			break;
			case 2:
				message_id=data;
				message_state=3;
			break;
			case 3:
				message_id+=(data*0x100);
				message_state=4;
			break;
			case 4:
				target_num=data;
				checksum+=data;
				message_state=5;
			break;
			case 5:
				checksum+=data;
				message_state=6;
			break;
			case 6:
				checksum+=data;
				message_state=7;
				distance0=data;
			break;
			case 7:
				checksum+=data;
				message_state=8;
				distance1=data;
			break;
			case 8:
				if(count<2)
					{
					checksum+=data;
					message_state=8;
					count++;
					}
				else
					{
					checksum+=data;
					message_state=9;
					}
			break;
			case 9:
				check_number=data;
				message_state=10;
			break;
			case 10:
				if(data==0x55)
					{
					message_state=11;
					}
				else
					{
					_num_error.Time_Tail_error++;
					message_state=0;
					}
			break;
			case 11:
				//check the end of message
				if(data==0x55)
					{
					//check the checksum of message
					if(checksum==check_number)
						{
						//examine the id of message and update the status
						if((message_id==0x60A)&&(message_status==0))
							{
							message_status=1;
							message_state=0;
							}
						else if((message_id==0x70B)&&(message_status==1))
							{
							if(target_num!=0)
								{
								message_status=2;
								}
							else
								{
								message_status=0;
								state.RangeFinder_no_target_count++;
								status_radar=true;
								}
							message_state=0;
							}
						else if((message_id==0x70C)&&(message_status==2))
							{
							//make sure the status of message and get the distance 
							uint16_t distance=0;
							distance=distance0;
							reading_cm=distance*256+distance1;
							valid=true;
							message_state=0;
							message_status=0;
							}
						else
							{
							_num_error.Time_Invalid_data++;
							message_state=0;
							}
						}
					else
						{
						_num_error.Time_Payload_error++;
						message_state=0;
						}
					}
				else
					{
					_num_error.Time_Tail_error++;
					message_state=0;
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
		if((_num_error.Time_Head_error!=0)||(_num_error.Time_Tail_error!=0)||(_num_error.Time_Invalid_data!=0)||(_num_error.Time_Payload_error!=0))
			{
			state.RangeFinder_error_count++;
			}
		return false;
		}
	}	
	

void AP_RangeFinder_Radar_NALEI::update(void)
{
    if (get_reading(state.distance_cm)) {
        // update range_valid state based on distance measured
        last_reading_ms = AP_HAL::millis();
        update_status();
    } else if (AP_HAL::millis() - last_reading_ms > 200) {
        if(state.RangeFinder_working_condition){
			set_status(RangeFinder::RangeFinder_NoData);
		}
		else{
			if(status_radar==false){
				set_status(RangeFinder::RangeFinder_NoData);
			}
			else{
				set_status(RangeFinder::RangeFinder_Good);
				update_status();
			}
			
			}
    }
}

