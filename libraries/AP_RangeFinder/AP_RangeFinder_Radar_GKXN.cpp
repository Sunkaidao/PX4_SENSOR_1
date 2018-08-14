#include <AP_HAL/AP_HAL.h>
#include "AP_RangeFinder_Radar_GKXN.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>



extern const AP_HAL::HAL& hal;

AP_RangeFinder_Radar_GKXN::AP_RangeFinder_Radar_GKXN(RangeFinder::RangeFinder_State &_state,
                                                               AP_SerialManager &serial_manager) :
    AP_RangeFinder_Backend(_state)
{
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Radar_GKXN, 0);
    if (uart != nullptr) {
       uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Radar_GKXN, 0));
    }
}
bool AP_RangeFinder_Radar_GKXN::detect(AP_SerialManager &serial_manager)
{
    return serial_manager.find_serial(AP_SerialManager::SerialProtocol_Radar_GKXN, 0) != nullptr;
}
bool AP_RangeFinder_Radar_GKXN::get_reading(uint16_t &reading_cm)
{
	if (uart == nullptr) {
        return false;
		}
	int16_t nbytes = uart->available();
	bool valid=false;
	while (nbytes-- > 0)
	{
	//depend on 20180713 email of gkxn
		uint8_t data=uart->read();
		switch(message_state)
		{
			case 0:
				if(data==0x55)
					{
					checksum=0x00;
					checksum+=data;
					_num_error.Time_Head_error=0;
					_num_error.Time_Invalid_data=0;
					_num_error.Time_Checksum_error=0;
					k=0;
					address_code=0x00;
					frame_longth=0x00;
					message_state=1;
					}
				else
				{
				_num_error.Time_Head_error++;
				message_state=0;
				}
			break;
			case 1:
				checksum+=data;
				address_code=data;
				message_state=2;
			break;
			case 2:
				checksum+=data;
				frame_longth=data;
				message_state=3;
			break;
			case 3:
				checksum+=data;
				message_state=4;
			break;
			case 4:
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
			break;
			case 7:
				checksum+=data;
				Height=(data<<8);
				message_state=8;
			break;
			case 8:
				checksum+=data;
				Height+=data;
				message_state=9;
			break;
			case 9:
				checksum+=data;
				message_status=data;
				message_state=10;
			break;
			case 10:
				if(data==(checksum&0xff))
					{
					if ((Getdata()==Right_data)&&(Height!=0xffff))
						{
						valid=true;
						reading_cm=Height;
						}
					else
						{
						_num_error.Time_Invalid_data++;
						}
					}
				else
					{
					_num_error.Time_Checksum_error++;
					}
				message_state=0;
			break;
			default:
				_num_error.Time_Invalid_data++;
			break;
			}
		}
		return valid;
	}	
int AP_RangeFinder_Radar_GKXN::Getdata()
{
	if(address_code!=0x0C)
		{
			return Invalid_data;
		}
	if(frame_longth!=0x0B)
		{
			return Invalid_data;	
		}
	if(((message_status>>4)&0x01)==0x01)
		{
		return Invalid_data;
		}
	return Right_data;
}


void AP_RangeFinder_Radar_GKXN::update(void)
{
    if (get_reading(state.distance_cm)) {
        // update range_valid state based on distance measured
        last_reading_ms = AP_HAL::millis();
        update_status();
    } else if (AP_HAL::millis() - last_reading_ms > 200) {
		 set_status(RangeFinder::RangeFinder_NoData);
	}
}

