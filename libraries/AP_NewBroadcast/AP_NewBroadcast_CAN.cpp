/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * @file AP_NewBroadcast_CAN.cpp
 * @author Breeder Bai <songshu_bai@icloud.com>
 *
 */

#include <AP_HAL/AP_HAL.h>
#include "AP_NewBroadcast_CAN.h"
#include "./../../ArduCopter/config.h"

extern const AP_HAL::HAL& hal;

AP_NewBroadcast_CAN::AP_NewBroadcast_CAN(Message_send_union &_payload) :
	payload(_payload),
	_parent_can_mgr(nullptr),
	send_index(0)
{
}

uint8_t AP_NewBroadcast_CAN::init()
{
	uint8_t res = 0;
	
	for (uint8_t i = 0; i < CAN_STM32_NUM_IFACES; i++)
	{
		if (hal.can_mgr[i] != nullptr && (hal.can_mgr[i]->get_UAVCAN() == nullptr)) 
		{
			_parent_can_mgr = (PX4CANManager *)hal.can_mgr[i];
			break;
		}
	}

	if(_parent_can_mgr == nullptr)
	{
		return res;
	}

	for (uint8_t i = 0; i < CAN_STM32_NUM_IFACES; i++)
	{
		if (_parent_can_mgr->getIface(i) != nullptr) 
		{
			Iface = _parent_can_mgr->getIface(i);
			break;
		}
	}

	if(Iface != nullptr)
	{
		res = 1;
	}

	//res, 0:failure,1:success
	return res;
}

uint8_t AP_NewBroadcast_CAN :: send(bool &send_flag,uint64_t &send_last_time)
{
    return send(payload.paylod_array,PAYLOAD_ARRAY_LEN,send_flag,send_last_time);
}

//Send paylod array and crc16
uint8_t AP_NewBroadcast_CAN :: send(char *pArray,uint16_t len,bool &send_flag,uint64_t &send_last_time)
{	
    uint16_t frame_num = len / 8;
    uint8_t last_frame_num_b = len % 8;

	if(Iface == nullptr)
	{
		return 0;
	}
	
    for(int16_t j = send_index; j<(frame_num+2); j++)
    {
    	 send_flag = PROCESSING;
		 send_index = j+1;
		
        if(!Iface->tx_pending())
        {
            if(j<frame_num)
            {
                 if(!sendDataStream(EXTENDID,pArray,j,8))
                 {
                    send_index = j;
                    j--;
                 }
            }
            else if(j == frame_num)
            {
                if(!sendDataStream(EXTENDID,pArray,j,last_frame_num_b))
                {
                    send_index = j;
                    j--;
                }
            }
            else
            {
                if(sendCrc16(EXTENDID))
                {
                    send_flag = COMPLETE;
					  send_index = 0;
					  send_last_time = AP_HAL::micros();
                }
				  else
				  {
				      send_index = j;
				  	  j--;
				  }
            }
        }
        else
        {
            send_index = j;
            j--;
			 break;
        }
    }

	return 1;
}

bool AP_NewBroadcast_CAN :: sendDataStream(frameType type,char *pArray,uint16_t index,uint8_t length)
{

    bool result = false;
    uint16_t j = index*8;

    uavcan::CanFrame frame;

    if (type == STANDARDID) {
        frame.id = uavcan::CanFrame::MaskStdID & 0x12;
    }
    else if(type == EXTENDID)
    {
        frame.id = uavcan::CanFrame::MaskExtID & 0x12;
        frame.id |= uavcan::CanFrame::FlagEFF;
    }

    frame.dlc = length & 0x0F;
    for(uint16_t i = 0;i<length;i++)
    {
        frame.data[i] = uint8_t(0XFF & pArray[j]);
        j++;
    }

	if(Iface == nullptr)
		return false;
	
    result =  Iface->send_rf(frame, uavcan::MonotonicTime::fromUSec(1), uavcan::CanIOFlagAbortOnError);

    return result;
}

bool AP_NewBroadcast_CAN :: sendCrc16(frameType type)
{
    bool result = false;
	uint16_t crc16;

    uavcan::CanFrame frame;

    if (type == STANDARDID) {
        frame.id = uavcan::CanFrame::MaskStdID & 0x12;
    }
    else if(type == EXTENDID)
    {
        frame.id = uavcan::CanFrame::MaskExtID & 0x12;
        frame.id |= uavcan::CanFrame::FlagEFF;
    }

	crc16 = crc16_ccitt(payload.paylod_array,PAYLOAD_ARRAY_LEN);

    frame.dlc = 2;

    frame.data[0] = crc16 & 0x00ff;
	frame.data[1] = (crc16 & 0xff00) >> 8;

	if(Iface == nullptr)
		return false;
	
    result =  Iface->send_rf(frame, uavcan::MonotonicTime::fromUSec(1), uavcan::CanIOFlagAbortOnError);

    return result;

}


