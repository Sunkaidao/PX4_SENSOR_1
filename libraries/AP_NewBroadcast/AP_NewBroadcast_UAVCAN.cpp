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
#include "AP_NewBroadcast_UAVCAN.h"
#include "./../../ArduCopter/config.h"

extern const AP_HAL::HAL& hal;

AP_NewBroadcast_UAVCAN::AP_NewBroadcast_UAVCAN(Message_send_union &_payload) :
	payload(_payload),
	_parent_can_mgr(nullptr)
{
}

uint8_t AP_NewBroadcast_UAVCAN::init()
{
	uint8_t res = 0;
	
	for (uint8_t i = 0; i < CAN_STM32_NUM_IFACES; i++)
	{
		if (hal.can_mgr[i] != nullptr && (hal.can_mgr[i]->get_UAVCAN() != nullptr)) 
		{
			_parent_can_mgr = (PX4CANManager *)hal.can_mgr[i];
			break;
		}
	}

	if(_parent_can_mgr == nullptr)
	{
		return res;
	}

	if(_parent_can_mgr->get_UAVCAN() != nullptr)
	{
		_uavcan = _parent_can_mgr->get_UAVCAN();
		res = 1;
	}

	//res, 0:failure,1:success
	return res;
}

uint8_t AP_NewBroadcast_UAVCAN :: send(bool &send_flag,uint64_t &send_last_time)
{
	int res = 0;
	
	if(_uavcan->nbc_out_sem_take())
	{
		_uavcan->nbc_out_send(payload,send_flag,send_last_time);
		_uavcan->nbc_actuators(true);
		res = 1;
		_uavcan->nbc_out_sem_give();
	}
	else
	{
		res = 0;
	}

	return res;
}

