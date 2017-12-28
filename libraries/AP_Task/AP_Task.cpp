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

#include "AP_Task.h"

#include "AP_ABMode.h"
#include "AP_ChargingStation.h"
#include "./../ArduCopter/Copter.h"
#include "TaskDevice.h"
#include <stdio.h>

AP_Task *AP_Task::_instance;

#define CONFIG_NOTIFY_DEVICES_COUNT 5

// table of user settable parameters
const AP_Param::GroupInfo AP_Task::var_info[] = {
    AP_GROUPEND
};

// Default constructor
AP_Task::AP_Task()
{
	AP_Param::setup_object_defaults(this, var_info);
    if (_instance != nullptr) {
        AP_HAL::panic("AP_Task must be singleton");
    }
    _instance = this;
}

TaskDevice *AP_Task::_devices[] = {nullptr, nullptr, nullptr, nullptr, nullptr};

// initialisation
void AP_Task::init()
{

#if CHARGINGSTATION == ENABLED
    _devices[0] = &chargingStation;
#endif

//baiyang added in 20171025
#if ABMODE == ENABLED
		_devices[1] = &abmode;
#endif
//added end
    
    for (uint8_t i = 0; i < CONFIG_NOTIFY_DEVICES_COUNT; i++) {
        if (_devices[i] != nullptr) {
            _devices[i]->pTask = this;
            _devices[i]->init();
        }
    }
}

// main update function, called at 50Hz
void AP_Task::update(void)
{
    for (uint8_t i = 0; i < CONFIG_NOTIFY_DEVICES_COUNT; i++) {
        if (_devices[i] != nullptr) {
            _devices[i]->update();
        }
    }
}
