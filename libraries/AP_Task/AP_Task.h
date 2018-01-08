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
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#if CHARGINGSTATION == ENABLED
#include "AP_ChargingStation.h"
#endif
#include "TaskDevice.h"

#include "AP_ABMode.h"


class AP_Task
{

public:
    // Constructor
    AP_Task();   

    // get singleton instance
    static AP_Task *instance(void) {
        return _instance;
    }

    // initialisation
    void init();

    /// update - allow updates of leds that cannot be updated during a timed interrupt
    void update(void);
#if CHARGINGSTATION == ENABLED
    AP_ChargingStation &get_chargingStation() {return chargingStation;}
#endif

#if ABMODE == ENABLED
	AP_ABMode &get_abmode() {return abmode;}
#endif

    static const struct AP_Param::GroupInfo var_info[];

private:

    static AP_Task *_instance;
#if CHARGINGSTATION == ENABLED
    AP_ChargingStation chargingStation;
#endif

#if ABMODE == ENABLED
	AP_ABMode abmode;
#endif

    static TaskDevice* _devices[];
};
