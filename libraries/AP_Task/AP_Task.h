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
#include "AP_ChargingStation.h"
#include "TaskDevice.h"


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
    // 
    // // handle a LED_CONTROL message
    // static void handle_led_control(mavlink_message_t* msg);
    // 
    // // handle a PLAY_TUNE message
    // static void handle_play_tune(mavlink_message_t* msg);
    // 
    // bool buzzer_enabled() const { return _buzzer_enable; }
    // 
    // // set flight mode string
    // void set_flight_mode_str(const char *str);
    // const char* get_flight_mode_str() const { return _flight_mode_str; }
    // 
    // // send text to display
    // void send_text(const char *str);
    // const char* get_text() const { return _send_text; }

    AP_ChargingStation &get_chargingStation() {return chargingStation;}
    static const struct AP_Param::GroupInfo var_info[];

private:

    static AP_Task *_instance;
    
    AP_ChargingStation chargingStation;
    // // parameters
    // AP_Int8 _rgb_led_brightness;
    // AP_Int8 _rgb_led_override;
    // AP_Int8 _buzzer_enable;
    // AP_Int8 _display_type;
    // AP_Int8 _oreo_theme;
    // 
    // char _send_text[NOTIFY_TEXT_BUFFER_SIZE];
    // uint32_t _send_text_updated_millis; // last time text changed
    // char _flight_mode_str[5];

    static TaskDevice* _devices[];
};
