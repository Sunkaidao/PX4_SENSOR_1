/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef AP_PassOSD_H
#define AP_PassOSD_H

#if PJTPASSOSD == ENABLED

#include <inttypes.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>                // ArduPilot Mega Vector/Matrix math Library


//	added by ZhangYong 20160414
#include <AP_SerialManager/AP_SerialManager.h>
#include <stdio.h>
#include <../././ArduCopter/config.h>


//	added end

#define AP_PassOSD_SEND_INTERVAL_MS   1000    // resend angle targets to gimbal once per second


//	added end


class AP_PassOSD
{
public:

    /// Constructor
    AP_PassOSD();

    /// Initialize the battery monitor
    //modified by ZhangYong 20160413
 //   void init();
	//	modifed end

	void init(const AP_SerialManager& serial_manager);

	uint16_t crc_calculate(uint8_t* pBuffer, uint8_t length);

	bool message_can_send();

	void send_message(PassOSD_data_status *pt_data_ptr);


protected:

   union PassOSD_data_status _passosd_data_buffer;

	

	

	AP_HAL::UARTDriver *_port;

	bool _initialised;

	bool _port_hw_fc_ok;

	uint32_t _last_send_time_ms;


	


	void read_incoming();

	

	
};
#endif  // AP_BATTMONITOR_H

#endif
