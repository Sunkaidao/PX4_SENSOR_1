/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#if PJTPASSOSD == ENABLED

#include "AP_PassOSD.h"

extern const AP_HAL::HAL& hal;

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AP_PassOSD::AP_PassOSD(void):
	_last_send_time_ms(0)
{

	_port = NULL;

	_initialised = false;

	_port_hw_fc_ok = false;


	memset(_passosd_data_buffer.bytes, 0, sizeof(struct PassOSD_data_struct));
	
}

// init - setup the battery and voltage pins
void
AP_PassOSD::init(const AP_SerialManager& serial_manager)
{
	//	modified by ZhangYong 20160413
    /*_volt_pin_analog_source = hal.analogin->channel(_volt_pin);
    _curr_pin_analog_source = hal.analogin->channel(_curr_pin);
    if (_volt2_pin != -1) {
        _volt2_pin_analog_source = hal.analogin->channel(_volt2_pin);
    } else {
        _volt2_pin_analog_source = NULL;
    }
	*/
	//	modified end

	_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_PassOSD, 0);

	_port_hw_fc_ok = false;

	if (nullptr != _port) 
	{
		//	added by zhangyong 20160219
//		printf("BD_MESSAGE initialize port %d\n", _port);
		//	added end
   		_initialised = true;



		memset(_passosd_data_buffer.bytes, 0, sizeof(struct PassOSD_data_struct));
				
	}
	else
	{
//		printf("BD_MESSAGE not initialize port\n");
	}

	
}


uint16_t AP_PassOSD::crc_calculate(uint8_t* pBuffer, uint8_t length)
{
	uint8_t lcl_cnt;

	uint16_t crc;

	crc = * pBuffer;

//	printf("\n");

	for(lcl_cnt = 1; lcl_cnt < length; lcl_cnt++)
	{
//		printf("%d crc 0x%2x : 0x%2x \n", lcl_cnt, crc, pBuffer[lcl_cnt]);
		crc = pBuffer[lcl_cnt] + crc;
//		printf("-> 0x%2x\n", crc);

		//printf("%d: 0x%x^0x%x=0x%x, ", lcl_cnt, crc_ptr[lcl_cnt], crc, temp_var);
		
		//crc ^= (*crc_ptr++);
//		crc = CRC8Table[temp_var];

//		printf("0x%x\n", crc);
		
		
	}

//	printf("last crc 0x%x\n", crc_ptr[lcl_cnt++]);

	

	return crc;
}




bool AP_PassOSD::message_can_send()
{
	if(false == _initialised)
	{
		return false;
	}

	if(_port->txspace() > sizeof(struct PassOSD_data_struct))
		return true;
	else
		return false;
}

void AP_PassOSD::send_message(PassOSD_data_status *pt_data_ptr)
{
	uint16_t local_crc;
	uint32_t local_uint32;
	static uint8_t lcl_cnt = 0;
//	int32_t local_int32;

	if(false == message_can_send())
		return;

	memset(_passosd_data_buffer.bytes, 0, sizeof(struct PassOSD_data_struct));

//	--------------------------------------------------
//	message head
//	--------------------------------------------------
	_passosd_data_buffer.data.stx0 = 0xEB;
	_passosd_data_buffer.data.stx1 = 0x90;
	
	

	
//	--------------------------------------------------
//	message length (byte)
//	--------------------------------------------------
	local_uint32 = sizeof(struct PassOSD_data_struct);
	_passosd_data_buffer.data.len = local_uint32;


//	--------------------------------------------------
//	message sequence
//	--------------------------------------------------	
	_passosd_data_buffer.data.seq = lcl_cnt++;


//	--------------------------------------------------
//	message source address
//	--------------------------------------------------	
	_passosd_data_buffer.data.source_id = 0;
	_passosd_data_buffer.data.destination_id = 0;

	

//	--------------------------------------------------
//	message source address
//	--------------------------------------------------

	
	_passosd_data_buffer.data.comp_id = 0;


//	--------------------------------------------------
//	packet type
//	--------------------------------------------------
	_passosd_data_buffer.data.message_id = 0x46;

//	--------------------------------------------------
//	destination address
//	--------------------------------------------------		

	
//	_passosd_data_buffer.data.voltage_battery = pt_data_ptr->data.voltage_battery;
//	_passosd_data_buffer.data.current_total_mah = pt_data_ptr->data.current_total_mah;


	
//	_passosd_data_buffer.data.fix_type = pt_data_ptr->data.fix_type;/
//	_passosd_data_buffer.data.num_sats = pt_data_ptr->data.num_sats;
//	_passosd_data_buffer.data.control_mode = pt_data_ptr->data.control_mode;
//	_passosd_data_buffer.data.velocity = pt_data_ptr->data.velocity;
//	_passosd_data_buffer.data.climbrate = pt_data_ptr->data.climbrate;
//	_passosd_data_buffer.data.altitude = pt_data_ptr->data.altitude;
//	_passosd_data_buffer.data.flight_time_sec = pt_data_ptr->data.flight_time_sec;
//	_passosd_data_buffer.data.yaw = pt_data_ptr->data.yaw;

	memcpy(_passosd_data_buffer.bytes+11, pt_data_ptr->bytes+11, 31);
	
/*	
	printf("%4.2f\n", _passosd_data_buffer.data.voltage_battery);
	printf("%4.2f\n", _passosd_data_buffer.data.current_total_mah);
	printf("%d\n", _passosd_data_buffer.data.fix_type);
	printf("%d\n", _passosd_data_buffer.data.num_sats);
	printf("%d\n", _passosd_data_buffer.data.control_mode);
	printf("%4.2f\n", _passosd_data_buffer.data.velocity);
	printf("%4.2f\n", _passosd_data_buffer.data.climbrate);
	printf("%4.2f\n", _passosd_data_buffer.data.altitude);
	printf("%d\n", _passosd_data_buffer.data.flight_time_sec);
	printf("%4.2f\n", _passosd_data_buffer.data.yaw);

	printf("%d\n", _passosd_data_buffer.data.crc0);
	printf("%d\n", _passosd_data_buffer.data.crc1);
*/	

//	--------------------------------------------------
//	crc
//	--------------------------------------------------
	local_crc = crc_calculate(_passosd_data_buffer.bytes, sizeof(struct PassOSD_data_struct));

	_passosd_data_buffer.data.crc0 = local_crc;
	_passosd_data_buffer.data.crc1 = local_crc >> 8;

	_passosd_data_buffer.data.etx0 = 0x89;
	_passosd_data_buffer.data.etx1 = 0x41;

	

//	--------------------------------------------------
//	send
//	--------------------------------------------------
//	printf("\n%d: ", lcl_cnt++);

	for (local_uint32 = 0;  local_uint32 != sizeof(struct PassOSD_data_struct) ; local_uint32++) 
    {
		_port->write(_passosd_data_buffer.bytes[local_uint32]);

//		printf("%d %.2x \n", local_uint32, _passosd_data_buffer.bytes[local_uint32]);
	}

//	printf("Succeed!\n");

	
}

#endif

