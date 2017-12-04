/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include "AC_BCBMonitor.h"

extern const AP_HAL::HAL& hal;



// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AC_BCBMonitor::AC_BCBMonitor(void) :
    _voltage(0),
    _current_amps(0),
    _current_total_mah(0),
    //	added by ZhangYong 20150601
	_power_w(0),
	//	added end
    _last_time_micros(0)
{

	_port = NULL;

	_initialised = 0;
	
	_last_time_micros = 0;

	test_var = 0;
	
	_bcbmonitor_status_timeout = 0;
	_bcbmonitor_status_timeout_cnt = 0;
	_bcbmonitor_status_lasttime = 0;
	_bcbmonitor_status_counter = 0;
	_bcbmonitor_status_timeout_switch = false;

//	_capacity_consumed_bd = 0;
	_capacity_consumed = 0;
	_power_w = 0;
	_power_mw = 0;

	memset(_bcbmonitor_buffer.bytes, 0, sizeof(struct BCBMonitor_status_struct));

	_bcbmonitor_batt_status_ptr = &_bcbmonitor_batt_status;
	memset(_bcbmonitor_batt_status_ptr, 0, sizeof(struct BCBMonitor_batt_status_struct));
								
	_bcbmonitor_sttc_err_ptr = &_bcbmonitor_sttc_err;
	memset(_bcbmonitor_sttc_err_ptr, 0, sizeof(struct BCBMonitor_statistics_err_struct));


	_step = 0;
	_status_passed = false;
	
	
}
	

// init - setup the battery and voltage pins
void
AC_BCBMonitor::init(const AP_SerialManager& serial_manager)
{

	test_var = 0;

	_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_BCBMonitor, 0);

	if (_port != nullptr) 
	{
		//		printf("f\n");
				//	added by zhangyong 20160219
				//printf("MNT503 initialize port\n");
				//	added end
    	_initialised = 1;

		memset(_bcbmonitor_buffer.bytes, 0, sizeof(struct BCBMonitor_status_struct));

		_bcbmonitor_batt_status_ptr = &_bcbmonitor_batt_status;
		memset(_bcbmonitor_batt_status_ptr, 0, sizeof(struct BCBMonitor_batt_status_struct));

		_bcbmonitor_sttc_err_ptr = &_bcbmonitor_sttc_err;
		memset(_bcbmonitor_sttc_err_ptr, 0, sizeof(struct BCBMonitor_statistics_err_struct));

		_step = 0;
		_status_passed = false;

		_bcbmonitor_status_timeout = 0;
		_bcbmonitor_status_timeout_cnt = 0;
		_bcbmonitor_status_lasttime = 0;
    	_last_time_micros = 0;
   	 	_bcbmonitor_status_counter = 0;
		_bcbmonitor_status_timeout_switch = false;

	}
	else
	{
		//		printf("g\n");
		_initialised = 0;
	}

}

// read - read the voltage and current
void
AC_BCBMonitor::read()
{
	//	added by ZhangYong 20160414
	uint8_t data;
    int16_t numc;
	float temp_float;

	//	added end

    // read current
	//	shielded by ZhangYong for debug purpose 20160509

	if(false == _initialised)
		return;

	
			//	added reading_incoming
	numc = _port->available();
		
	
	if(true == _bcbmonitor_status_timeout_switch)
	{
		if(numc <= 0)
		{
			_bcbmonitor_status_timeout++;
		}
		else
		{
			_bcbmonitor_status_timeout = 0;
		}

		if(_bcbmonitor_status_timeout > AP_BCBMONITOR_STATUS_TIMEOUT)
		{
			_bcbmonitor_status_timeout_cnt++;
		}
		else
		{
		}
	}

	//printf("%d\n", numc);


	if (numc <= 0 )
	{
   		return;
	}

    for (int16_t i = 0; i < numc; i++) 
	{        // Process bytes received
   		data = _port->read();

//		printf("0x%x\n", data);
		
        parse_bcbmonitor_status(data);

	}
   
}






uint32_t AC_BCBMonitor::bcbmonitor_status_lasttime()
{
	if (!_initialised) {
        return 0;
    }

	return _bcbmonitor_status_lasttime;
}


uint16_t AC_BCBMonitor::voltageA() 
{ 
	if(battery_is_OK())
	{
		return (float)_bcbmonitor_batt_status_ptr->voltage_A;
	}
	
	return 0;
}


uint16_t AC_BCBMonitor::voltageB() 
{ 
	if(battery_is_OK())
	{
		return (float)_bcbmonitor_batt_status_ptr->voltage_B;
	}
	
	return 0;
}


float AC_BCBMonitor::current_amps() 
{
	
	return (float)_bcbmonitor_batt_status_ptr->current_A;
	
}



bool AC_BCBMonitor::battery_is_OK()
{
	if(false == _initialised)
		return false;


	if(true == _bcbmonitor_status_timeout_switch)
	{
		if((AP_HAL::millis() - _bcbmonitor_status_lasttime) < AP_BCBMONITOR_STATUS_TIMEOUT_MICROS)
			return true;
	}


	return false;
}




uint8_t AC_BCBMonitor::crc_calculate_bcbmonitor(uint8_t* pBuffer, uint8_t length)
{
	uint8_t lcl_cnt;

	uint8_t crc;

	crc = * pBuffer;

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

	return crc;

//	printf("last crc 0x%x\n", crc_ptr[lcl_cnt++]);
}



void AC_BCBMonitor::parse_bcbmonitor_status(uint8_t data) {
    uint8_t crc;
	uint8_t crc_bm503;
    bool crc_ok;
	uint8_t tmp_uint8;
	uint16_t tmp_uint16;
	float past_second;
	uint32_t tmp_uint32;

	if(false == _initialised)
		return;

//	printf("%d\n", _step);

	
	switch(_step)
	{
		case 0:
//			printf("%d. 0x%x\n", _step, data);
			if(BCBMONITOR_STATUS_HEADA == data)
			{
				_step++;
				_bcbmonitor_buffer.bytes[_bcbmonitor_status_counter++] = data;
				
			}
			else
			{
				_step = 0;
				_bcbmonitor_status_counter = 0;
				memset(_bcbmonitor_buffer.bytes, 0, sizeof(struct BCBMonitor_status_struct));
				
				_bcbmonitor_sttc_err_ptr->bcbmonitor_err_cnt++;
				crc_ok = false;
//				printf("ERROR! BCBMONITOR_STATUS_HEADA\n");
			}

			_status_passed = false;
				
			break;

		case 1:
//			printf("%d. 0x%x\n", _step, data);
			if(BCBMONITOR_STATUS_HEADB == data)
			{
				//printf("B\n");
				_step++;
				_bcbmonitor_buffer.bytes[_bcbmonitor_status_counter++] = data;
			}
			else
			{
				_step = 0;
				_bcbmonitor_status_counter = 0;
				memset(_bcbmonitor_buffer.bytes, 0, sizeof(struct BCBMonitor_status_struct));
				_status_passed = false;
				_bcbmonitor_sttc_err_ptr->bcbmonitor_err_cnt++;
				crc_ok = false;
//				printf("ERROR! BCBMONITOR_STATUS_HEADB\n");
			}
			break;

		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
			
		case 7:
		case 8:
		case 9:
		case 10:
		case 11:
			
		case 12:
		case 13:
		case 14:
		case 15:
		case 16:
			
			_step++;
			_bcbmonitor_buffer.bytes[_bcbmonitor_status_counter++] = data;
			break;

		case 17:
			if(BCBMONITOR_STATUS_ENDA == data)
			{
				_step++;
				_bcbmonitor_buffer.bytes[_bcbmonitor_status_counter++] = data;
			}
			else
			{
				_step = 0;
				_bcbmonitor_status_counter = 0;
				memset(_bcbmonitor_buffer.bytes, 0, sizeof(struct BCBMonitor_status_struct));
				_status_passed = false;
				_bcbmonitor_sttc_err_ptr->bcbmonitor_err_cnt++;
				crc_ok = false;
//printf("ERROR! BCBMONITOR_STATUS_ENDA\n");
			}
			break;

		case 18:
			
			if(BCBMONITOR_STATUS_ENDB != data)
			{
//				printf("ERROR! BCBMONITOR_STATUS_ENDB\n");
				_step = 0;
				_bcbmonitor_status_counter = 0;
				memset(_bcbmonitor_buffer.bytes, 0, sizeof(struct BCBMonitor_status_struct));
				_status_passed = false;
				_bcbmonitor_sttc_err_ptr->bcbmonitor_err_cnt++;
				crc_ok = false;
				break;
			}
			
			crc = crc_calculate_bcbmonitor(&_bcbmonitor_buffer.bytes[2], sizeof(struct BCBMonitor_status_struct) - 5);
			crc_bm503 = _bcbmonitor_buffer.data.crc;

			if(crc_bm503 == crc)
				crc_ok = true;
			else
				crc_ok = false;
			
	
			if(crc_ok)
			{
				_status_passed = true;
				_bcbmonitor_buffer.bytes[_bcbmonitor_status_counter++] = data;

				if(false == _bcbmonitor_status_timeout_switch)
				{
					_bcbmonitor_status_timeout_switch = true;
					
				}

				///	convert RMS 2 APM
				memset(_bcbmonitor_batt_status_ptr, 0, sizeof(struct BCBMonitor_batt_status_struct));

				tmp_uint16 = _bcbmonitor_buffer.data.voltage_AH;
				tmp_uint16 = tmp_uint16 << 8;
				tmp_uint16 += _bcbmonitor_buffer.data.voltage_AL;
				_bcbmonitor_batt_status_ptr->voltage_A = tmp_uint16;
				
				
				tmp_uint16 = _bcbmonitor_buffer.data.current_AH;
				tmp_uint16 = tmp_uint16 << 8;
				tmp_uint16 += _bcbmonitor_buffer.data.current_AL;
				_bcbmonitor_batt_status_ptr->current_A = tmp_uint16;
				

				tmp_uint16 = _bcbmonitor_buffer.data.temp_AH;
				tmp_uint16 = tmp_uint16 << 8;
				tmp_uint16 += _bcbmonitor_buffer.data.temp_AL;
				_bcbmonitor_batt_status_ptr->temp_A = tmp_uint16;
				

				tmp_uint16 = _bcbmonitor_buffer.data.voltage_BH;
				tmp_uint16 = tmp_uint16 << 8;
				tmp_uint16 += _bcbmonitor_buffer.data.voltage_BL;
				_bcbmonitor_batt_status_ptr->voltage_B = tmp_uint16;


				tmp_uint16 = _bcbmonitor_buffer.data.current_BH;
				tmp_uint16 = tmp_uint16 << 8;
				tmp_uint16 += _bcbmonitor_buffer.data.current_BL;
				_bcbmonitor_batt_status_ptr->current_B = tmp_uint16;

				tmp_uint16 = _bcbmonitor_buffer.data.temp_BH;
				tmp_uint16 = tmp_uint16 << 8;
				tmp_uint16 += _bcbmonitor_buffer.data.temp_BL;
				_bcbmonitor_batt_status_ptr->temp_B = tmp_uint16;
		
//-----------------------------------------------------------

				_power_mw = (_bcbmonitor_batt_status_ptr->voltage_A) * (_bcbmonitor_batt_status_ptr->current_A);

				_power_w = _power_mw / 1000000;

				if(0 != _bcbmonitor_status_lasttime)
				{
					past_second = (float)(AP_HAL::millis() - _bcbmonitor_status_lasttime);
					past_second = past_second / 1000;
					_capacity_consumed += (uint32_t)(_power_w * past_second);
				}

				_bcbmonitor_status_lasttime = AP_HAL::millis();

	//			printf("A: %d, B: %d\n", (_bcbmonitor_batt_status_ptr->voltage_A / 100), (_bcbmonitor_batt_status_ptr->voltage_B / 100));

	//			printf("CRC ok\n");
		
			}
			else
			{
	//			printf("CRC ERROR\n");
			
				
				_bcbmonitor_sttc_err_ptr->bcbmonitor_err_cnt++;
			}

		
			_status_passed = false;
			memset(_bcbmonitor_buffer.bytes, 0, sizeof(struct BCBMonitor_status_struct));
			_bcbmonitor_status_counter = 0;
			_step = 0;
			crc_ok = false;
			
			break;

		default:
			_step = 0;
			_bcbmonitor_status_counter = 0;
			memset(_bcbmonitor_buffer.bytes, 0, sizeof(struct BCBMonitor_status_struct));
			_status_passed = false;
			_bcbmonitor_sttc_err_ptr->bcbmonitor_err_cnt++;
			break;
					
	}

	
}



