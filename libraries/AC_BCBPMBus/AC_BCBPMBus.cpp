/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include "AC_BCBPMBus.h"

extern const AP_HAL::HAL& hal;


/*
AP_Int16 _fs_vin;
	AP_Int16 _fs_vout;
	AP_Int16 _fs_temp;

*/



// table of user settable parameters
const AP_Param::GroupInfo AC_BCBPMBus::var_info[] = {

	// @Param: _ENABLE
    // @DisplayName: PMBUS_ENABLE
    // @Description: determing whether this module should be enabled 
    // @Values: 0:Disable,1:Enable
    // @User: Advanced
    AP_GROUPINFO("_ENABLE",    1, AC_BCBPMBus, _enable, 0),
	

    // @Param: _FS_VIN
    // @DisplayName: PMBUS_FC_VIN
    // @Description: failsafe threshold, when vin below this value, triger failsafe
    // @Values: 
    // @User: Advanced
    AP_GROUPINFO("_FS_VIN",    2, AC_BCBPMBus, _fs_vin, 352),

	// @Param: _FS_VOUT
    // @DisplayName: PMBUS_FS_VOUT
    // @Description: failsafe threshold, when vout below this value, triger failsafe
    // @Values:
    // @User: Advanced
    AP_GROUPINFO("_FS_VOUT",    3, AC_BCBPMBus, _fs_vout, 44),

	// @Param: _FS_TEMP
    // @DisplayName: PMBUS_FS_TEMP
    // @Description: failsafe threshold, when vin above this value, triger failsafe
    // @Values: 
    // @User: Advanced
    AP_GROUPINFO("_FS_TEMP",    4, AC_BCBPMBus, _fs_temp, 95),

	// @Param: _FS_TEMP
    // @DisplayName: PMBUS_FS_TEMP
    // @Description: failsafe threshold, when vin above this value, triger failsafe
    // @Values: 
    // @User: Advanced
    AP_GROUPINFO("_FS_IOUT",    5, AC_BCBPMBus, _fs_iout, 2),


	// @Param: _FS_THREHOLD
    // @DisplayName: _FS_THD
    // @Description: failsafe threshold, when vin, vout, temp, iout above shtrshold continuially, triger failsafe
    // @Values: 
    // @User: Advanced
    AP_GROUPINFO("_FS_THD",    6, AC_BCBPMBus, _fs_threshold, 3),

	// @Param: _FS_THREHOLD
    // @DisplayName: PMBUS_FS_TEMP
    // @Description: failsafe threshold, when vin above this value, triger failsafe
    // @Values: 
    // @User: Advanced
    AP_GROUPINFO("_FS_ENABLE",    7, AC_BCBPMBus, _fs_enable, 0),

	// @Param: _FS_THR_MIN
    // @DisplayName: PMBUS_FS_THR_MIN
    // @Description: failsafe triger throttle threshold, when monitor the IOUT throttle should hiher than this value to ensure iout not zero
    // @Values: [0, 300]
    // @User: Advanced
    AP_GROUPINFO("_FS_THR_MIN",    8, AC_BCBPMBus, _fs_iout_thr_min, 0),


     AP_GROUPEND
};


// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AC_BCBPMBus::AC_BCBPMBus(AP_SerialManager& _serial_manager) :
	serial_manager(_serial_manager),
    _last_time_micros(0)
{
	AP_Param::setup_object_defaults(this, var_info);

	_port = NULL;

	_initialised = 0;
	
	_last_time_micros = 0;

	test_var = 0;
	
	_bcbpmbus_status_timeout = 0;
	_bcbpmbus_status_timeout_cnt = 0;
	_bcbpmbus_status_lasttime = 0;
	_bcbpmbus_status_timeout_switch = false;

//	_capacity_consumed_bd = 0;
	

	memset(_bcbpmbus_buffer.bytes, 0, sizeof(struct BCBPMBus_frame_struct));

	
	memset(_bcbpmbus_status, 0, (AC_BCBPMBUS_MODULE_MAX_COMPONENT * sizeof(struct BCBPMBus_modules_struct)));
								
	_bcbpmbus_sttc_err_ptr = &_bcbpmbus_sttc_err;
	memset(_bcbpmbus_sttc_err_ptr, 0, sizeof(struct BCBPMBus_statistics_err_struct));


//	_bcbpmbus_component_info_ptr = &_bcbpmbus_component_info;
//	memset(_bcbpmbus_component_info_ptr, 0, sizeof(struct BCBPMBus_component_info_struct));
	


	_step = 0;
	_status_passed = false;

//	printf("AC_BCBPMBus AC_BCBPMBus over\n");
	
}

// init - setup the battery and voltage pins
void
AC_BCBPMBus::init()
{

	if(!(_enable.get()))
		return;


	test_var = 0;

	_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_BCBPMBus, 0);

	if (_port != nullptr) 
	{
		//		printf("f\n");
				//	added by zhangyong 20160219
		
				//	added end
    	_initialised = 1;

		memset(_bcbpmbus_buffer.bytes, 0, sizeof(struct BCBPMBus_frame_struct));

		memset(_bcbpmbus_status, 0, (AC_BCBPMBUS_MODULE_MAX_COMPONENT * sizeof(struct BCBPMBus_modules_struct)));

		_bcbpmbus_sttc_err_ptr = &_bcbpmbus_sttc_err;
		memset(_bcbpmbus_sttc_err_ptr, 0, sizeof(struct BCBPMBus_statistics_err_struct));

		_step = 0;
		_status_passed = false;

		_bcbpmbus_status_timeout = 0;
		_bcbpmbus_status_timeout_cnt = 0;
		_bcbpmbus_status_lasttime = 0;
    	_last_time_micros = 0;
 		_bcbpmbus_status_timeout_switch = false;

	}
	else
	{
		//		printf("g\n");
		_initialised = 0;
	}

//	printf("AC_BCBPMBus::init() %d\n", _initialised);
}

// read - read the voltage and current
void
AC_BCBPMBus::read()
{
	//	added by ZhangYong 20160414
	uint8_t data;
	int16_t numc;

	//	added end

    // read current
	//	shielded by ZhangYong for debug purpose 20160509


//	printf("read %d %d %d\n", _enable, (_enable.get()), !(_enable.get()));

//	printf("_fs_vin %d _fs_vout %d _fs_temp %d\n", _fs_vin, _fs_vout, _fs_temp);

	if(!(_enable.get()))
		return;

	if(false == _initialised)
		return;

	
	

	
			//	added reading_incoming
	numc = _port->available();
		
	/*
	if(true == _bcbpmbus_status_timeout_switch)
	{
		if(numc <= 0)
		{
			_bcbpmbus_status_timeout++;
		}
		else
		{
			_bcbpmbus_status_timeout = 0;
		}

		if(_bcbpmbus_status_timeout > AP_BCBPMBUS_STATUS_TIMEOUT)
		{
			_bcbpmbus_status_timeout_cnt++;
		}
		else
		{
		}
	}*/

	//printf("%d\n", numc);


	if (numc <= 0 )
	{
   		return;
	}

    for (int16_t i = 0; i < numc; i++) 
	{        // Process bytes received
   		data = _port->read();

//		printf("0x%x\n", data);
		
		parse_bcbpmbus_status(data);

	}
   
}






uint32_t AC_BCBPMBus::bcbpmbus_status_lasttime()
{
	if (!_initialised) {
        return 0;
    }

	return _bcbpmbus_status_lasttime;
}

uint32_t AC_BCBPMBus::get_last_read(uint8_t idx)
{
	if(idx >= AC_BCBPMBUS_MODULE_MAX_COMPONENT)
		return 0;

	return _bcbpmbus_status[idx].last_read;
}

uint8_t AC_BCBPMBus::get_component_seq(uint8_t idx)
{
	if(idx >= AC_BCBPMBUS_MODULE_MAX_COMPONENT)
		return 0;

	return _bcbpmbus_status[idx].seq;

/*	if(true == _bcbpmbus_status[idx].new_data)
//	{
		return _bcbpmbus_status[idx].seq;
//	}
//	else
//	{
//		return 0;
//	}
*/
}


uint8_t AC_BCBPMBus::get_component_id(uint8_t idx)
{
	if(idx >= AC_BCBPMBUS_MODULE_MAX_COMPONENT)
		return 0;

	return _bcbpmbus_status[idx].component_id;

/*	if(true == _bcbpmbus_status[idx].new_data)
	{
		return _bcbpmbus_status[idx].component_id;
	}
	else
	{
		return 0;
	}
*/
}


uint8_t AC_BCBPMBus::get_status_byte(uint8_t idx)
{
	if(idx >= AC_BCBPMBUS_MODULE_MAX_COMPONENT)
		return 0;

	return _bcbpmbus_status[idx].status_byte;

/*
	if(true == _bcbpmbus_status[idx].new_data)
	{
		return _bcbpmbus_status[idx].status_byte;
	}
	else
	{
		return 0;
	}
*/}

uint8_t AC_BCBPMBus::get_status_word(uint8_t idx)
{
	if(idx >= AC_BCBPMBUS_MODULE_MAX_COMPONENT)
		return 0;

	return _bcbpmbus_status[idx].status_word;

/*	if(true == _bcbpmbus_status[idx].new_data)
	{
		return _bcbpmbus_status[idx].status_word;
	}
	else
	{
		return 0;
	}
*/}

uint8_t AC_BCBPMBus::get_status_iout(uint8_t idx)
{
	if(idx >= AC_BCBPMBUS_MODULE_MAX_COMPONENT)
		return 0;

	return _bcbpmbus_status[idx].status_iout;

/*	if(true == _bcbpmbus_status[idx].new_data)
	{
		return _bcbpmbus_status[idx].status_iout;
	}
	else
	{
		return 0;
	}
*/
}

uint8_t AC_BCBPMBus::get_status_input(uint8_t idx)
{
	if(idx >= AC_BCBPMBUS_MODULE_MAX_COMPONENT)
		return 0;


	return _bcbpmbus_status[idx].status_input;

/*	if(true == _bcbpmbus_status[idx].new_data)
	{
		return _bcbpmbus_status[idx].status_input;
	}
	else
	{
		return 0;
	}
*/
}

uint8_t AC_BCBPMBus::get_status_temperature(uint8_t idx)
{
	if(idx >= AC_BCBPMBUS_MODULE_MAX_COMPONENT)
		return 0;

	return _bcbpmbus_status[idx].status_temperature;

/*
	if(true == _bcbpmbus_status[idx].new_data)
	{
		return _bcbpmbus_status[idx].status_temperature;
	}
	else
	{
		return 0;
	}
*/
}

uint8_t AC_BCBPMBus::get_status_cml(uint8_t idx)
{
	if(idx >= AC_BCBPMBUS_MODULE_MAX_COMPONENT)
		return 0;


	return _bcbpmbus_status[idx].status_cml;

/*
	if(true == _bcbpmbus_status[idx].new_data)
	{
		return _bcbpmbus_status[idx].status_cml;
	}
	else
	{
		return 0;
	}
*/
}


uint16_t AC_BCBPMBus::get_read_vin(uint8_t idx)
{
	if(idx >= AC_BCBPMBUS_MODULE_MAX_COMPONENT)
		return 0;

	return _bcbpmbus_status[idx].read_vin;

/*
if(true == _bcbpmbus_status[idx].new_data)
	{
		return _bcbpmbus_status[idx].read_vin;
	}
	else
	{
		return 0;
	}
*/
}

	/// Battery voltage.  Initialized to 0
uint16_t AC_BCBPMBus::get_read_iin(uint8_t idx)
{
	if(idx >= AC_BCBPMBUS_MODULE_MAX_COMPONENT)
		return 0;

	return _bcbpmbus_status[idx].read_iin;
/*	if(true == _bcbpmbus_status[idx].new_data)
	{
		return _bcbpmbus_status[idx].read_iin;
	}
	else
	{
		return 0;
	}
*/
}

uint16_t AC_BCBPMBus::get_read_vout(uint8_t idx)
{
	if(idx >= AC_BCBPMBUS_MODULE_MAX_COMPONENT)
		return 0;


	return _bcbpmbus_status[idx].read_vout;

/*	if(true == _bcbpmbus_status[idx].new_data)
	{
		return _bcbpmbus_status[idx].read_vout;
	}
	else
	{
		return 0;
	}
*/
}

uint16_t AC_BCBPMBus::get_read_iout(uint8_t idx)
{
	if(idx >= AC_BCBPMBUS_MODULE_MAX_COMPONENT)
		return 0;

	return _bcbpmbus_status[idx].read_iout;

/*
	if(true == _bcbpmbus_status[idx].new_data)
	{
		return _bcbpmbus_status[idx].read_iout;
	}
	else
	{
		return 0;
	}
*/
}

uint16_t AC_BCBPMBus::get_read_temperature(uint8_t idx)
{
	if(idx >= AC_BCBPMBUS_MODULE_MAX_COMPONENT)
		return 0;

	return _bcbpmbus_status[idx].read_temperature;

/*	if(true == _bcbpmbus_status[idx].new_data)
	{
		return _bcbpmbus_status[idx].read_temperature;
	}
	else
	{
		return 0;
	}
*/
}

uint16_t AC_BCBPMBus::get_read_pout(uint8_t idx)
{
	if(idx >= AC_BCBPMBUS_MODULE_MAX_COMPONENT)
		return 0;

	return _bcbpmbus_status[idx].read_pout;

/*
	if(true == _bcbpmbus_status[idx].new_data)
	{
		return _bcbpmbus_status[idx].read_pout;
	}
	else
	{
		return 0;
	}
*/
}


uint8_t AC_BCBPMBus::pmbus_is_OK()
{
	if(!(_enable.get()))
		return 0;

	if(false == _initialised)
		return false;

	if(true == _bcbpmbus_status_timeout_switch)
	{
		if((AP_HAL::millis() - _bcbpmbus_status_lasttime) >= AC_BCBPMBUS_STATUS_TIMEOUT_MICROS)
			return false;
	}

	return true;
}


//	uint8_t *slot which module is not ok
//	uint8_t *reason
//	0x0: OK
//	0x1: input voltage lower than 352	(middle priority)
//	0x2: output voltage lower than 44 (low priority)
//	0x4: iout lower than 1        	(higher priority)
//	0x8: temprature higher than 80 	(high priority)
//	0x10: timeout						(highest priority)


#define AC_BCBPMBUS_FS_DISABLE_BIT_MASK		0x0		//	0b0000 0000
#define AC_BCBPMBUS_FS_VIN_BIT_MASK			0x1		//	0b0000 0001
#define AC_BCBPMBUS_FS_VOUT_BIT_MASK		0x2		//	0b0000 0010
#define AC_BCBPMBUS_FS_IOUT_BIT_MASK		0x4		//	0b0000 0100
#define AC_BCBPMBUS_FS_TEMP_BIT_MASK		0x8		//	0b0000 1000
#define AC_BCBPMBUS_FS_TIMEOUT_BIT_MASK		0x10	//	0b0001 0000	

uint8_t AC_BCBPMBus::component_is_OK(uint8_t slot, uint8_t *reason)
{
	uint16_t vin;
	uint16_t vout;
	static uint8_t _fs_iout_cnt[AC_BCBPMBUS_MODULE_MAX_COMPONENT] = {0, 0, 0, 0,};
	static uint8_t _fs_temp_cnt[AC_BCBPMBUS_MODULE_MAX_COMPONENT] = {0, 0, 0, 0,};
	static uint8_t _fs_vin_cnt[AC_BCBPMBUS_MODULE_MAX_COMPONENT] = {0, 0, 0, 0,};
	static uint8_t _fs_vout_cnt[AC_BCBPMBUS_MODULE_MAX_COMPONENT] = {0, 0, 0, 0,};

	if(slot >= AC_BCBPMBUS_MODULE_MAX_COMPONENT)
	{
		return false;
	}



	if(_fs_vin.get() < 300)
		_fs_vin.set_and_save(352);

	if(_fs_vin.get() > 400)
		_fs_vin.set_and_save(352);

	if(_fs_vout.get() < 42)
		_fs_vout.set_and_save(44);

	if(_fs_vout.get() > 52)
		_fs_vout.set_and_save(44);

	if(_fs_temp.get() < 25)
		_fs_temp.set_and_save(80);

	if(_fs_temp.get() > 100)
		_fs_temp.set_and_save(80);

	if(_fs_iout.get() < 0)
		_fs_iout.set_and_save(2);

	if(_fs_iout.get() > 50)
		_fs_iout.set_and_save(2);

	if(_fs_threshold.get() < 0)
		_fs_threshold.set_and_save(3);
	
	if(_fs_threshold.get() > 10)
		_fs_threshold.set_and_save(3);

	if(_fs_iout_thr_min.get() >= 200)
		_fs_iout_thr_min.set_and_save(100);

	if(_fs_iout_thr_min.get() < 0)
		_fs_iout_thr_min.set_and_save(100);
	

	vin = _bcbpmbus_status[slot].read_vin;
	vout = _bcbpmbus_status[slot].read_vout;

	vin = vin / 10;
	vout = vout / 10;

	if(true == _bcbpmbus_status_timeout_switch)
	{
		if((AP_HAL::millis() - _bcbpmbus_status[slot].last_read) >= AC_BCBPMBUS_STATUS_TIMEOUT_MICROS)
		{
			*reason = AC_BCBPMBUS_FS_TIMEOUT_BIT_MASK;
			return false;
		}
	}

//	printf("component_is_OK %d %d\n", _bcbpmbus_status[slot].read_iout, _fs_iout.get());
	
//	printf("component_is_OK vin %d vout %d temp %d\n", vin, vout, _bcbpmbus_status[slot].read_temperature);

	if(false == _bcbpmbus_status[slot].new_data)
	{
		return true;
	}

	_bcbpmbus_status[slot].new_data = false;


	if(_bcbpmbus_status[slot].read_iout < _fs_iout.get())
	{
		_fs_iout_cnt[slot]++;
		
		if(_fs_iout_cnt[slot] >= _fs_threshold.get())
		{
/*			printf("%d [%d] %d %d %d %d false\n", AP_HAL::millis(), \
											slot, \
											_fs_iout_cnt[slot], \
											_fs_temp_cnt[slot], \
											_fs_vin_cnt[slot], \
											_fs_vout_cnt[slot]);
*/		

			


			*reason = AC_BCBPMBUS_FS_IOUT_BIT_MASK;
			_fs_iout_cnt[slot] = 0;
			return false;
		}
		else
		{
/*			printf("%d [%d] %d %d %d %d true\n", AP_HAL::millis(), \
											slot, \
											_fs_iout_cnt[slot], \
											_fs_temp_cnt[slot], \
											_fs_vin_cnt[slot], \
											_fs_vout_cnt[slot]);
*/		}
	}
	else
	{
		_fs_iout_cnt[slot] = 0;
	}


	if(_bcbpmbus_status[slot].read_temperature > _fs_temp.get())
	{
		_fs_temp_cnt[slot]++;

		if(_fs_temp_cnt[slot] >= _fs_threshold.get())
		{
			

		
			*reason = AC_BCBPMBUS_FS_TEMP_BIT_MASK;
			_fs_temp_cnt[slot] = 0;
			return false;
		}
	}
	else
	{
		_fs_temp_cnt[slot] = 0;
	}
	
	if(vin < _fs_vin.get())
	{
		_fs_vin_cnt[slot]++;
	
		if(_fs_vin_cnt[slot] >= _fs_threshold.get())
		{
		
			*reason = AC_BCBPMBUS_FS_VIN_BIT_MASK;
			_fs_vin_cnt[slot] = 0;
			return false;
		}
	}
	else
	{
		_fs_vin_cnt[slot] = 0;
	}

	if(vout < _fs_vout.get())
	{
		_fs_vout_cnt[slot]++;
	
		if(_fs_vout_cnt[slot] >= _fs_threshold.get())
		{
			*reason = AC_BCBPMBUS_FS_VOUT_BIT_MASK;
			_fs_vout_cnt[slot] = 0;
			return false;
		}
	}
	else
	{
		_fs_vout_cnt[slot] = 0;
	}

/*	printf("[%d] iout %d temp %d vin %d vout %d\n", slot, \
													_fs_iout_cnt[slot], \
													_fs_temp_cnt[slot], \
													_fs_vin_cnt[slot], \
													_fs_vout_cnt[slot]);
*/
	return true;

}

//	uint8_t *slot which module is not ok
//	uint8_t *reason
//	0x0: OK
//	0x1: input voltage lower than 352	(middle priority)
//	0x2: output voltage lower than 44 (low priority)
//	0x4: iout lower than 1        	(higher priority)
//	0x8: temprature higher than 80 	(high priority)
//	0x10: timeout						(highest priority)



uint8_t AC_BCBPMBus::components_is_OK(uint8_t *slot, uint8_t *reason)
{
	uint8_t lcl_cnt;

	for(lcl_cnt = 0; lcl_cnt < AC_BCBPMBUS_MODULE_MAX_COMPONENT; lcl_cnt++)
	{
		if(true == _bcbpmbus_status[lcl_cnt].occupied)
		{
			if(false == component_is_OK(lcl_cnt, reason))
			{
				*slot = lcl_cnt;
				return false;
				
			}
		}
	}

	*slot = 0;
	*reason = 0;
	return true;
}


uint8_t AC_BCBPMBus::crc_calculate_bcbpmbus(uint8_t* pBuffer, uint8_t length)
{
	uint8_t lcl_cnt;

	uint8_t crc;

	crc = * pBuffer;

	for(lcl_cnt = 1; lcl_cnt < length; lcl_cnt++)
	{
		if(lcl_cnt == AC_BCBPMBUS_FRAME_CRC_OFFSET)
			continue;
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


uint8_t AC_BCBPMBus::log_id_2_component_id(uint8_t log_id)
{
	uint8_t component_id;

	switch(log_id)
	{
		case LOG_PMBUS0_MSG:
		case LOG_PMBUS1_MSG:
		case LOG_PMBUS2_MSG:
		case LOG_PMBUS3_MSG:
			component_id = (log_id-LOG_PMBUS0_MSG);
			break;
		default:
			component_id = 0;
			break;
	}


	return component_id;
}

uint8_t AC_BCBPMBus::component_id_2_log_id(uint8_t component_id)
{
	switch(component_id)
	{
		case LOG_PMBUS0_MSG:
		case LOG_PMBUS1_MSG:
		case LOG_PMBUS2_MSG:
		case LOG_PMBUS3_MSG:
			return component_id+LOG_PMBUS0_MSG;
		default:
			break;
	}

	return 0;
}
/*


//bool AC_BCBPMBus::get_component_slot_info(uint8_t index)
{

	if(NULL == _bcbpmbus_component_info_ptr)
	{
		*ture_false = false;
		return NULL;
	}

	if(idx >= AP_BCBPMBUS_MODULE_MAX_COMPONENT)
	{
		*ture_false = false;
		return NULL;
	}
	
	if(false == _bcbpmbus_status[idx].occupied)
	{
		*ture_false = false;
		return NULL;
	}


	_bcbpmbus_component_info_ptr->component_id = _bcbpmbus_status[idx].component_id;
	_bcbpmbus_component_info_ptr->status_byte = _bcbpmbus_status[idx].status_byte;
	_bcbpmbus_component_info_ptr->status_word = _bcbpmbus_status[idx].status_word;
	_bcbpmbus_component_info_ptr->status_iout = _bcbpmbus_status[idx].status_iout;
	_bcbpmbus_component_info_ptr->status_input = _bcbpmbus_status[idx].status_input;
	_bcbpmbus_component_info_ptr->status_temperature = _bcbpmbus_status[idx].status_temperature;
	_bcbpmbus_component_info_ptr->status_cml = _bcbpmbus_status[idx].status_cml;

	_bcbpmbus_component_info_ptr->read_vin = _bcbpmbus_status[idx].read_vin;
	_bcbpmbus_component_info_ptr->read_iin = _bcbpmbus_status[idx].read_iin;
	_bcbpmbus_component_info_ptr->read_vout = _bcbpmbus_status[idx].read_vout;
	_bcbpmbus_component_info_ptr->read_iout = _bcbpmbus_status[idx].read_iout;
	_bcbpmbus_component_info_ptr->read_temperature = _bcbpmbus_status[idx].read_temperature;
	_bcbpmbus_component_info_ptr->read_pout = _bcbpmbus_status[idx].read_pout;

//	*ture_false = true;
//	return _bcbpmbus_component_info_ptr

	return true;
}

*/


void AC_BCBPMBus::parse_bcbpmbus_status(uint8_t data) {
    uint8_t crc;
	uint8_t crc_frame;
    bool crc_ok;
	uint8_t component_slot_idx;


	if(false == _initialised)
		return;

//	printf("%d\n", _step);

	
	
	switch(_step)
	{
		case 0:
//			printf("%d. 0x%x\n", _step, data);
			if(BCBPMBUS_STATUS_HEADA == data)
			{
				_bcbpmbus_buffer.bytes[_step++] = data;
				
			}
			else
			{
				_step = 0;
				memset(_bcbpmbus_buffer.bytes, 0, sizeof(struct BCBPMBus_frame_struct));
				
				_bcbpmbus_sttc_err_ptr->bcbpmbus_err_cnt++;
				crc_ok = false;
//				printf("ERROR! BCBMONITOR_STATUS_HEADA\n");
			}

			_status_passed = false;
				
			break;

		case 1:
//			printf("%d. 0x%x\n", _step, data);
			if(BCBPMBUS_STATUS_HEADB == data)
			{
				//printf("B\n");
				_bcbpmbus_buffer.bytes[_step++] = data;
			}
			else
			{
				_step = 0;
				memset(_bcbpmbus_buffer.bytes, 0, sizeof(struct BCBPMBus_frame_struct));
				_status_passed = false;
				_bcbpmbus_sttc_err_ptr->bcbpmbus_err_cnt++;
				crc_ok = false;
//				printf("ERROR! BCBMONITOR_STATUS_HEADB\n");
			}
			break;

		case 2:						//	seq
			_bcbpmbus_buffer.bytes[_step++] = data;		
			break;
		case 3:						//	length
			_bcbpmbus_buffer.bytes[_step++] = data;		
			
			if(AC_BCBPMBUS_FRAME_VALIDE_LENGTH != data)
			{
				_step = 0;
				memset(_bcbpmbus_buffer.bytes, 0, sizeof(struct BCBPMBus_frame_struct));
				_status_passed = false;
				_bcbpmbus_sttc_err_ptr->bcbpmbus_err_cnt++;
				crc_ok = false;
			}
			break;
			
		case 4:						//	msg_id
			_bcbpmbus_buffer.bytes[_step++] = data;		
			
			if(1 != data)
			{
				_step = 0;
				memset(_bcbpmbus_buffer.bytes, 0, sizeof(struct BCBPMBus_frame_struct));
				_status_passed = false;
				_bcbpmbus_sttc_err_ptr->bcbpmbus_err_cnt++;
				crc_ok = false;
			}
			break;
		case 5:						//	source_id
			_bcbpmbus_buffer.bytes[_step++] = data;		
			
			if(AC_BCBPMBUS_FRAME_SOURCE_ID != data)
			{
				_step = 0;
				memset(_bcbpmbus_buffer.bytes, 0, sizeof(struct BCBPMBus_frame_struct));
				_status_passed = false;
				_bcbpmbus_sttc_err_ptr->bcbpmbus_err_cnt++;
				crc_ok = false;
			}
			break;
			
		case 6:						//	source_component
			_bcbpmbus_buffer.bytes[_step++] = data;	

			break;
			
		case 7:						//	dest_id
			_bcbpmbus_buffer.bytes[_step++] = data;		
			
			if(AC_BCBPMBUS_FRAME_DEST_ID != data)
			{
				_step = 0;
					memset(_bcbpmbus_buffer.bytes, 0, sizeof(struct BCBPMBus_frame_struct));
				_status_passed = false;
				_bcbpmbus_sttc_err_ptr->bcbpmbus_err_cnt++;
				crc_ok = false;
			}
			break;
		case 8:						//	dest_compomnent
			_bcbpmbus_buffer.bytes[_step++] = data;		
			
			if(AC_BCBPMBUS_FRAME_DEST_COMPONENT != data)
			{
				_step = 0;
				memset(_bcbpmbus_buffer.bytes, 0, sizeof(struct BCBPMBus_frame_struct));
				_status_passed = false;
				_bcbpmbus_sttc_err_ptr->bcbpmbus_err_cnt++;
				crc_ok = false;
			}
			break;
		case 9:						//	status
			_bcbpmbus_buffer.bytes[_step++] = data;		

			if(AC_BCB_PMBUS_FRAME_STATUS_VALIDE != data)
			{
				_step = 0;
				memset(_bcbpmbus_buffer.bytes, 0, sizeof(struct BCBPMBus_frame_struct));
				_status_passed = false;
				_bcbpmbus_sttc_err_ptr->bcbpmbus_err_cnt++;
				crc_ok = false;
			}
			break;
			
		case 10:			//	status_byte
		case 11:			//	status_word
		case 12:			//	status_iout
		case 13:			//	status_input
		case 14:			//	status_temperature
		case 15:			//	status_cml

		case 16:			//	read_vin_h
		case 17:			//	read_vin_l

		case 18:			//	read_iin_h
		case 19:			//	read_iin_l

		case 20:			//	read_vout_h
		case 21:			//	read_vout_l

		case 22:			//	read_iout_h
		case 23:			//	read_iout_l	

		case 24:			//	read_temperatur_1_h
		case 25:			//	read_temperatur_1_l

		case 26:			//	read_pout_h
		case 27:			//	read_pout_l

		case 28:			//	crc
			
			_bcbpmbus_buffer.bytes[_step++] = data;
			break;

		case 29:
			if(BCBPMBUS_STATUS_ENDA == data)
			{
				_bcbpmbus_buffer.bytes[_step++] = data;
			}
			else
			{
				_step = 0;
				memset(_bcbpmbus_buffer.bytes, 0, sizeof(struct BCBPMBus_frame_struct));
				_status_passed = false;
				_bcbpmbus_sttc_err_ptr->bcbpmbus_err_cnt++;
				crc_ok = false;
//printf("ERROR! BCBMONITOR_STATUS_ENDA\n");
			}
			break;

		case 30:
			
			if(BCBPMBUS_STATUS_ENDB != data)
			{
//				printf("ERROR! BCBMONITOR_STATUS_ENDB\n");
				_step = 0;
				memset(_bcbpmbus_buffer.bytes, 0, sizeof(struct BCBPMBus_frame_struct));
				_status_passed = false;
				_bcbpmbus_sttc_err_ptr->bcbpmbus_err_cnt++;
				crc_ok = false;
				break;
			}

			



			_bcbpmbus_buffer.bytes[_step++] = data;

			
			
			crc = crc_calculate_bcbpmbus(_bcbpmbus_buffer.bytes, sizeof(struct BCBPMBus_frame_struct));
			crc_frame = _bcbpmbus_buffer.data.crc;

//			printf("CRC 0x%x vs 0x%x\n", crc, crc_frame);
			

			if(crc_frame == crc)
				crc_ok = true;
			else
				crc_ok = false;

			
			if(crc_ok)
			{
				
			
				_status_passed = true;
//				_bcbpmbus_buffer.bytes[_bcbpmbus_status_counter++] = data;
	

				if(false == _bcbpmbus_status_timeout_switch)
				{
					_bcbpmbus_status_timeout_switch = true;
					
				}

				_bcbpmbus_sttc_err_ptr->bcbpmbus_ok_cnt++;

				
				//	we begin to store the pmbus information to buffers
				//	then write the buffer to log & failsafe & mavlink
				if(true == find_component_slot((_bcbpmbus_buffer.data.source_component), &component_slot_idx))
				{
					//	we have this component before
					//	store data to this slot
/*					printf("parse_bcbpmbus_status found component_id %d at slot %d\n", \
																						_bcbpmbus_buffer.data.source_component, \
																						component_slot_idx);
*/					
					if(false == set_component_slot_data(component_slot_idx))
					{
						printf("parse_bcbpmbus_status set_component_slot_data\n");
					}
					else
					{
/*						printf("parse_bcbpmbus_status push component_id %d to slot %d\n", \
																						_bcbpmbus_buffer.data.source_component, \
																						component_slot_idx);
*/						
					}
				}
				else
				{
/*					printf("parse_bcbpmbus_status component_id %d not found\n", \
																			_bcbpmbus_buffer.data.source_component);
*/					
					if(false == find_available_component_slot(&component_slot_idx))
					{
						//	no more component buffer, report this error to GCS, discards data
						printf("parse_bcbpmbus_status: no more component buffer\n");
						
					}
					else
					{
/*						printf("parse_bcbpmbus_status available slot %d\n", \
																			component_slot_idx);
*/						
						//	double check the slot idx
						if(false == set_component_slot_data(component_slot_idx))
						{
							printf("parse_bcbpmbus_status set_component_slot_data\n");
						}
						else
						{
/*							printf("parse_bcbpmbus_status push component_id %d to slot %d\n", \
																						_bcbpmbus_buffer.data.source_component, \
																						component_slot_idx);
*/						
						}
		

						
						//	we have already got this component set to the corresponding slot
						
					}
				}

//				printf_component_slot_data(component_slot_idx);
//				printf_component_slot_info(component_slot_idx);

				_bcbpmbus_status_lasttime = AP_HAL::millis();

				

				///	convert RMS 2 APM
	//			printf("CRC ok\n");
		
			}
			else
			{
	//			printf("CRC ERROR\n");
			
				
				_bcbpmbus_sttc_err_ptr->bcbpmbus_err_cnt++;
			}

		
			_status_passed = false;
			memset(_bcbpmbus_buffer.bytes, 0, sizeof(struct BCBPMBus_frame_struct));
			_step = 0;
			crc_ok = false;

			//printf("OK %d ERROR %d\n", _bcbpmbus_sttc_err_ptr->bcbpmbus_ok_cnt, _bcbpmbus_sttc_err_ptr->bcbpmbus_err_cnt);
			
			break;

		default:
			_step = 0;
			memset(_bcbpmbus_buffer.bytes, 0, sizeof(struct BCBPMBus_frame_struct));
			_status_passed = false;
			_bcbpmbus_sttc_err_ptr->bcbpmbus_err_cnt++;
			break;
					
	}

	
}


bool AC_BCBPMBus::find_component_slot(uint8_t componnet_id, uint8_t *idx)
{
	uint8_t lcl_cnt;

	for(lcl_cnt = 0; lcl_cnt < AC_BCBPMBUS_MODULE_MAX_COMPONENT; lcl_cnt ++)
	{
		if(false == _bcbpmbus_status[lcl_cnt].occupied)
		{
			continue;
		}
		else
		{
			if(componnet_id == _bcbpmbus_status[lcl_cnt].component_id)
			{
				*idx = lcl_cnt;
				return true;
			}
		}
	}

	return false;
}


bool AC_BCBPMBus::find_available_component_slot(uint8_t *idx)
{
	uint8_t lcl_cnt;

	for(lcl_cnt = 0; lcl_cnt < AC_BCBPMBUS_MODULE_MAX_COMPONENT; lcl_cnt ++)
	{
		if(false == _bcbpmbus_status[lcl_cnt].occupied)
		{
			*idx = lcl_cnt;
			return true;
		}
		else
		{
			continue;
		}
	}

	return false;
}


bool AC_BCBPMBus::set_available_component_slot_occupy(uint8_t idx)
{


	
	if(false == _bcbpmbus_status[idx].occupied)
	{
		_bcbpmbus_status[idx].occupied = true;
		return true;
	}
	else
	{
		printf("set_available_component_slot_occupy slot cnt %d already occupied\n", idx);
	}


	return false;
}




bool AC_BCBPMBus::set_component_slot_data(uint8_t idx)
{
	uint16_t lcl_uint16;

	//	double check the idx
	if(idx >= AC_BCBPMBUS_MODULE_MAX_COMPONENT)
	{
		return false;
	}
	

	if(_bcbpmbus_status[idx].occupied)
	{
		if((_bcbpmbus_status[idx].component_id) != (_bcbpmbus_buffer.data.source_component))
		{
			printf("set_component_slot_data component new id %d old id %d\n", \
										_bcbpmbus_buffer.data.source_component, \
										_bcbpmbus_status[idx].component_id);

			return false;
		}
	}
	else
	{
		set_available_component_slot_occupy(idx);
		_bcbpmbus_status[idx].component_id = _bcbpmbus_buffer.data.source_component;

	}

	_bcbpmbus_status[idx].seq = _bcbpmbus_buffer.data.seq;
	
	_bcbpmbus_status[idx].status_byte = _bcbpmbus_buffer.data.status_byte;
	_bcbpmbus_status[idx].status_word = _bcbpmbus_buffer.data.status_word;
	_bcbpmbus_status[idx].status_iout = _bcbpmbus_buffer.data.status_iout;
	_bcbpmbus_status[idx].status_input = _bcbpmbus_buffer.data.status_input;
	_bcbpmbus_status[idx].status_temperature = _bcbpmbus_buffer.data.status_temperature;
	_bcbpmbus_status[idx].status_cml = _bcbpmbus_buffer.data.status_cml;

	lcl_uint16 = _bcbpmbus_buffer.data.read_vin_h;
	lcl_uint16 = lcl_uint16 << 8;
	lcl_uint16 |= _bcbpmbus_buffer.data.read_vin_l;
	_bcbpmbus_status[idx].read_vin = lcl_uint16;

	lcl_uint16 = _bcbpmbus_buffer.data.read_iin_h;
	lcl_uint16 = lcl_uint16 << 8;
	lcl_uint16 |= _bcbpmbus_buffer.data.read_iin_l;
	_bcbpmbus_status[idx].read_iin = lcl_uint16;

	lcl_uint16 = _bcbpmbus_buffer.data.read_vout_h;
	lcl_uint16 = lcl_uint16 << 8;
	lcl_uint16 |= _bcbpmbus_buffer.data.read_vout_l;
	_bcbpmbus_status[idx].read_vout = lcl_uint16;

	lcl_uint16 = _bcbpmbus_buffer.data.read_iout_h;
	lcl_uint16 = lcl_uint16 << 8;
	lcl_uint16 |= _bcbpmbus_buffer.data.read_iout_l;
	_bcbpmbus_status[idx].read_iout = lcl_uint16;

	lcl_uint16 = _bcbpmbus_buffer.data.read_temperatur_1_h;
	lcl_uint16 = lcl_uint16 << 8;
	lcl_uint16 |= _bcbpmbus_buffer.data.read_temperatur_1_l;
	_bcbpmbus_status[idx].read_temperature = lcl_uint16;

	lcl_uint16 = _bcbpmbus_buffer.data.read_pout_h;
	lcl_uint16 = lcl_uint16 << 8;
	lcl_uint16 |= _bcbpmbus_buffer.data.read_pout_l;
	_bcbpmbus_status[idx].read_pout = lcl_uint16;
		
	_bcbpmbus_status[idx].last_read = AP_HAL::millis();

//	printf("set_component_slot_data [%d] last_read %d\n", idx, _bcbpmbus_status[idx].last_read);

	_bcbpmbus_status[idx].should_log = true;

	_bcbpmbus_status[idx].should_report[0] = true;
	_bcbpmbus_status[idx].should_report[1] = true;

	_bcbpmbus_status[idx].new_data = true;

	
	return true;
}

bool AC_BCBPMBus::printf_component_slot_data(uint8_t idx)
{
	static uint16_t lcl_uint16 = 0;
	

	//	double check the idx
	if(idx >= AC_BCBPMBUS_MODULE_MAX_COMPONENT)
	{
		return false;
	}
	

	if(_bcbpmbus_status[idx].occupied)
	{
		
		printf("%.5d component id 0x%.2x\n", lcl_uint16, _bcbpmbus_status[idx].component_id);
		printf("%.5d status_byte 0x%.2x\n", lcl_uint16, _bcbpmbus_status[idx].status_byte);
		printf("%.5d status_word 0x%.2x\n", lcl_uint16, _bcbpmbus_status[idx].status_word);
		printf("%.5d status_iout 0x%.2x\n", lcl_uint16, _bcbpmbus_status[idx].status_iout);
		printf("%.5d status_input 0x%.2x\n", lcl_uint16, _bcbpmbus_status[idx].status_input);
		printf("%.5d status_temperature 0x%.2x\n", lcl_uint16, _bcbpmbus_status[idx].status_temperature);
		printf("%.5d status_cml 0x%.2x\n", lcl_uint16, _bcbpmbus_status[idx].status_cml);
		
		printf("%.5d read_vin 0x%.4x\n", lcl_uint16, _bcbpmbus_status[idx].read_vin);
		printf("%.5d read_iin 0x%.4x\n", lcl_uint16, _bcbpmbus_status[idx].read_iin);
		printf("%.5d read_vout 0x%.4x\n", lcl_uint16, _bcbpmbus_status[idx].read_vout);
		printf("%.5d read_iout 0x%.4x\n", lcl_uint16, _bcbpmbus_status[idx].read_iout);
		printf("%.5d read_temperature 0x%.4x\n", lcl_uint16, _bcbpmbus_status[idx].read_temperature);
		printf("%.5d read_pout 0x%.4x\n", lcl_uint16, _bcbpmbus_status[idx].read_pout);
	

		lcl_uint16 ++;
	}
	else
	{
		printf("printf_component_slot_data no data %d", idx);
	}

	
	return true;
}


void AC_BCBPMBus::componengt_slot_info_logged(uint8_t idx) 
{ 
	if(!(_enable.get()))
		return;

	if(false == _initialised)
		return;
	
	_bcbpmbus_status[idx].should_log = false; 
};

void AC_BCBPMBus::componengt_slot_info_reported(uint8_t idx, uint8_t chan) 
{ 
	if(!(_enable.get()))
		return;

	if(false == _initialised)
		return;

	if(chan > MAVLINK_COMM_NUM_BUFFERS)
		return;
	
	_bcbpmbus_status[idx].should_report[chan] = false; 
};




bool AC_BCBPMBus::printf_component_slot_info(uint8_t idx)
{
	static uint16_t lcl_uint16 = 0;
	

	//	double check the idx
	if(idx >= AC_BCBPMBUS_MODULE_MAX_COMPONENT)
	{
		return false;
	}
	

	if(_bcbpmbus_status[idx].occupied)
	{
		printf("%.5d component id 0x%.2x\n", lcl_uint16, _bcbpmbus_status[idx].component_id);
		printf("%.5d status_byte 0x%.2x\n", lcl_uint16, _bcbpmbus_status[idx].status_byte);
		printf("%.5d status_word 0x%.2x\n", lcl_uint16, _bcbpmbus_status[idx].status_word);
		printf("%.5d status_iout 0x%.2x\n", lcl_uint16, _bcbpmbus_status[idx].status_iout);
		printf("%.5d status_input 0x%.2x\n", lcl_uint16, _bcbpmbus_status[idx].status_input);
		printf("%.5d status_temperature 0x%.2x\n", lcl_uint16, _bcbpmbus_status[idx].status_temperature);
		printf("%.5d status_cml 0x%.2x\n", lcl_uint16, _bcbpmbus_status[idx].status_cml);
		
		printf("%.5d read_vin %.4d\n", lcl_uint16,(_bcbpmbus_status[idx].read_vin / 10));
		printf("%.5d read_iin %.4d\n", lcl_uint16, (_bcbpmbus_status[idx].read_iin / 1000));
		printf("%.5d read_vout %.4d\n", lcl_uint16, (_bcbpmbus_status[idx].read_vout / 10));
		printf("%.5d read_iout %.4d\n", lcl_uint16, (_bcbpmbus_status[idx].read_iout / 100));
		printf("%.5d read_temperature %.4d\n", lcl_uint16, _bcbpmbus_status[idx].read_temperature);
		printf("%.5d read_pout %.4d\n", lcl_uint16, _bcbpmbus_status[idx].read_pout);
	
		lcl_uint16++;
	}
	else
	{
		printf("printf_component_slot_data no data %d", idx);
	}

	
	return true;
}


bool AC_BCBPMBus::should_log_componengt_slot_info(uint8_t idx)
{
	if(!(_enable.get()))
		return false;

	if(false == _initialised)
		return false;


	if(idx >= AC_BCBPMBUS_MODULE_MAX_COMPONENT)
	{
		return false;
	}

	if(_bcbpmbus_status[idx].occupied)
	{
		if(true == _bcbpmbus_status[idx].should_log)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}
}

bool AC_BCBPMBus::should_report_componengt_slot_info(uint8_t idx, uint8_t chan)
{
	if(!(_enable.get()))
		return false;

	if(false == _initialised)
		return false;


	if(idx >= AC_BCBPMBUS_MODULE_MAX_COMPONENT)
	{
		return false;
	}

	if(chan > MAVLINK_COMM_NUM_BUFFERS)
	{
		return false;
	}

	if(_bcbpmbus_status[idx].occupied)
	{
		if(true == _bcbpmbus_status[idx].should_report[chan])
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}
}


