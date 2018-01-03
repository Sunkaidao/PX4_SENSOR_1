#include <AP_Flowmeter/AP_Flowmeter.h>
//#include <AP_HAL.h>




extern const AP_HAL::HAL& hal;


const AP_Param::GroupInfo AP_Flowmeter::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Sprayer enable/disable
    // @Description: Allows you to enable (1) or disable (0) the flowmeter
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("ENABLE",      0,  AP_Flowmeter, _enabled, 0),

    AP_GROUPEND
};


AP_Flowmeter::AP_Flowmeter()
{
	_initialised = false;
	AP_Param::setup_object_defaults(this, var_info);

//	printf("0.%d", _initialised);
	
}

AP_Flowmeter::~AP_Flowmeter()
{}


void AP_Flowmeter::init(const AP_SerialManager& serial_manager)
{
	uint8_t i;

	if(0 == _enabled)
		return;

	_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_FlowMeter_GKXN, 0);

	if(_port != nullptr)
	{
	
		for(i=0;i<4;i++)
		{
			Tx_Buff[i]=0;
		}
		for(i=0;i<11;i++)
		{
			Rx_Buff[i]=0;
		}
		_Flo_data.high=0;
		_Flo_data.volume=0;
		_Flo_data.warning=0;
		_Flo_data.mode=0;
		_Flo_data.packet_cnt = 0;
  	 	 state_tim=0;
   	 	_num_error.Time_Head_error=0;
   		_num_error.Time_Invalid_data=0;
    	_num_error.Time_Parity_error=0;

		_initialised = true;

		
		
	}

//	printf("1.%d\n", _initialised);
}


void AP_Flowmeter::SendCMD(uint8_t CMD)
{
	uint8_t checkByte=0;
	uint8_t i;
	
	Tx_Buff[0]=0x55;
	Tx_Buff[1]=0x03;
	Tx_Buff[2]=CMD;
	checkByte=0x55+0x03+CMD;
	Tx_Buff[3]=checkByte;
	
	for(i=0;i<4;i++)
    	_port->write(Tx_Buff[i]);
}

int AP_Flowmeter::GetDate()
{
	uint16_t j;
	uint8_t checksum = 0x00;

	if((Rx_Buff[0] != 0x55) && (Rx_Buff[1] != 0x03) && (Rx_Buff[2] != 11))
	{
		_num_error.Time_Head_error++;
		return	Head_error;
	}
	if( Rx_Buff[9] != 0)
	{
		_num_error.Time_Invalid_data++;
		return	Invalid_data;
	}
	for(j=0;j<10; j++)
		checksum += Rx_Buff[j];
	if(checksum != Rx_Buff[10])
	{
	    _num_error.Time_Parity_error++;
		return	Parity_error;
	}
	_Flo_data.high =((Rx_Buff[3]<<8)+Rx_Buff[4])/100.0;
	_Flo_data.volume =((Rx_Buff[5]<<8)+Rx_Buff[6])/100.0;
	_Flo_data.warning = Rx_Buff[7];
	_Flo_data.mode = Rx_Buff[8];

	_Flo_data.packet_cnt ++;

	return Right_data;
}
void AP_Flowmeter::get_Flowmeter_data()
{
	uint16_t i;
	int16_t numc;
	
	//	wite someting to flsuh the buffer of UART
	

	numc = _port->available();

//	printf("%d\n", numc);
	
	if(0 == numc)
		return;

	for (i = 0; i < numc; i++) {        // Process bytes received
        Rx_Buff[i] = _port->read();

//		printf("0x%x\n", Rx_Buff[i]);
	}

	
	if(GetDate()==Right_data)
	{
//		printf("liquid high 	  is %f\n",_Flo_data.high);
//		printf("liquid volume  is %f\n",_Flo_data.volume);
//		printf("liquid warning is %x\n",_Flo_data.warning);
//		printf("liquid mode 	  is %x\n",_Flo_data.mode);
	}
}

void AP_Flowmeter::update(const AP_SerialManager& serial_manager)
{
//	printf("2.%d\n", _initialised);
	if(0 == _enabled)
		return;

	//	to support warm plug
	if(!_initialised)
	{
		init(serial_manager);
	
	}

	if(!_initialised)
		return;

	get_Flowmeter_data();

//	if(state_tim%100==0)
//	{
		//hal.uartD->printf("Have output the CMD!\n");
	SendCMD(Read_Data);
		//hal.uartD->printf("Please input: \n");
//		state_tim=0;
//	}
	
//	state_tim++;
}


uint8_t AP_Flowmeter::get_warning()
{
	if(0 == _enabled)
		return 0;

	if(0 == _initialised)
		return 0;

	
	return _Flo_data.warning;
}


bool AP_Flowmeter::exhausted()
{
	static uint8_t lcl_cnt = 0;;

	if(0 == _enabled)
	{
		return false;
	}

	if(!_initialised)

	{
		return false;
	}

	
	if(1 == _Flo_data.warning)
	{
		lcl_cnt ++;

		if(lcl_cnt > AP_FLOWMETER_EXHAUSTED_SHRESHOLD)
		{
			lcl_cnt = 0;
			return true;
		}
	}
	else if(0 == _Flo_data.warning)
	{
		lcl_cnt = 0;
	}
	else
	{
//		printf("unsupportted value %d\n", _Flo_data.warning);
	}


	return false;
	
}


uint8_t AP_Flowmeter::get_packet_cnt()
{
	if(0 == _enabled)
	{
		return false;
	}

	if(!_initialised)

	{
		return false;
	}
	
	return _Flo_data.packet_cnt;
}


