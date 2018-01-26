/*
 * AP_UAVCAN.cpp
 *
 *      Author: Eugene Shamaev
 */

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>


#include "AP_CanTest.h"
#include <GCS_MAVLink/GCS.h>
#include "./../ArduCopter/Copter.h"


#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>



// Zubax GPS and other GPS, baro, magnetic sensors
#include <uavcan/equipment/gnss/Fix.hpp>
#include <uavcan/equipment/gnss/Auxiliary.hpp>
#include <uavcan/equipment/ahrs/MagneticFieldStrength.hpp>
#include <uavcan/equipment/air_data/StaticPressure.hpp>
#include <uavcan/equipment/air_data/StaticTemperature.hpp>
#include <uavcan/equipment/actuator/ArrayCommand.hpp>
#include <uavcan/equipment/actuator/Command.hpp>
#include <uavcan/equipment/actuator/Status.hpp>
#include <uavcan/equipment/esc/RawCommand.hpp>

extern const AP_HAL::HAL& hal;
//const AP_HAL::HAL &hal = AP_HAL::get_HAL();

#define debug_uavcan(level, fmt, args...) do { if ((level) <= AP_BoardConfig_CAN::get_can_debug()) { hal.console->printf(fmt, ##args); }} while (0)

// Translation of all messages from UAVCAN structures into AP structures is done
// in AP_UAVCAN and not in corresponding drivers.
// The overhead of including definitions of DSDL is very high and it is best to
// concentrate in one place.

// TODO: temperature can come not only from baro. There should be separation on node ID
// to check where it belongs to. If it is not baro that is the source, separate layer
// of listeners/nodes should be added.

// table of user settable CAN bus parameters
const AP_Param::GroupInfo AP_CanTest::var_info[] = {

    AP_GROUPEND
};


AP_CanTest::AP_CanTest()
{
	_initialized = false;   //test
}

AP_CanTest::~AP_CanTest()
{
}

void AP_CanTest::init()
{
	_initialized = true;
	//_parent_can_mgr = new PX4::PX4CANManager;
	//_parent_can_mgr->begin(1000000, 0);
	//_parent_can_mgr = static_cast<PX4::PX4CANManager*>(const_cast <AP_HAL::HAL&> (hal).can_mgr[0]);
	_parent_can_mgr = (PX4CANManager *)hal.can_mgr[0];
	timer = AP_HAL::millis();

	if(_parent_can_mgr != NULL)
	{
		_initialized = true;
		printf("init time: %d\n",timer);
		printf("CAN TEST init success\n");
	}
	else
	{
		_initialized = false;
		printf("CAN TEST init false\n");
	}
}


void AP_CanTest :: send_msg(uint8_t id_mode,uint8_t frame_mode,uint8_t data_length)
{

	uavcan::CanFrame frame;
	
    if (id_mode == 0) {
        frame.id = uavcan::CanFrame::MaskStdID & 0x12;
    } else {
        frame.id = uavcan::CanFrame::MaskExtID & 0x12;
        frame.id |= uavcan::CanFrame::FlagEFF;
    }

    if (frame_mode != 0) {
        frame.id |= uavcan::CanFrame::FlagRTR;
    }

    frame.dlc = data_length & 15;

    frame.data[0] = uint8_t(0xFF & 0);
    frame.data[1] = uint8_t(0xFF & 1);
    frame.data[2] = uint8_t(0xFF & 2);
    frame.data[3] = uint8_t(0xFF & 3);
    frame.data[4] = uint8_t(0xFF & 4);
    frame.data[5] = uint8_t(0xFF & 5);
    frame.data[6] = uint8_t(0xFF & 6);
    frame.data[7] = uint8_t(0xFF & 7);
	
	printf("Send status %d\n",_parent_can_mgr->getIface(0)->send_rf(frame, uavcan::MonotonicTime::fromUSec(1), uavcan::CanIOFlagAbortOnError));
	printf("Send Date: OK!\n");
	printf("tx pending %d\n",_parent_can_mgr->getIface(0)->tx_pending());
}

void AP_CanTest ::handle_msg()
{
	int32_t num;
	num = _parent_can_mgr->getIface(0)->available();

	//printf("Receive Data amount is %d\n",num);
	
	if(num <= 0)
		return;
	
	//printf("current time: %d ,time interval: %d\n",(AP_HAL::millis()),(AP_HAL::millis()-timer));
	uavcan::CanFrame out_frame;
	uavcan::MonotonicTime out_ts_monotonic;
    uavcan::UtcTime out_ts_utc;
	uavcan::CanIOFlags out_flags;
	
	for(int i = 0;i<num;i++)
	{
		_parent_can_mgr->getIface(0)->receive_rf(out_frame,out_ts_monotonic,out_ts_utc,out_flags);
		printf("ExtID %X\n",out_frame.id);
		for(int j = 0;j<out_frame.dlc;j++)
		{
			printf("%X \n",out_frame.data[j]);
		
		}
		//printf("****************\n");
	}
	printf("Receive: OK!\n");
	timer = AP_HAL::millis();
}

void AP_CanTest ::update()
{
	if(!_initialized)
		return;	

	if(copter.g.can_test_rt)
		handle_msg();
	else
		send_msg(1,0,8);
}

