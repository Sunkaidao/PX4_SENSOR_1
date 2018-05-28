#include <AP_HAL/AP_HAL.h>
#include "AP_Proximity_Radar_GKXN.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Math/crc.h>
#include <ctype.h>
#include <stdio.h>
#include "AP_Proximity_Backend.h"
#include "AP_Proximity.h"
#include "./../ArduCopter/Copter.h"






extern const AP_HAL::HAL& hal;

/*
   The constructor also initialises the proximity sensor. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the proximity sensor
*/
AP_Proximity_Radar_GKXN::AP_Proximity_Radar_GKXN(AP_Proximity &_frontend,
                                                         AP_Proximity::Proximity_State &_state,
                                                         AP_SerialManager &serial_manager) :
    AP_Proximity_Backend(_frontend, _state)
{
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Radar_GKXN, 0);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Radar_GKXN, 0));
    }
}

// detect if a Radar_GKXN proximity sensor is connected by looking for a configured serial port
bool AP_Proximity_Radar_GKXN::detect(AP_SerialManager &serial_manager)
{
    AP_HAL::UARTDriver *uart = nullptr;
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Radar_GKXN, 0);
    return uart != nullptr;
}

// update the state of the sensor
void AP_Proximity_Radar_GKXN::update(void)
{
    if (uart == nullptr) {
        return;
    }

	


	
	//printf("radar_gkxn\n");
	send_sensor_command();
    // process incoming messages
    read_sensor_data();

    // check for timeout and set health status
    if ((_last_distance_received_ms == 0) || (AP_HAL::millis() - _last_distance_received_ms > PROXIMITY_Radar_GKXN_TIMEOUT_MS)) {
        set_status(AP_Proximity::Proximity_NoData);
    } else {
        set_status(AP_Proximity::Proximity_Good);
    }
	/*// added the start delay and set the delay 5000ms
	// check for timeout and set health status
    if (start_flag==1)
    	{
    	_start_GKXN_time=AP_HAL::millis();
		start_flag=0;
		set_status(AP_Proximity::Proximity_Good);
    	}
	else
		{
		if(AP_HAL::millis()-_start_GKXN_time<5000)
			{
			set_status(AP_Proximity::Proximity_Good);
		    }
		else
			{
            if ((_last_distance_received_ms == 0) || (AP_HAL::millis() - _last_distance_received_ms > PROXIMITY_Radar_GKXN_TIMEOUT_MS)) {
            set_status(AP_Proximity::Proximity_NoData);
            } else {
            set_status(AP_Proximity::Proximity_Good);
            }
			}
		}*/
}

// get maximum and minimum distances (in meters) of primary sensor
float AP_Proximity_Radar_GKXN::distance_max() const
{
    return 30.0f;
}
float AP_Proximity_Radar_GKXN::distance_min() const
{
    return 0.60f;
}
bool AP_Proximity_Radar_GKXN::send_sensor_command()
{
	if (uart == nullptr) {
		  return false;
	  }
//	printf("send sensor command\n");
	uint8_t checkByte=0;
	int i,j;
	for(i=0;i<8;i++)
    {
      TX_Buff[i]=0;
	}
	int32_t pitch_radar=0-get_table_angle();
	int16_t pitch_msb=( pitch_radar & 0xFF00)>>8;
	int16_t pitch_lsb=pitch_radar&0x00FF;
//	printf("pitch radar=%u\n",pitch_radar);
	TX_Buff[0]=0x55;
	TX_Buff[1]=0x0C;
	TX_Buff[2]=0x00;
	TX_Buff[3]=0x00;
	
	TX_Buff[4]=get_table_orient();
//	printf("get_table_orient()=%x\n",TX_Buff[4]);
    TX_Buff[5]=pitch_msb;
//    printf("TX_Buff[5]=%x\n",TX_Buff[5]);
	TX_Buff[6]=pitch_lsb;
//	printf("TX_Buff[6]=%x\n",TX_Buff[6]);
	for(j=0;j<7; j++)
	{
		checkByte += TX_Buff[j];
	}
	TX_Buff[7]=checkByte;
	
	
	for(i=0;i<8; i++)
		{
		uart->write(TX_Buff[i]);
//		printf("%x",TX_Buff[i]);
		}
//	printf("\n");
	return true;
	
}

// check for replies from sensor, returns true if at least one message was processed

bool AP_Proximity_Radar_GKXN::read_sensor_data()
{   
   if (uart == nullptr) {
        return false;
    }
    int16_t nbytes = uart->available();
	uint16_t front_data_back=0;
	uint16_t back_data_back=0;
	int      k=0;
	uint16_t checksum = 0x00;
	uint8_t message_state=0;
	int      message_complete=0;
	uint16_t d1=0x00;
	uint16_t d2=0x00;
	uint8_t front_data_valid=0;
	uint8_t back_data_valid=0;
	uint8_t front_data_warning=0;
	uint8_t back_data_warning=0;

   while (nbytes-- > 0)  {
		
       uint8_t data=uart->read();
	   switch(message_state)
	   	{
	   	case 0:
	   	  if(data==0x55)
	   	  	{_num_error.Time_Head_error=0;
		     _num_error.Time_Invalid_data=0;
		     _num_error.Time_Checksum_error=0 ;
			 front_data_back=0;
	         back_data_back=0;
			 checksum=0x00;
			 k=0;
			 front_warning=0;
			 back_warning=0;
			 checksum+=data;
			 front_data_warning=0;
	         back_data_warning=0;
			 message_complete=0;
			 message_state=1;
			 
	   	  	}
		  else
		 	{
		 	_num_error.Time_Head_error++;
		 	message_state=0;
		 	}
		 break;
		case 1:
		  if(data==0x0C)
		  	{
		  	message_state=2;
			checksum+=data;
		  	}
		  else
		 	{
		 	_num_error.Time_Invalid_data++;
		 	message_state=0;
		 	}
		  break;
		case 2:
		   if(data==0x0B)
		 	{
		 	message_state=3;
			checksum+=data;
		 	}
			else
		 	{
		 	_num_error.Time_Invalid_data++;
		 	message_state=0;
		 	}
			break;
		case 3:
			if(data!=0xff)
			{
			front_data_back=(data<<8);
			checksum+=data;
			message_state=4;
			}
			else
			{
			front_data_back=(data<<8);
			checksum+=data;
			k=1;
			message_state=4;
			}
			break;
		case 4:
			if((k==1)&&(data==0xff))
			{
		     k=0;
			 checksum+=data;
			 front_data_back+=data;
			 message_state=5;
			 front_data_valid++;
			}
		    else
			{
		     front_data_back+=data;
			 checksum+=data;
			 k=0;
			 message_state=5;
			}
		break;
		case 5:
		    if(data!=0xff)
		    {
			back_data_back=(data<<8);
			checksum+=data;
		    message_state=6;
		    }
		    else
		    {
		    back_data_back=(data<<8);
		    checksum+=data;
			k=1;
			message_state=6;
			}
		break;
		case 6:
	        if((k==1)&&(data==0xff))
		    {
		    k=0;
			checksum+=data;
			back_data_back+=data;
		    message_state=7;
			back_data_valid++;
		    }
		    else
			{
			back_data_back+=data;
			checksum+=data;
			k=0;
		    message_state=7;
			}
		break;
		case 7:
			checksum+=data;
			message_state=8;
		break;
		case 8:
			checksum+=data;
			message_state=9;
		break;
		case 9:
		    if (((data&0x01)!=0x01)&&(((data>>2)&0x01)!=0x01))
		    {
		    checksum+=data;
		    message_state=10;
		    }
			else if (((data&0x01)==0x01)&&(front_data_valid!=0))
		    {
		     message_state=10;
		     checksum+=data;
			 front_data_warning=1; 
		    }
			else if((((data>>2)&0x01)==0x01)&&(back_data_valid!=0))
			{
		     message_state=10;
		     checksum+=data;
			 back_data_warning=1;
			}
		    else
		    {
		    checksum+=data;
		    message_state=10;
		    }
		break;
		case 10:
			if(data==(checksum&0xff))
		    {
			message_complete=1;
			d1=front_data_back;
			d2=back_data_back;
		    }
		    else
		    {
		    _num_error.Time_Checksum_error++;
		    message_state=0;
		    }
		break;
		default:
			_num_error.Time_Invalid_data++;
		break;
	   	}
       
    }


	if (message_complete==1)
		{
		// record the warning message
		if(front_data_warning==0)
		{
		update_sector_data(0, d1);
		}
		else
		{
		front_warning=1;
		}
		
		if(back_data_warning==0)
		{
		update_sector_data(180, d2);
		}
		else
		{
		back_warning=1;
		}
		}
	
	else
		{
		uncm_num++;
		if((_num_error.Time_Head_error!=0)||(_num_error.Time_Checksum_error!=0)||(_num_error.Time_Invalid_data!=0))
   	    {
   	    error_num++;
   	    }
		}
    return true;
}


uint16_t AP_Proximity_Radar_GKXN::process_distance(uint8_t buf1, uint8_t buf2)
{
    return (buf1 << 8) + buf2;
}

// process reply
void AP_Proximity_Radar_GKXN::update_sector_data(int16_t angle_deg, uint16_t distance_cm)
{
    uint8_t sector;
    if (convert_angle_to_sector(angle_deg, sector)) {
        _angle[sector] = angle_deg;
        _distance[sector] = ((float) distance_cm) / 100.0f;
	   // printf(" _distance[sector] =%f\n", _distance[sector] );
        _distance_valid[sector] = (_distance[sector] >= distance_min()) && (_distance[sector] <= distance_max());
        _last_distance_received_ms = AP_HAL::millis();
        // update boundary used for avoidance
        update_boundary_for_sector(sector);
    }
}


