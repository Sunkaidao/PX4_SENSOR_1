#include <AP_HAL/AP_HAL.h>
#include "AP_Proximity_Radar_GKXN.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Math/crc.h>
#include <ctype.h>
#include <stdio.h>
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
	//	printf("%x\n",_last_distance_received_ms);
    } else {
        set_status(AP_Proximity::Proximity_Good);
    }
}

// get maximum and minimum distances (in meters) of primary sensor
float AP_Proximity_Radar_GKXN::distance_max() const
{
    return 4.5f;
}
float AP_Proximity_Radar_GKXN::distance_min() const
{
    return 0.20f;
}
bool AP_Proximity_Radar_GKXN::send_sensor_command()
{
	if (uart == nullptr) {
		  return false;
	  }
//	printf("send sensor command\n");
	uint8_t checkByte=0;
	int i,j;
	uint8_t orientation=0x00;
	for(i=0;i<8;i++)
    {
      TX_Buff[i]=0;
	}
	//float pitch_radar=ahrs.pitch_sensor;
	//printf("pitch radar=%f",pitch_radar);
	TX_Buff[0]=0x55;
	TX_Buff[1]=0x0C;
	TX_Buff[2]=0x00;
	TX_Buff[3]=0x00;
	uint16_t rc2_in=RC_Channels::rc_channel(CH_2)->get_radio_in();
	if (rc2_in<1600) orientation=0x00;
	else if(rc2_in>1600) orientation=0x01;
	TX_Buff[4]=orientation;
	//printf("%x",orientation);
	TX_Buff[6]=0x00;
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
	//printf("nbytes=%x\n",nbytes);
	uint16_t front_data_back=0;
	uint16_t back_data_back=0;
	int      k=0;
	uint16_t checksum = 0x00;
	uint8_t message_state=0;
	int      message_complete=0;
	uint16_t d1=0x00;
	uint16_t d2=0x00;
    
    while (nbytes-- > 0)  {
		
       uint8_t data=uart->read();
	//   printf("%x",data);
	   switch(message_state)
	   	{case 0:
	   	  if(data==0x55)
	   	  	{_num_error.Time_Head_error=0;
		     _num_error.Time_Invalid_data=0;
		     _num_error.Time_Checksum_error=0 ;
			 front_data_back=0;
	         back_data_back=0;
			 checksum=0x00;
			 k=0;
			 checksum+=data;
			 message_complete=0;
			 message_state=1;
			 // printf("case0\n");
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
		//	printf("case1\n");
		  	}
		  else
		 	{
		 	_num_error.Time_Invalid_data++;
		 	message_state=0;
		 	}
		  break;
		  case 2:
		 // printf("case2 data=%x",data);
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
			//	printf("case3\n");
				if(data!=0xff)
				{
				front_data_back=(data<<8);
			    checksum+=data;
				message_state=4;
				//printf("front_data_back=%x\n",front_data_back);
			//	printf("case3 data=%x\n",data);
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
				//   	printf("Case4\n");
					   if((k==1)&&(data==0xff))
				   {
				       k=0;
			           checksum+=data;
					   front_data_back+=data;
					   message_state=5;
				     }
				      else
				   {
				      front_data_back+=data;
				      checksum+=data;
				      k=0;
				      message_state=5;
			//		  printf("case4 data=%x\n",data);
					}
			          break;
					 case 5:
					 //	printf("case5\n");
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
							//printf("case6\n");
							if((k==1)&&(data==0xff))
				             {
				               k=0;
							   checksum+=data;
							 //  printf("back_data_back=%x\n",back_data_back);
							   back_data_back+=data;
							//   printf("back_data_back+=%x\n",back_data_back);
				               message_state=7;
			                   
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
							//	printf("case7\n");
								checksum+=data;
							//    printf("checksum=%x",checksum);
								message_state=8;
							 break;
							 case 8:
							 //	printf("case8\n");
							 	checksum+=data;
								message_state=9;
							  break;
							  case 9:
							  //	printf("case9 before data=%x",data);
							  //	printf("case9\n");
							  	if ((data&0x01)==0x01)
							  		{
							  		_num_error.Time_Invalid_data++;
									message_state=10;
									checksum+=data;
							  		}
							    else
									{
								//	printf("case9 data=%x",data);
									checksum+=data;
									message_state=10;
								//	printf("checksum=%x",checksum);
									}
								break;
								case 10:
								//	printf("case10\n");
								//printf("checksum=%x\n",checksum);
								//printf("data=%x\n",data);
									if(data==(checksum&0xff))
										{//printf("checksum right\n");
										 message_complete=1;
									     d1=front_data_back;
									//	 printf("d1=%x\n",d1);
									     d2=back_data_back;
										//  printf("d2=%x\n",d2);
										}
		                           else
		                           	{
		                           	message_state=0;
		                           	}
	   	}
       
    }
	//printf("\n");
	if (message_complete==1)
		{
	//	printf("message_complete==1");
		update_sector_data(0, d1);
		update_sector_data(180, d2);
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
        _distance_valid[sector] = distance_cm ;
        _last_distance_received_ms = AP_HAL::millis();
        // update boundary used for avoidance
        update_boundary_for_sector(sector);
    }
}

