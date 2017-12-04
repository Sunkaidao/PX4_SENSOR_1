 #include <AP_HAL/AP_HAL.h>
#include "AP_RangeFinder_Radar.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>



extern const AP_HAL::HAL& hal;

AP_RangeFinder_Radar::AP_RangeFinder_Radar(RangeFinder &_ranger, uint8_t instance,
                                                               RangeFinder::RangeFinder_State &_state,
                                                               AP_SerialManager &serial_manager) :
    AP_RangeFinder_Backend(_ranger, instance, _state, MAV_DISTANCE_SENSOR_LASER)
{
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Radar, 0);
    if (uart != nullptr) {
       uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Radar, 0));
    }
}
bool AP_RangeFinder_Radar::detect(RangeFinder &_ranger, uint8_t instance, AP_SerialManager &serial_manager)
{
    return serial_manager.find_serial(AP_SerialManager::SerialProtocol_Radar, 0) != nullptr;
}
bool AP_RangeFinder_Radar::get_reading(uint16_t &reading_cm)
{   
	uint8_t checksum = 0x00;
	static uint16_t distance = 0;
	uint8_t message_state=0;
	uint8_t message_complete=0;
	static uint8_t message_status=0;
	int      count=0;
	//printf("radar get reading \n");
	
    if (uart == nullptr) {
        return false;
	//	printf("nullptr fasle\n");
    }
  
    // set buffer and init
	int16_t nbytes = uart->available();
//	printf("nbytes=%x",nbytes);
	reading_cm=0;
	distance=0;
	while (nbytes-- > 0)
		{
		uint16_t data=uart->read();
	//printf("data=%x",data);
		
	//printf("\n");
	//printf("read data from radar\n");
	//check the message is complete or not 
	switch(message_state)
		{
		case 0:
		 if(data==0xAA)
		 	{
		     _num_error.Time_Head_error=0;
		     _num_error.Time_Invalid_data=0;
			 _num_error.Time_Payload_error=0;
			 _num_error.Time_Tail_error=0 ; 
			 count=0;
			 message_state=1;
		//	 printf("case0 init \n");
		 	}
		 else
		 	{
		 	_num_error.Time_Head_error++;
		 	message_state=0;
		 	}
		 break;
		 case 1:
		  if(data==0xAA)
		 	{message_state=2;
		  //   printf("case1  \n");
		 	}
		  else
		 	{
		 	_num_error.Time_Head_error++;
		 	message_state=0;
		 	}
		  break;
		  case 2:
		  	
		 // printf("case2 data=%x",data);
		 	if(data==0x0A)
		 		{
		 	//	printf("message_status=1\n");
		 		message_status=1;
				message_state=3;
		 		}
		    else if((data==0x0B)&&(message_status==1))
		    	{
		    //	printf("message_status=2\n");
		    	message_status=2;
				message_state=4;
		    	}
			else if((data==0x0C)&&(message_status==2))
				{
			//	printf("message_status=3\n");
				message_status=3;
				message_state=4;
				
				}
			else 
				{
				_num_error.Time_Invalid_data++;
				message_state=0;
				}
			break;
		    case 3:
			//	printf("case3\n");
			 if(data==0x06)
				{
				message_state=5;
				}
			 else 
				{
				_num_error.Time_Invalid_data++;
			    message_state=0;
				}
			 break;
			 case 4:
			 //	printf("case4=%x\n",data);
			 	if(data==0x07)
			 		{
			 		message_state=5;
			 		}
				else
					{
					_num_error.Time_Invalid_data++;
				    message_state=0;
					}
				break;
				case 5:
				//	printf("case5 data=%x\n",data);
					if(data==0x01)
						{
						checksum=0;
					    checksum+=data;
						message_state=6;
						count+=1;
						}
					else
						{
						_num_error.Time_Payload_error++;
						message_state=0;
						}
					break;
				    case 6:
					//	printf("case6 data=%x\n",data);
						//printf("case6\n");
					 // printf("message_status=%x",message_status);
						if(message_status==1)
							{ 
						//	printf("message_status=1\n");
							if((data==0x10)||(data==0x11)||(data==0x12)||(data==0x13))
								{
								checksum+=data;
								count+=1;
								message_state=7;
								}
							else
								{
								_num_error.Time_Payload_error++;
						        message_state=0;
								}
							}
						else if(message_status==2)
							{
						//	printf("message_status=2\n");
							if((data==0x00)||(data==0x01)||(data==0x02)||(data==0x03))
								{
								checksum+=data;
								count+=1;
								message_state=7;
								}
							else
								{
								_num_error.Time_Payload_error++;
						        message_state=0;
							    }
							}
						else if(message_status==3)
							{ 
							// printf("message_status=3\n");
							    checksum+=data;
								count+=1;
								message_state=10;
							}
						break;
						case 7:
						//	printf("case7\n");
							if((data==0x00)&&(count<6))
								{
								checksum+=data;
								count+=1;
								message_state=7;
								}
							else if(count==6)
								{
								checksum+=data;
								count+=1;
								message_state=8;
								}
				           else
				           	{
				           	    _num_error.Time_Payload_error++;
						        message_state=0;
				           	}
						   break;
						   case 8:
						  // 	printf("case8\n");
						   	if(data==(checksum&0xFF))
						   		{
						   		message_state=9;
						   		}
							else
								{
								_num_error.Time_Invalid_data++;
								message_state=0;
								}
							break;
							case 9:
							//	printf("case9\n");
								if(data==0x55)
						   		{
						   		message_state=13;
						   		}
								else
								{
								_num_error.Time_Tail_error++;
								message_state=0;
								}
								break;
								case 10:
								//	printf("case10\n");
								 distance=data*0x100;
								 checksum+=data;
								 count+=1;
								 message_state=11;
								 break;
								 case 11:
								 //	printf("case11\n");
									distance+=data;
									checksum+=data;
								    count+=1;
								    message_state=12;
									break;
									case 12:
									//	printf("case12 data=%x\n",data);
										if(count<6)
											{
											checksum+=data;
								            count+=1;
								            message_state=12;
											}
										else if(count==6)
											{
											checksum+=data;
								            count+=1;
								            message_state=8;
										//	printf("go to checksum");
											}
										else
											{
											_num_error.Time_Payload_error++;
						                    message_state=0;
											}
										break;
										case 13:
										//	printf("case13\n");
											if(data==0x55)
												{
												message_state=0;
												if (message_status==01)
													{
												
													//printf("radar receive the status message\n");
													}
												if (message_status==02)
													{
													//printf("radar receive the target status message\n");
													}
												else if(message_status==03)
													{ message_complete=1;
													//printf("diatance=%x\n",distance);
													
													
													//printf("radar receive the distance message\n");
													}
												}
											else
								                {
								                 _num_error.Time_Tail_error++;
								                 message_state=0;
												 
								                 }
								
		}
   	
   }
	if(message_complete==1)
		{
	//	printf("diatance=%x\n",distance);
		reading_cm=distance;
		
	//	printf("reading_cm=%x\n",reading_cm);
		}
	return true;
	}	
	

void AP_RangeFinder_Radar::update(void)
{
    if (get_reading(state.distance_cm)) {
        // update range_valid state based on distance measured
        last_reading_ms = AP_HAL::millis();
        update_status();
    } else if (AP_HAL::millis() - last_reading_ms > 200) {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}

