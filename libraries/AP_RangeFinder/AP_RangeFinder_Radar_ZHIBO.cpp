#include <AP_HAL/AP_HAL.h>
#include "AP_RangeFinder_Radar_ZHIBO.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>



extern const AP_HAL::HAL& hal;

AP_RangeFinder_Radar_ZHIBO::AP_RangeFinder_Radar_ZHIBO(RangeFinder::RangeFinder_State &_state,
                                   AP_SerialManager &serial_manager) :
    AP_RangeFinder_Backend(_state)
{
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Radar_ZHIBO, 0);
    if (uart != nullptr) {
       uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Radar_ZHIBO, 0));
    }
}
bool AP_RangeFinder_Radar_ZHIBO::detect(AP_SerialManager &serial_manager)
{
    return serial_manager.find_serial(AP_SerialManager::SerialProtocol_Radar_ZHIBO, 0) != nullptr;
}
bool AP_RangeFinder_Radar_ZHIBO::get_reading(uint16_t &reading_cm)
{   
	uint8_t checksum = 0x00;
	uint16_t distance0 = 0;
	uint16_t distance1=0;
	uint8_t message_state=0;
	uint8_t message_complete=0;
	int      count=0;
	int      target_num=0;
	//printf("radar get reading \n");
	
    if (uart == nullptr) {
        return false;
	 //	printf("nullptr fasle\n");
    }
  
    // set buffer and init
	int16_t nbytes = uart->available();
//	printf("nbytes=%x",nbytes);
	reading_cm=0;
	while (nbytes-- > 0)
		{
		uint16_t data=uart->read();
	//printf("data=%x\n",data);
	//printf("read data from radar\n");
	//check the message is complete or not 
	switch(message_state)
		{
		case 0:
		 if(data==0xfe)
		 	{
		     _num_error.Time_Head_error=0;
		     _num_error.Time_Invalid_data=0;
			 _num_error.Time_Payload_error=0;
			 _num_error.Time_Tail_error=0 ; 
			 count=0;
			 checksum=0x00;
			 distance0=distance1=0;
			 message_complete=0;
			 target_num=0;
			 checksum+=data; 
			 count+=1;
			 message_state=1;
//			printf("case0 count=%d\n",count);
//			 printf("case0 init \n");
		 	}
		 else if(data==0xf0)
		 	{
		 	//printf("no target\n");
		 	message_state=0;
		 	}
		 else
		 	{
		 	_num_error.Time_Head_error++;
		 	message_state=0;
		 	}
		 break;
		 case 1:
		  if((data>0)&&(data<4))
		 	{target_num=data;
		     checksum+=data;
		     count+=1;
		 	message_state=2;
//			printf("case1 count=%d\n",count);
//		     printf("case1  \n");
		 	}
		  else
		 	{
		 	_num_error.Time_Invalid_data++;
		 	message_state=0;
		 	}
		  break;
		  case 2:
		  	
		//  printf("case2 \n");
		 	if(target_num>0)
		 		{
		 //		printf("target_num=%d\n",target_num);
		 		distance1=0;
			    distance1=data*0x80;
				checksum+=data;
				count+=1;
				message_state=3;
			//	printf("case2 count=%d\n",count);
		 		}
		    else if(target_num==0)
		    	{
		    //	printf("target_num==0\n");
			    if(count<7)
			 		{
			 		checksum+=data;
					count+=1;
			 		message_state=4;
				//	printf("count=%d\n",count);
				//	printf("nbytes=%x\n",nbytes);
			 		}
			    else if(count==7)
					{
					checksum+=data;
					message_state=5;
				//	printf("count==8\n");
					}
		    	}
			else 
				{
				_num_error.Time_Payload_error++;
				message_state=0;
				}
			break;
		    case 3:
			//	printf("case3\n");
			 
				distance1+=data;
				checksum+=data;
				target_num-=1;
				count+=1;
				if(distance0==0)
					{distance0=distance1;
					message_state=2;
					}
				else 
					{ 
				if (distance0>distance1)
					{
					distance0=distance1;
					}
				message_state=2;
//				printf("distance1=%x\n",distance1);
//				printf("distance0=%x\n",distance0);
				}
			//	printf("case3 count=%d\n",count);
			 break;
			 case 4:
			 //	printf("case4\n");
			 	if(count<7)
			 		{
			 		checksum+=data;
					count+=1;
			 		message_state=4;
			//		printf("count=%d\n",count);
			//		printf("nbytes=%x\n",nbytes);
			 		}
			    else if(count==7)
					{
					checksum+=data;
					message_state=5;
				//	printf("count==7\n");
				//	printf("nbytes=%x\n",nbytes);
					}
				else
					{
					_num_error.Time_Payload_error++;
				    message_state=0;
				//	printf("case 4 error\n");
					}
				break;
				case 5:
				//	printf("case5 \n");
					if(data==(checksum&0x007f))
						{
						message_complete=1;
						message_state=0;
					//	printf("message complete\n");
						}
					else
						{
						_num_error.Time_Tail_error++;
						message_state=0;
						}
					break;
				   
								
		}
   	
   }
   	
	if(message_complete==1)
		{
	//	printf("diatance0=%x\n",distance0);
		reading_cm=distance0;
		
	//	printf("reading_cm=%x\n",reading_cm);
		}
	return true;
	}	
	

void AP_RangeFinder_Radar_ZHIBO::update(void)
{
    if (get_reading(state.distance_cm)) {
        // update range_valid state based on distance measured
        last_reading_ms = AP_HAL::millis();
        update_status();
    } else if (AP_HAL::millis() - last_reading_ms > 200) {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}

