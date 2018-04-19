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
	if (uart == nullptr) {
		   return false;
	   }
	  // tick++;
	   uint8_t numc,j;
	   uint8_t k;
	   uint8_t *p1;
	   uint8_t nbytes = uart->available();
	   reading_cm=0;
	   if(nbytes!=0)
	   {p1=new uint8_t [nbytes];
	   for(k=0;k<nbytes;k++)
		   {
		   uint8_t data=uart->read();
		   p1[k]=data;
		   }
	   numc=nbytes;
	   while (nbytes-- > 0)
		   {
		   j=numc-nbytes-1;
		   switch(message_state)
			   {
			   case 0:
				   if(p1[j]==0xfe)
					   {
						_num_error.Time_Head_error=0;
						_num_error.Time_Invalid_data=0;
						_num_error.Time_Tail_error=0 ; 
						count=0;
						checksum=0x00;
						distance0=distance1=0;
						checksum+=p1[j]; 
						message_state=1;
						reading_cm=0;
					   }
				   else if(p1[j]==0xf0)
					   {
						 message_state=0;
						 reading_cm=0x1388;
						}
				   else
					   {
						 _num_error.Time_Head_error++;
						 message_state=0;
					   }
			   break;
			   case 1:	
				   if((p1[j]>0)&&(p1[j]<4))
					   {
					   checksum+=p1[j];
					   message_state=2;
					   }
				   else 
					   {
					   message_state=0;
					   _num_error.Time_Invalid_data++;
					   }
			   break;
			   case 2:
			   	   distance0=p1[j];
			       checksum+=p1[j];
				   message_state=3;
			   break;
			   case 3:
			   	    distance1=p1[j];
				    checksum+=p1[j];
				    message_state=4;
			   break;
			   case 4:
			      if(count<4)
					   {
					   checksum+=p1[j];
					   count++;
					   message_state=4;
					   }	
				   else if(count==4)
					   {
					   if(p1[j]==(checksum&0x007f))
						   {
						  reading_cm=distance0*0x80+distance1;
						  message_state=0;  
						   }
					   else
						   {
						   message_state=0;
						   _num_error.Time_Tail_error++;
						   } 
					   }
				   else 
					   {
					   message_state=0;
					   _num_error.Time_Invalid_data++;
					   }
			   break;
			   default:
			   break;								   
			   }
		 /*  //record the number of mistake
		  if((_num_error.Time_Head_error!=0)||(_num_error.Time_Invalid_data!=0)||(_num_error.Time_Tail_error!=0))
		   	{num++;}
		   if(tick==50)
		   	{
			tick=0;
			num=0;
		   	}*/
		   }
	   delete[] p1;
	   }  
	return true;
	}	
	

void AP_RangeFinder_Radar_ZHIBO::update(void)
{
    if (get_reading(state.distance_cm)) {
        // update range_valid state based on distance measured
        last_reading_ms = AP_HAL::millis();
        update_status();
//		printf("AP_RangeFinder_Radar_ZHIBO::update right\n");
	} else if (AP_HAL::millis() - last_reading_ms > 200) {
        set_status(RangeFinder::RangeFinder_NoData);
//		printf("AP_RangeFinder_Radar_ZHIBO::update wrong\n");
    }
}

