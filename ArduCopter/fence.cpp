#include "Copter.h"

// Code to integrate AC_Fence library with main ArduCopter code

#if AC_FENCE == ENABLED

// fence_check - ask fence library to check for breaches and initiate the response
// called at 1hz
void Copter::fence_check()
{
    uint8_t new_breaches; // the type of fence that has been breached
    uint8_t orig_breaches = fence.get_breaches();

    #if RF_FENCE == ENABLED
    	float fixorg_distance;
    	Vector2f fixorg_ralative;
    	Vector3f curr_ralative;
    	bool return_value;
    	static bool reach_targrt = false;
      
      //	added by ZhangYong 20170206
	    uint8_t fence_action = fence.get_action();
	    //	added end
    #endif	
    
    // give fence library our current distance from home in meters
    fence.set_home_distance(home_distance*0.01f);

    // check for a breach
    new_breaches = fence.check_fence(current_loc.alt/100.0f);

    #if RF_FENCE == ENABLED
    	//	added by ZhangYong 20161219
      //	fixorg_distance = inertial_nav.calc_fixorg_distance(fence.return_fixorg_lat(), fence.return_fixorg_lng());
    	Location fixorg;
    	
    	fixorg.lat = fence.return_fixorg_lat();
    	fixorg.lng = fence.return_fixorg_lng();
    	fixorg_distance = get_distance_cm(current_loc,fixorg);//cm
    
    	fence.change_fixorg_distance(fixorg_distance*0.01f);	
    	//	added end
    	
    	//	added by ZhangYong 20161230
    	//	printf("fixorg_distance = %4.4f\n", fixorg_distance*0.01f);
    	//	added end
    		
    #endif

    // return immediately if motors are not armed
    if(!motors->armed()) {
        return;
    }

    // if there is a new breach take action
    if( new_breaches != AC_FENCE_TYPE_NONE ) {

        // if the user wants some kind of response and motors are armed
        if(fence.get_action() != AC_FENCE_ACTION_REPORT_ONLY ) {

            // disarm immediately if we think we are on the ground or in a manual flight mode with zero throttle
            // don't disarm if the high-altitude fence has been broken because it's likely the user has pulled their throttle to zero to bring it down
            if (ap.land_complete || (mode_has_manual_throttle(control_mode) && ap.throttle_zero && !failsafe.radio && ((fence.get_breaches() & AC_FENCE_TYPE_ALT_MAX)== 0))){
                init_disarm_motors();
            }else{
                // if we are within 100m of the fence, RTL
                if (fence.get_breach_distance(new_breaches) <= AC_FENCE_GIVE_UP_DISTANCE) {
                  
#if RF_FENCE == ENABLED
                    switch(fence_action)
                  	{
                  			case AC_FENCE_ACTION_RTL_AND_LAND:
                        
                  			     //	modified by ZhangYong 20170214
                             return_value = set_mode(RTL,MODE_REASON_FENCE_BREACH);
                  						//	modified end
                  				
                  					 if (!return_value) 
                  					 {
                                set_mode(LAND,MODE_REASON_FENCE_BREACH);
                             }
                  					 break;
                  							
                  	    case AC_FENCE_ACTION_BACKAWAY_AND_HOVER:
                  					curr_ralative.x = 0; 
                  					curr_ralative.y = 0;
                  
                  					curr_ralative = inertial_nav.get_position();
                  
                  					if(true == get_ralative_postion(&(fixorg_ralative.x), &(fixorg_ralative.y), fence.return_fixorg_lat(), fence.return_fixorg_lng()))
                  					{
                  						if(set_mode(GUIDED,MODE_REASON_FENCE_BREACH)){
                  							  guided_set_destination(fence.calc_backaway_destination(curr_ralative, fixorg_ralative, fixorg_distance));
                  							  reach_targrt = true;
                  						}else
                  								set_mode(LAND,MODE_REASON_FENCE_BREACH);
                  						}
                  					break;
                  		  default:
                  					break;
                   }
#else
                   if (!set_mode(RTL, MODE_REASON_FENCE_BREACH)) {
                            set_mode(LAND, MODE_REASON_FENCE_BREACH);
                   }
#endif
                    // if (!set_mode(RTL, MODE_REASON_FENCE_BREACH)) {
                    //     set_mode(LAND, MODE_REASON_FENCE_BREACH);
                    // }
                }else{
                    // if more than 100m outside the fence just force a land
                    set_mode(LAND, MODE_REASON_FENCE_BREACH);
                }
            }
        }

        // log an error in the dataflash
        Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_FENCE, new_breaches);
    }

    // record clearing of breach
    if(orig_breaches != AC_FENCE_TYPE_NONE && fence.get_breaches() == AC_FENCE_TYPE_NONE) {
        Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_FENCE, ERROR_CODE_ERROR_RESOLVED);
    }
    
    #if RF_FENCE == ENABLED
    	//baiyang added in 20170718
    	if(reach_targrt && wp_nav->reached_wp_destination()){
    		set_mode(LOITER,MODE_REASON_FENCE_BREACH);
    		reach_targrt = false;
    	}
    	//added end
    #endif
}

// fence_send_mavlink_status - send fence status to ground station
void Copter::fence_send_mavlink_status(mavlink_channel_t chan)
{   
    if (fence.enabled()) {
        // traslate fence library breach types to mavlink breach types
        uint8_t mavlink_breach_type = FENCE_BREACH_NONE;
        uint8_t breaches = fence.get_breaches();
        if ((breaches & AC_FENCE_TYPE_ALT_MAX) != 0) {
            mavlink_breach_type = FENCE_BREACH_MAXALT;
        }
        if ((breaches & (AC_FENCE_TYPE_CIRCLE | AC_FENCE_TYPE_POLYGON)) != 0) {
            mavlink_breach_type = FENCE_BREACH_BOUNDARY;
        }

        // send status
        mavlink_msg_fence_status_send(chan,
                                      (int8_t)(fence.get_breaches()!=0),
                                      fence.get_breach_count(),
                                      mavlink_breach_type,
                                      fence.get_breach_time());
    }
}

#if RF_FENCE == ENABLED
//baiyang added in 20170717
bool Copter::get_ralative_postion(float *x, float *y, int32_t para_lat, int32_t para_lng)
{
	if((0 == para_lat) || (0 == para_lng))
	{
		return false;
	}

	*x = (float)(para_lat - ahrs.get_home().lat) * LATLON_TO_CM;
    *y = (float)(para_lng - ahrs.get_home().lng) * longitude_scale(ahrs.get_home())*LATLON_TO_CM;

	return true;
}
//added end
#endif

#endif
