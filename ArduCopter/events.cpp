#include "Copter.h"

/*
 *       This event will be called when the failsafe changes
 *       boolean failsafe reflects the current state
 */
void Copter::failsafe_radio_on_event()
{
    // if motors are not armed there is nothing to do
    if( !motors->armed() ) {
        return;
    }

    if (should_disarm_on_failsafe()) 
	{
        init_disarm_motors();
		
    } 
	else 
	{
        if (control_mode == AUTO && g.failsafe_throttle == FS_THR_ENABLED_CONTINUE_MISSION) 
		{
		
            // continue mission
    	} 
		else if (control_mode == LAND && g.failsafe_battery_enabled == FS_BATT_LAND && failsafe.battery) 
		{
			
            // continue landing
        } 
		else 
        {
            if (g.failsafe_throttle == FS_THR_ENABLED_ALWAYS_LAND) 
			{
                set_mode_land_with_pause(MODE_REASON_RADIO_FAILSAFE);
			
            } 
			else if(FS_THR_ENABLED_ALWAYS_RTL == g.failsafe_throttle)
			{
				set_mode_RTL_or_land_with_pause(MODE_REASON_RADIO_FAILSAFE);
			}
			else
            {
            	
            	//	modified by ZhangYong 20171212
            	switch(control_mode)
				{
					case STABILIZE:
        			case ACRO:
						
            			set_mode_RTL_or_land_with_pause(MODE_REASON_RADIO_FAILSAFE);
           				
						
						break;

					case AUTO:
						
					
						if(g.failsafe_throttle == FS_THR_ENABLED_CONTINUE_MISSION)
           				{
           				
						}
						else if(FS_THR_ENABLED_ALWAYS_RTL == g.failsafe_throttle)
						{
							
						
							if(false == set_mode(GUIDED, MODE_REASON_RADIO_FAILSAFE))
            				{
            					set_mode_land_with_pause(MODE_REASON_RADIO_FAILSAFE);
           					}							
						}
						
						break;

					default:
						if(g.failsafe_throttle == FS_THR_ENABLED_CONTINUE_MISSION)
           				{
           			
							
           					if(false == set_mode(GUIDED, MODE_REASON_RADIO_FAILSAFE))
            				{
            			
            					set_mode_land_with_pause(MODE_REASON_RADIO_FAILSAFE);
           					}						
						}
						else
						{
							
							set_mode_land_with_pause(MODE_REASON_RADIO_FAILSAFE);
						}
						break;
            	}
				//	modified end
                //set_mode_RTL_or_land_with_pause(MODE_REASON_RADIO_FAILSAFE);
            }
        }
    }

    // log the error to the dataflash
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_RADIO, ERROR_CODE_FAILSAFE_OCCURRED);

}

// failsafe_off_event - respond to radio contact being regained
// we must be in AUTO, LAND or RTL modes
// or Stabilize or ACRO mode but with motors disarmed
void Copter::failsafe_radio_off_event()
{
    // no need to do anything except log the error as resolved
    // user can now override roll, pitch, yaw and throttle and even use flight mode switch to restore previous flight mode
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_RADIO, ERROR_CODE_FAILSAFE_RESOLVED);
}

void Copter::failsafe_battery_event(void)
{
    // return immediately if low battery event has already been triggered
    if (failsafe.battery) {
        return;
    }

    // failsafe check
    if (g.failsafe_battery_enabled != FS_BATT_DISABLED && motors->armed()) {
        if (should_disarm_on_failsafe()) {
            init_disarm_motors();
        } else {
            if (g.failsafe_battery_enabled == FS_BATT_RTL || control_mode == AUTO) {
                set_mode_RTL_or_land_with_pause(MODE_REASON_BATTERY_FAILSAFE);
            } else {
                set_mode_land_with_pause(MODE_REASON_BATTERY_FAILSAFE);
            }
        }
    }

    // set the low battery flag
    set_failsafe_battery(true);

    // warn the ground station and log to dataflash
    gcs_send_text(MAV_SEVERITY_WARNING,"Low battery");
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_BATT, ERROR_CODE_FAILSAFE_OCCURRED);

}


//	added by ZhangYong for payload failsafe 20160918
/**/
void Copter::failsafe_payload_on_event(void)
{
	//	added by ZhangYong 20170214
	//	added end

//	printf("failsafe_payload_on_event, 0x%d\n", failsafe.payload);

    // return immediately if low battery event has already been triggered
//    if (failsafe.payload) 
//	{
//        return;
//    }
/*	printf("failsafe_payload_on_event, 0x%x, 0x%x, 0x%x\n", \
											g.failsafe_pld_type, \
											g.failsafe_pld_action, \
											motors.armed());
*/	

    // failsafe check
    if (\
			(\
				((g.failsafe_pld_type & FS_PLD_FM_BIT) != FS_PLD_NONE) || \
				((g.failsafe_pld_type & FS_PLD_PMBUS_BIT) != FS_PLD_NONE) \
			) && \
			(g.failsafe_pld_action != FS_PLD_DISABLE) && \
			(motors->armed()) \
		) 
	{
        switch(control_mode) 
		{
            case STABILIZE:
            case ACRO:
                // if throttle is zero OR vehicle is landed disarm motors
                if (should_disarm_on_failsafe()) 
				{
                    init_disarm_motors();
                }
				else
				{
                    // set mode to RTL or LAND
                    if (g.failsafe_pld_action == FS_PLD_ENABLED_ALWAYS_RTL) 
					{
//						printf("0x%x %4.6f\n", home_distance, wp_nav.get_wp_radius());
						//	modified by ZhangYong 20170214
//						reenter_mode = true;
//						return_to_fixorg = false;
//						rtl_last_state = Land;
						
            			set_mode_RTL_or_land_with_pause(MODE_REASON_PAYLOAD_FAILSAFE);
                    }
					else
					{
                        set_mode_land_with_pause(MODE_REASON_PAYLOAD_FAILSAFE);
                    }
                }
                break;
            case AUTO:	
				if (should_disarm_on_failsafe()) 
				{
                    init_disarm_motors();
	            } 
				else
                {
					switch(g.failsafe_pld_action)
					{
						case FS_PLD_ENABLED_ALWAYS_LAND:
						case FS_PLD_ENABLED_ALWAYS_RTL:
							
							//	modified by ZhangYong 20170214
//							reenter_mode = true;
//							return_to_fixorg = false;
//							rtl_last_state = Land;
								
            				set_mode_RTL_or_land_with_pause(MODE_REASON_PAYLOAD_FAILSAFE);
                		
							break;
							
/*							case FS_PLD_PREDEFINED_ROUTING_RTL:
								
								//printf("Predefined routing Battery RTL\n");
						if (home_distance > wp_nav.get_wp_radius()) 
							{
								if(0 == (uint16_t)g.failsafe_prrtl_wpnum)
								{
									//	modified by ZhangYong 20170214
//									reenter_mode = true;
//									return_to_fixorg = false;
//									rtl_last_state = Land;
									
            						return_value = set_mode(RTL);
									//	modified end
								
						
			if (!return_value) 
									{
                       					set_mode_land_with_pause();
                    				}
								}
								else
								{
                    				if (!mission.set_current_cmd((uint16_t)g.failsafe_prrtl_wpnum)) 
									{
										//	modified by ZhangYong 20170214
										reenter_mode = true;
										return_to_fixorg = false;
										rtl_last_state = Land;
										
            							return_value = set_mode(RTL);
										//	modified end
									
                       					if (!return_value) 
										{
                       						set_mode_land_with_pause();
                    					}
                    				}
								}	
                			}
							else
							{
                    			// We are very close to home so we will land
                    			set_mode_land_with_pause();
                			}	
							break;

						case FS_PLD_REVERSE_ROUTING_RTL:
							//	printf("Reverse routing Battery RTL\n");
							if (home_distance > wp_nav.get_wp_radius()) 
							{
                   		 		mission.set_inverted_waypoint(true);						
                			}
							else
							{
                    			// We are very close to home so we will land
                    			set_mode_land_with_pause();
                			}
							break;
*/
						default:
							break;
						
					}
				}
                break;
            default:
            	//	added by ZhangYong
            	//printf("ap.land_complete %d\n", ap.land_complete);
				//printf("home_distance %d\n", home_distance);
            	
              	if (should_disarm_on_failsafe()) 
				{
                    init_disarm_motors();

                // set mode to RTL or LAND
                }
				else 
				{
					switch(g.failsafe_pld_action)
					{
						case FS_PLD_ENABLED_ALWAYS_LAND:
						case FS_PLD_ENABLED_ALWAYS_RTL:
							
							//	modified by ZhangYong 20170214
//							reenter_mode = true;
//							return_to_fixorg = false;
//							rtl_last_state = Land;

							set_mode_RTL_or_land_with_pause(MODE_REASON_PAYLOAD_FAILSAFE);
								
            				
							break;
						default:
							break;
						
					}
				}
				break;
		}
    }
    

    // warn the ground station and log to dataflash
    gcs_send_text(MAV_SEVERITY_CRITICAL, "payload emergency!");
    
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_PLD, ERROR_CODE_FAILSAFE_OCCURRED);
/*
//    printf("failsafe_payload_on_event, 0x%x, 0x%x\n", \
//										ERROR_SUBSYSTEM_FAILSAFE_PLD, \
//										ERROR_CODE_FAILSAFE_OCCURRED);
*/

}



void Copter::failsafe_payload_off_event(void)
{
	Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_PLD, ERROR_CODE_FAILSAFE_RESOLVED);

/*	printf("failsafe_payload_off_event, 0x%x, 0x%x\n", \
										ERROR_SUBSYSTEM_FAILSAFE_PLD, \
										ERROR_CODE_FAILSAFE_RESOLVED);
*/}

//	added end


// failsafe_gcs_check - check for ground station failsafe
void Copter::failsafe_gcs_check()
{
	//	added by ZhangYong 20171212
	bool return_value;
    //	added end
    uint32_t last_gcs_update_ms;

    // return immediately if gcs failsafe is disabled, gcs has never been connected or we are not overriding rc controls from the gcs and we are not in guided mode
    // this also checks to see if we have a GCS failsafe active, if we do, then must continue to process the logic for recovery from this state.
    if ((!failsafe.gcs)&&(g.failsafe_gcs == FS_GCS_DISABLED || failsafe.last_heartbeat_ms == 0 || (!failsafe.rc_override_active && control_mode != GUIDED))) {
        return;
    }

    // calc time since last gcs update
    // note: this only looks at the heartbeat from the device id set by g.sysid_my_gcs
    last_gcs_update_ms = millis() - failsafe.last_heartbeat_ms;

    // check if all is well
    if (last_gcs_update_ms < FS_GCS_TIMEOUT_MS) {
        // check for recovery from gcs failsafe
        if (failsafe.gcs) {
            failsafe_gcs_off_event();
            set_failsafe_gcs(false);
        }
        return;
    }

    // do nothing if gcs failsafe already triggered or motors disarmed
    if (failsafe.gcs || !motors->armed()) {
        return;
    }

    // GCS failsafe event has occurred
    // update state, log to dataflash
    set_failsafe_gcs(true);
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_GCS, ERROR_CODE_FAILSAFE_OCCURRED);

    // clear overrides so that RC control can be regained with radio.
    hal.rcin->clear_overrides();
    failsafe.rc_override_active = false;
//	printf("failsafe_gcs_check false\n");

	//	added by ZhangYong to pull the throttle to bottom, and reset roll, pitch, and yaw to trim;
	channel_roll->set_pwm(channel_roll->get_radio_trim());
	channel_pitch->set_pwm(channel_pitch->get_radio_trim());
	channel_throttle->set_pwm(channel_throttle->get_radio_min());
	channel_yaw->set_pwm(channel_yaw->get_radio_trim());
	
	if((STABILIZE == control_mode) || (ACRO == control_mode) || (SPORT == control_mode))
    {
    	
    	channel_throttle->set_control_in(0);
		channel_throttle->set_radio_in(0);
	}
	else
	{
		if(FS_GCS_ENABLED_LOITER != g.failsafe_gcs)
		{
			
			channel_throttle->set_control_in(0);
			channel_throttle->set_radio_in(0);
		}
	}
	//	added end
	
//	g2.rc_channels

    if (should_disarm_on_failsafe()) {
        init_disarm_motors();

    } else {
    	//	modified by ZhangYong 20171212 to loiter the copter when gcs control signal lose
        /*if (control_mode == AUTO && g.failsafe_gcs == FS_GCS_ENABLED_CONTINUE_MISSION) {
            // continue mission
        } else if (g.failsafe_gcs != FS_GCS_DISABLED) {
            set_mode_RTL_or_land_with_pause(MODE_REASON_GCS_FAILSAFE);
        }
        */
        switch(control_mode)
        {
        	case STABILIZE:
        	case ACRO:
        	case SPORT:
				set_mode_RTL_or_land_with_pause(MODE_REASON_GCS_FAILSAFE);
				break;

			case AUTO:
				/*
				enum FS_GCSSrategy
			//	{
			//		FS_GCSDisable = 0,
			//		FS_GCSRTL = 1,
			//		FS_GCSConAuto = 2,
			//		FS_GCSLoiter = 4,
			//		FS_GCSPreRTL = 5,
			//		FS_GCSRevRTL = 6
			//	};
				*/
	
				switch(g.failsafe_gcs)
				{
					case FS_GCS_DISABLED:
					case FS_GCS_ENABLED_CONTINUE_MISSION:
					

						break;

					case FS_GCS_ENABLED_ALWAYS_RTL:
						set_mode_RTL_or_land_with_pause(MODE_REASON_GCS_FAILSAFE);

						break;

/*					case FS_GCS_ENABLED_LOITER:
						return_value = set_mode(LOITER, MODE_REASON_GCS_FAILSAFE);
						printf("LOITER %d\n", return_value);							
                		if (!return_value) 
                		{
                			set_mode_RTL_or_land_with_pause(MODE_REASON_GCS_FAILSAFE);
                		}
						break;
*/
					default:
	
						set_mode_RTL_or_land_with_pause(MODE_REASON_GCS_FAILSAFE);
						break;
				}
				
				break;	//	AUTO
				
			default:

				switch(g.failsafe_gcs)
				{
					case FS_GCS_ENABLED_CONTINUE_MISSION:
					//	if(copter.ap.rc_receiver_present && channel_throttle->get_control_in())
						return_value = set_mode(GUIDED, MODE_REASON_GCS_FAILSAFE);
						
                		if (!return_value) 
                		{
                			set_mode_RTL_or_land_with_pause(MODE_REASON_GCS_FAILSAFE);
                		}
						break;

					default:
	
						set_mode_RTL_or_land_with_pause(MODE_REASON_GCS_FAILSAFE);
						break;
				}	
				break;
				
        }
        
    }
}

// failsafe_gcs_off_event - actions to take when GCS contact is restored
void Copter::failsafe_gcs_off_event(void)
{
    // log recovery of GCS in logs?
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_GCS, ERROR_CODE_FAILSAFE_RESOLVED);
}

// executes terrain failsafe if data is missing for longer than a few seconds
//  missing_data should be set to true if the vehicle failed to navigate because of missing data, false if navigation is proceeding successfully
void Copter::failsafe_terrain_check()
{
    // trigger with 5 seconds of failures while in AUTO mode
    bool valid_mode = (control_mode == AUTO || control_mode == GUIDED || control_mode == GUIDED_NOGPS || control_mode == RTL);
    bool timeout = (failsafe.terrain_last_failure_ms - failsafe.terrain_first_failure_ms) > FS_TERRAIN_TIMEOUT_MS;
    bool trigger_event = valid_mode && timeout;

    // check for clearing of event
    if (trigger_event != failsafe.terrain) {
        if (trigger_event) {
            failsafe_terrain_on_event();
        } else {
            Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_TERRAIN, ERROR_CODE_ERROR_RESOLVED);
            failsafe.terrain = false;
        }
    }
}

// set terrain data status (found or not found)
void Copter::failsafe_terrain_set_status(bool data_ok)
{
    uint32_t now = millis();

    // record time of first and latest failures (i.e. duration of failures)
    if (!data_ok) {
        failsafe.terrain_last_failure_ms = now;
        if (failsafe.terrain_first_failure_ms == 0) {
            failsafe.terrain_first_failure_ms = now;
        }
    } else {
        // failures cleared after 0.1 seconds of persistent successes
        if (now - failsafe.terrain_last_failure_ms > 100) {
            failsafe.terrain_last_failure_ms = 0;
            failsafe.terrain_first_failure_ms = 0;
        }
    }
}

// terrain failsafe action
void Copter::failsafe_terrain_on_event()
{
    failsafe.terrain = true;
    gcs_send_text(MAV_SEVERITY_CRITICAL,"Failsafe: Terrain data missing");
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_TERRAIN, ERROR_CODE_FAILSAFE_OCCURRED);

    if (should_disarm_on_failsafe()) {
        init_disarm_motors();
    } else if (control_mode == RTL) {
        rtl_restart_without_terrain();
    } else {
        set_mode_RTL_or_land_with_pause(MODE_REASON_TERRAIN_FAILSAFE);
    }
}

// set_mode_RTL_or_land_with_pause - sets mode to RTL if possible or LAND with 4 second delay before descent starts
//  this is always called from a failsafe so we trigger notification to pilot
void Copter::set_mode_RTL_or_land_with_pause(mode_reason_t reason)
{
    // attempt to switch to RTL, if this fails then switch to Land
    if (!set_mode(RTL, reason)) {
        // set mode to land will trigger mode change notification to pilot
        set_mode_land_with_pause(reason);
    } else {
        // alert pilot to mode change
        AP_Notify::events.failsafe_mode_change = 1;
    }
}

bool Copter::should_disarm_on_failsafe() {
//	printf("should_disarm_on_failsafe delay %d, zero %d, complete %d\n", 
//			ap.in_arming_delay, ap.throttle_zero, ap.land_complete);

	if (ap.in_arming_delay) {
        return true;
    }

    switch(control_mode) {
        case STABILIZE:
        case ACRO:
            // if throttle is zero OR vehicle is landed disarm motors
            return ap.throttle_zero || ap.land_complete;
        case AUTO:
            // if mission has not started AND vehicle is landed, disarm motors
            return !ap.auto_armed && ap.land_complete;
        default:
            // used for AltHold, Guided, Loiter, RTL, Circle, Drift, Sport, Flip, Autotune, PosHold
            // if landed disarm
            return ap.land_complete;
    }
}

void Copter::update_events()
{
    ServoRelayEvents.update_events();
}

