#include "Copter.h"

#if ABMODE == ENABLED
/*
 * Init and run calls for abmode flight mode
 *
 * This file contains the implementation for Land, Waypoint navigation and Takeoff from Auto mode
 * Command execution code (i.e. command_logic.pde) should:
 *      a) switch to Auto flight mode with set_mode() function.  This will cause auto_init to be called
 * The main loop (i.e. fast loop) will call update_flight_modes() which will in turn call abmode_run() which, based upon the auto_mode variable will call
 *      correct abmode_wp_run to actually implement the feature
 */

/*
 *  While in the auto flight mode, navigation or do/now commands can be run.
 *  Code in this file implements the navigation commands
 */

// auto_init - initialise auto controller
bool Copter::abmode_init(bool ignore_checks)
{
   //guided_init(ignore_checks);
   
   if ((position_ok() 
   		&& rf_abmode.abmode_start()) \
   	   	|| ignore_checks) {
        // initialise yaw
        set_auto_yaw_mode(get_default_auto_yaw_mode(false));
        // start in position control mode
        guided_pos_control_start();

        // initialize vertical speed and acceleration
        pos_control->set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
        pos_control->set_accel_z(g.pilot_accel_z);
		
        return true;
    }else{
        return false;
    }
}

// auto_run - runs the auto controller
//      should be called at 100hz or more
//      relies on run_autopilot being called at 10hz which handles decision making and non-navigation related commands
void Copter::abmode_run()
{
    //guided_run();
    abmode_pos_control_run();
}

// abmode_pos_control_run - runs the abmode position controller
// called from guided_run
void Copter::abmode_pos_control_run()
{
	//	added by ZhangYong 20180202
	//	in current control mode, guided, no terral follow, 
	surface_tracking_climb_rate = 0;
	//	added end


    // if not auto armed or motors not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock() || ap.land_complete) {
    //if (!motors->armed() || !ap.auto_armed || !motors->get_interlock()) {
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0, 0, 0, get_smoothing_gain());
        attitude_control->set_throttle_out(0,false,g.throttle_filt);
#else
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        // multicopters do not stabilize roll/pitch/yaw when disarmed
        attitude_control->set_throttle_out_unstabilized(0,true,g.throttle_filt);
#endif
        return;
    }

    // initialize vertical speed and acceleration
    pos_control->set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control->set_accel_z(g.pilot_accel_z);
		
    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
		
        // get pilot desired climb rate
        surface_tracking_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        surface_tracking_climb_rate = constrain_float(surface_tracking_climb_rate, -g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    }else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        wp_nav->clear_pilot_desired_acceleration();
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // run waypoint controller
    failsafe_terrain_set_status(wp_nav->update_wpnav_xy());

	//	added by ZhangYong 20180202 
	//	in order to realize terra follow with rangefinder
	if (rangefinder_alt_ok()) 
	{
		//	added by zhangyong 20180205 
		//printf("guided_pos_control_run\n");
		//	added end
		
		// if rangefinder is ok, use surface tracking
		surface_tracking_climb_rate = get_surface_tracking_climb_rate(surface_tracking_climb_rate, pos_control->get_alt_target(), G_Dt);
		//printf("%4.2f\n", target_climb_rate);

		pos_control->set_alt_target_from_climb_rate_ff(surface_tracking_climb_rate, G_Dt, false);
	}
	else
	{
		pos_control->set_alt_target_from_climb_rate(surface_tracking_climb_rate, G_Dt, false);
	}
	//	added end
	
    // update altitude target and call position controller
    //pos_control->set_alt_target_from_climb_rate_ff(surface_tracking_climb_rate, G_Dt, false);
    //pos_control->set_alt_target_from_climb_rate(surface_tracking_climb_rate, G_Dt, false);
    pos_control->update_z_controller();

    // call attitude controller
    if (auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate, get_smoothing_gain());
    } else if (auto_yaw_mode == AUTO_YAW_RATE) {
        // roll & pitch from waypoint controller, yaw rate from mavlink command or mission item
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), get_auto_yaw_rate_cds(), get_smoothing_gain());
    } else {
        // roll, pitch from waypoint controller, yaw heading from GCS or auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), get_auto_heading(), true, get_smoothing_gain());
    }
/*
	static int step = 0;
	step++;
	if(step>400)
	{
		step = 0;
		printf("abmode run,V %4.4f\n",surface_tracking_climb_rate);
	}
*/

}

// guided_set_destination - sets guided mode's target destination
// Returns true if the fence is enabled and guided waypoint is within the fence
// else return false if the waypoint is outside the fence
bool Copter::abmode_set_destination_xy(const Vector3f& destination, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw)
{
    // ensure we are in position control mode
    if (guided_mode != Guided_WP) {
        guided_pos_control_start();
    }

#if AC_FENCE == ENABLED
    // reject destination if outside the fence
    Location_Class dest_loc(destination);
    if (!fence.check_destination_within_fence(dest_loc)) {
        Log_Write_Error(ERROR_SUBSYSTEM_NAVIGATION, ERROR_CODE_DEST_OUTSIDE_FENCE);
        // failure is propagated to GCS with NAK
        return false;
    }
#endif

    // set yaw state
    guided_set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

    // no need to check return status because terrain data is not used
    wp_nav->set_wp_destination_xy(destination, false);

    // log target
    Log_Write_GuidedTarget(guided_mode, destination, Vector3f());
    return true;
}

// sets guided mode's target from a Location object
// returns false if destination could not be set (probably caused by missing terrain data)
// or if the fence is enabled and guided waypoint is outside the fence
bool Copter::abmode_set_destination_xy(const Location_Class& dest_loc, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw)
{
    // ensure we are in position control mode
    if (guided_mode != Guided_WP) {
        guided_pos_control_start();
    }

#if AC_FENCE == ENABLED
    // reject destination outside the fence.
    // Note: there is a danger that a target specified as a terrain altitude might not be checked if the conversion to alt-above-home fails
    if (!fence.check_destination_within_fence(dest_loc)) {
        Log_Write_Error(ERROR_SUBSYSTEM_NAVIGATION, ERROR_CODE_DEST_OUTSIDE_FENCE);
        // failure is propagated to GCS with NAK
        return false;
    }
#endif

    if (!wp_nav->set_wp_destination_xy(dest_loc)) {
        // failure to set destination can only be because of missing terrain data
        Log_Write_Error(ERROR_SUBSYSTEM_NAVIGATION, ERROR_CODE_FAILED_TO_SET_DESTINATION);
        // failure is propagated to GCS with NAK
        return false;
    }

    // set yaw state
    guided_set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

    // log target
    Log_Write_GuidedTarget(guided_mode, Vector3f(dest_loc.lat, dest_loc.lng, dest_loc.alt),Vector3f());
    return true;
}


#endif

