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
   		&& task.get_abmode().abmode_start()) \
   	   	|| ignore_checks) {
        // initialise yaw
        set_auto_yaw_mode(get_default_auto_yaw_mode(false));
        // start in position control mode
        guided_pos_control_start();
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
    guided_run();
}

#endif

