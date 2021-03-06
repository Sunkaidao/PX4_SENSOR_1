#include "Copter.h"

/**
 *
 * Detects failures of the ekf or inertial nav system triggers an alert
 * to the pilot and helps take countermeasures
 *
 */

#ifndef EKF_CHECK_ITERATIONS_MAX
 # define EKF_CHECK_ITERATIONS_MAX          10      // 1 second (ie. 10 iterations at 10hz) of bad variances signals a failure
#endif

#ifndef EKF_CHECK_WARNING_TIME
 # define EKF_CHECK_WARNING_TIME            (30*1000)   // warning text messages are sent to ground no more than every 30 seconds
#endif

////////////////////////////////////////////////////////////////////////////////
// EKF_check strucutre
////////////////////////////////////////////////////////////////////////////////
//	modified by zhangyong for arming check
/*static struct {
    uint8_t fail_count;         // number of iterations ekf or dcm have been out of tolerances
    uint8_t bad_variance : 1;   // true if ekf should be considered untrusted (fail_count has exceeded EKF_CHECK_ITERATIONS_MAX)
    uint32_t last_warn_time;    // system time of last warning in milliseconds.  Used to throttle text warnings sent to GCS
    //	added by zhangyong for different variance
    EKF_VARIANCE_TYPE type;
	//	added end
} ekf_check_state;
*/

static EKF_check_state ekf_check_state;


// ekf_check - detects if ekf variance are out of tolerance and triggers failsafe
// should be called at 10hz
void Copter::ekf_check()
{
	//ekf_check_state.test_counter ++;

	
    // exit immediately if ekf has no origin yet - this assumes the origin can never become unset
    Location temp_loc;
    if (!ahrs.get_origin(temp_loc)) {
        return;
    }

	//printf("%d %d %f\n", !motors->armed(), ap.usb_connected, g.fs_ekf_thresh);

    // return immediately if motors are not armed, ekf check is disabled, not using ekf or usb is connected
    if (!motors->armed() || ap.usb_connected || (g.fs_ekf_thresh <= 0.0f)) {
        ekf_check_state.fail_count = 0;
        ekf_check_state.bad_variance = false;
	
		//	added by zhangyong for ekc variance 20180813
		ekf_check_state.type = EKF_VARIANCE_NON;
		//	added end
		
		AP_Notify::flags.ekf_bad = ekf_check_state.bad_variance;
        failsafe_ekf_off_event();   // clear failsafe
        
        return;
    }

    // compare compass and velocity variance vs threshold
    if (ekf_over_threshold()) {
        // if compass is not yet flagged as bad
        if (!ekf_check_state.bad_variance) {
            // increase counter
            ekf_check_state.fail_count++;
            // if counter above max then trigger failsafe
            if (ekf_check_state.fail_count >= EKF_CHECK_ITERATIONS_MAX) {
                // limit count from climbing too high
                ekf_check_state.fail_count = EKF_CHECK_ITERATIONS_MAX;
                ekf_check_state.bad_variance = true;
                // log an error in the dataflash
                Log_Write_Error(ERROR_SUBSYSTEM_EKFCHECK, ERROR_CODE_EKFCHECK_BAD_VARIANCE);
                // send message to gcs
                if ((AP_HAL::millis() - ekf_check_state.last_warn_time) > EKF_CHECK_WARNING_TIME) {
					//	modified by zhangyong 
                    //gcs().send_text(MAV_SEVERITY_CRITICAL,"EKF variance");
					//	modified end
					switch(ekf_check_state.type)
					{
						case EKF_VARIANCE_MAG:
							gcs().send_text(MAV_SEVERITY_CRITICAL,"EKF variance MAG");
							break;
						case EKF_VARIANCE_VEL:
							gcs().send_text(MAV_SEVERITY_CRITICAL,"EKF variance VEL");
							break;
							
						case EKF_VARIANCE_POS:
							gcs().send_text(MAV_SEVERITY_CRITICAL,"EKF variance POS");
							break;
							
						default:
							gcs().send_text(MAV_SEVERITY_CRITICAL,"EKF variance");
							
							break;
					}

				ekf_check_state.last_warn_time = AP_HAL::millis();
                }
                failsafe_ekf_event();
            }
        }
    } else {
        // reduce counter
        if (ekf_check_state.fail_count > 0) {
            ekf_check_state.fail_count--;

            // if compass is flagged as bad and the counter reaches zero then clear flag
            if (ekf_check_state.bad_variance && ekf_check_state.fail_count == 0) {
                ekf_check_state.bad_variance = false;

				//	added by zhangong for EKF VARIANCE TYEP
				ekf_check_state.type = EKF_VARIANCE_NON;
				//	added end
			
                // log recovery in the dataflash
                Log_Write_Error(ERROR_SUBSYSTEM_EKFCHECK, ERROR_CODE_EKFCHECK_VARIANCE_CLEARED);
                // clear failsafe
                failsafe_ekf_off_event();
            }
        }
    }

    // set AP_Notify flags
    AP_Notify::flags.ekf_bad = ekf_check_state.bad_variance;

    // To-Do: add ekf variances to extended status
}

// ekf_over_threshold - returns true if the ekf's variance are over the tolerance
bool Copter::ekf_over_threshold()
{
    // return false immediately if disabled
    if (g.fs_ekf_thresh <= 0.0f) {
        return false;
    }

	
    // return true immediately if position is bad
    if (!ekf_position_ok() && !optflow_position_ok()) {
        return true;
    }

	
    // use EKF to get variance
    float position_variance, vel_variance, height_variance, tas_variance;
    Vector3f mag_variance;
    Vector2f offset;
    ahrs.get_variances(vel_variance, position_variance, height_variance, mag_variance, tas_variance, offset);

    // return true if two of compass, velocity and position variances are over the threshold
    uint8_t over_thresh_count = 0;
    if (mag_variance.length() >= g.fs_ekf_thresh) {
        over_thresh_count++;
		//	added by zhangyong for ekf variance type report
		ekf_check_state.type = EKF_VARIANCE_MAG;
		//	added end
    }
    if (vel_variance >= g.fs_ekf_thresh) {
        over_thresh_count++;
		//	added by zhangyong for ekf variance type report
		ekf_check_state.type = EKF_VARIANCE_VEL;
		//	added end
    }
	
    if (position_variance >= g.fs_ekf_thresh) {
        over_thresh_count++;
		//	added by zhangyong for ekf variance type report
		ekf_check_state.type = EKF_VARIANCE_POS;
		//	added end
    }

	
    return (over_thresh_count >= 2);
}


// failsafe_ekf_event - perform ekf failsafe
void Copter::failsafe_ekf_event()
{
    // return immediately if ekf failsafe already triggered
    if (failsafe.ekf) {
        return;
    }

    // do nothing if motors disarmed
    if (!motors->armed()) {
        return;
    }

    // do nothing if not in GPS flight mode and ekf-action is not land-even-stabilize
    if ((control_mode != LAND) && !mode_requires_GPS(control_mode) && (g.fs_ekf_action != FS_EKF_ACTION_LAND_EVEN_STABILIZE)) {
        return;
    }

    // EKF failsafe event has occurred
    failsafe.ekf = true;
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_EKFINAV, ERROR_CODE_FAILSAFE_OCCURRED);

    // take action based on fs_ekf_action parameter
    switch (g.fs_ekf_action) {
        case FS_EKF_ACTION_ALTHOLD:
            // AltHold
            if (failsafe.radio || !set_mode(ALT_HOLD, MODE_REASON_EKF_FAILSAFE)) {
                set_mode_land_with_pause(MODE_REASON_EKF_FAILSAFE);
            }
            break;
        default:
            set_mode_land_with_pause(MODE_REASON_EKF_FAILSAFE);
            break;
    }

    // if flight mode is already LAND ensure it's not the GPS controlled LAND
    if (control_mode == LAND) {
        land_do_not_use_GPS();
    }
}

// failsafe_ekf_off_event - actions to take when EKF failsafe is cleared
void Copter::failsafe_ekf_off_event(void)
{
    // return immediately if not in ekf failsafe
    if (!failsafe.ekf) {
        return;
    }

    // clear flag and log recovery
    failsafe.ekf = false;
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_EKFINAV, ERROR_CODE_FAILSAFE_RESOLVED);
}

// check for ekf yaw reset and adjust target heading, also log position reset
void Copter::check_ekf_reset()
{
    // check for yaw reset
    float yaw_angle_change_rad = 0.0f;
    uint32_t new_ekfYawReset_ms = ahrs.getLastYawResetAngle(yaw_angle_change_rad);
    if (new_ekfYawReset_ms != ekfYawReset_ms) {
        attitude_control->shift_ef_yaw_target(ToDeg(yaw_angle_change_rad) * 100.0f);
        ekfYawReset_ms = new_ekfYawReset_ms;
        Log_Write_Event(DATA_EKF_YAW_RESET);
    }

#if AP_AHRS_NAVEKF_AVAILABLE
    // check for change in primary EKF (log only, AC_WPNav handles position target adjustment)
    if ((EKF2.getPrimaryCoreIndex() != ekf_primary_core) && (EKF2.getPrimaryCoreIndex() != -1)) {
        ekf_primary_core = EKF2.getPrimaryCoreIndex();
        Log_Write_Error(ERROR_SUBSYSTEM_EKF_PRIMARY, ekf_primary_core);
        gcs().send_text(MAV_SEVERITY_WARNING, "EKF primary changed:%d\n", (unsigned)ekf_primary_core);
    }
#endif
}
