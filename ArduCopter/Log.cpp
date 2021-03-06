#include "Copter.h"
#include "version.h"

#if LOGGING_ENABLED == ENABLED

// Code to Write and Read packets from DataFlash log memory
// Code to interact with the user to dump or erase logs

#if AUTOTUNE_ENABLED == ENABLED
struct PACKED log_AutoTune {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t axis;           // roll or pitch
    uint8_t tune_step;      // tuning PI or D up or down
    float   meas_target;    // target achieved rotation rate
    float   meas_min;       // maximum achieved rotation rate
    float   meas_max;       // maximum achieved rotation rate
    float   new_gain_rp;    // newly calculated gain
    float   new_gain_rd;    // newly calculated gain
    float   new_gain_sp;    // newly calculated gain
    float   new_ddt;        // newly calculated gain
};

// Write an Autotune data packet
void Copter::Log_Write_AutoTune(uint8_t axis, uint8_t tune_step, float meas_target, float meas_min, float meas_max, float new_gain_rp, float new_gain_rd, float new_gain_sp, float new_ddt)
{
    struct log_AutoTune pkt = {
        LOG_PACKET_HEADER_INIT(LOG_AUTOTUNE_MSG),
        time_us     : AP_HAL::micros64(),
        axis        : axis,
        tune_step   : tune_step,
        meas_target : meas_target,
        meas_min    : meas_min,
        meas_max    : meas_max,
        new_gain_rp : new_gain_rp,
        new_gain_rd : new_gain_rd,
        new_gain_sp : new_gain_sp,
        new_ddt     : new_ddt
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_AutoTuneDetails {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float    angle_cd;      // lean angle in centi-degrees
    float    rate_cds;      // current rotation rate in centi-degrees / second
};

// Write an Autotune data packet
void Copter::Log_Write_AutoTuneDetails(float angle_cd, float rate_cds)
{
    struct log_AutoTuneDetails pkt = {
        LOG_PACKET_HEADER_INIT(LOG_AUTOTUNEDETAILS_MSG),
        time_us     : AP_HAL::micros64(),
        angle_cd    : angle_cd,
        rate_cds    : rate_cds
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}
#endif


#if SPRAYER == ENABLED
//	added by ZhangYong 20170405
void Copter::Log_Write_Sprayer(AC_Sprayer &para_sprayer, uint32_t wp_dist, uint8_t para_fm_warn, uint8_t para_pk_cnt, uint16_t para_fm_vol, uint16_t para_fm_high, uint16_t view_flight_area)
{
	DataFlash.Log_Write_Sprayer(para_sprayer, wp_dist, para_fm_warn, para_pk_cnt, para_fm_vol, para_fm_high, view_flight_area);
}


//	added end
#endif


// Write a Current data packet
void Copter::Log_Write_Current()
{
    DataFlash.Log_Write_Current(battery);

    // also write power status
    DataFlash.Log_Write_Power();
}

struct PACKED log_Optflow {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t surface_quality;
    float flow_x;
    float flow_y;
    float body_x;
    float body_y;
};

// Write an optical flow packet
void Copter::Log_Write_Optflow()
{
 #if OPTFLOW == ENABLED
    // exit immediately if not enabled
    if (!optflow.enabled()) {
        return;
    }
    const Vector2f &flowRate = optflow.flowRate();
    const Vector2f &bodyRate = optflow.bodyRate();
    struct log_Optflow pkt = {
        LOG_PACKET_HEADER_INIT(LOG_OPTFLOW_MSG),
        time_us         : AP_HAL::micros64(),
        surface_quality : optflow.quality(),
        flow_x          : flowRate.x,
        flow_y          : flowRate.y,
        body_x          : bodyRate.x,
        body_y          : bodyRate.y
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
 #endif     // OPTFLOW == ENABLED
}

struct PACKED log_Nav_Tuning {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float    desired_pos_x;
    float    desired_pos_y;
    float    pos_x;
    float    pos_y;
    float    desired_vel_x;
    float    desired_vel_y;
    float    vel_x;
    float    vel_y;
    float    desired_accel_x;
    float    desired_accel_y;
	float  	 accel_x;
	float 	 accel_y;
	//	added by ZhangYong for avoidance use
	float	 vel_bf_x;
	float	 vel_bf_y;
	//	added end
};

// Write an Nav Tuning packet
void Copter::Log_Write_Nav_Tuning()
{
    const Vector3f &pos_target = pos_control->get_pos_target();
    const Vector3f &vel_target = pos_control->get_vel_target();
    const Vector3f &accel_target = pos_control->get_accel_target();
    const Vector3f &position = inertial_nav.get_position();
    const Vector3f &velocity = inertial_nav.get_velocity();
	const Vector3f &accel = ahrs.get_accel_ef_blended();
	
	

	//	added by ZhangYong 20180109 for avoidance 
	Vector3f &velocity_bf = inertial_nav.get_velocity_bf();
	//	added end

	/*	modified by ZhangYong 20180109 for avoidance usage
	struct log_Nav_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_NAV_TUNING_MSG),
        time_us         : AP_HAL::micros64(),
        desired_pos_x   : pos_target.x,
        desired_pos_y   : pos_target.y,
        pos_x           : position.x,
        pos_y           : position.y,
        desired_vel_x   : vel_target.x,
        desired_vel_y   : vel_target.y,
        vel_x           : velocity.x,
        vel_y           : velocity.y,
        desired_accel_x : accel_target.x,
        desired_accel_y : accel_target.y
    };
	*/
	

    struct log_Nav_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_NAV_TUNING_MSG),
        time_us         : AP_HAL::micros64(),
        desired_pos_x   : pos_target.x,
        desired_pos_y   : pos_target.y,
        pos_x           : position.x,
        pos_y           : position.y,
        desired_vel_x   : vel_target.x,
        desired_vel_y   : vel_target.y,
        vel_x           : velocity.x,
        vel_y           : velocity.y,
        desired_accel_x : accel_target.x,
        desired_accel_y : accel_target.y,
        accel_x			: accel.x * 100,
        accel_y			: accel.y * 100,
        vel_bf_x		: velocity_bf.x,
        vel_bf_y		: velocity_bf.y
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Control_Tuning {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float    throttle_in;
    float    angle_boost;
    float    throttle_out;
    float    throttle_hover;
    float    desired_alt;
    float    inav_alt;
    int32_t  baro_alt;
    int16_t  desired_rangefinder_alt;
    int16_t  rangefinder_alt;
    float    terr_alt;
    int16_t  target_climb_rate;
    int16_t  climb_rate;
	float	 rangefinder_climb_rate;
	//	modified by ZhangYong 20180111
	int32_t  mavlink_id33_rel_alt;
	//	modified end
};

// Write a control tuning packet
void Copter::Log_Write_Control_Tuning()
{
    // get terrain altitude
    float terr_alt = 0.0f;

	//	added by zhangyong for DSAT log in different flight mode
	int16_t desired_terr_alt = 0;
	//	added end
	
#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
    if (terrain.height_above_terrain(terr_alt, true)) {
        terr_alt = 0.0f;
    }
#endif

	//	added by zhangyong for DSAT log in different flight mode
	if((AUTO == control_mode) || (GUIDED == control_mode))
	{
		if(NULL != wp_nav)
			desired_terr_alt = (int16_t)wp_nav->get_wp_destination_z();
	}
	else
	{
		desired_terr_alt = target_rangefinder_alt;
	}
	//	added end


	/*
    struct log_Control_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CONTROL_TUNING_MSG),
        time_us             	: AP_HAL::micros64(),
        throttle_in         	: attitude_control->get_throttle_in(),
        angle_boost         	: attitude_control->angle_boost(),
        throttle_out        	: motors->get_throttle(),
        throttle_hover      	: motors->get_throttle_hover(),
        desired_alt         	: pos_control->get_alt_target() / 100.0f,
        inav_alt            	: inertial_nav.get_altitude() / 100.0f,
        baro_alt            	: baro_alt,
        desired_rangefinder_alt : (int16_t)target_rangefinder_alt,
        rangefinder_alt     	: rangefinder_state.alt_cm,
        terr_alt            	: terr_alt,
        target_climb_rate   	: (int16_t)pos_control->get_vel_target_z(),
        climb_rate          	: climb_rate,
//		added by ZhangYong 20180111
		rangefinder_climb_rate 	: surface_tracking_climb_rate,
		mavlink_id33_rel_alt	: current_loc.alt
//		added end
    };*/
//		mavlink_id33_rel_alt	: (int32_t)(inertial_nav.get_altitude() * 10)


	 struct log_Control_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CONTROL_TUNING_MSG),
        time_us             	: AP_HAL::micros64(),
        throttle_in         	: attitude_control->get_throttle_in(),
        angle_boost         	: attitude_control->angle_boost(),
        throttle_out        	: motors->get_throttle(),
        throttle_hover      	: motors->get_throttle_hover(),
        desired_alt         	: pos_control->get_alt_target() / 100.0f,
        inav_alt            	: inertial_nav.get_altitude() / 100.0f,
        baro_alt            	: baro_alt,
        desired_rangefinder_alt : desired_terr_alt,
        rangefinder_alt     	: rangefinder_state.alt_cm,
        terr_alt            	: terr_alt,
        target_climb_rate   	: (int16_t)pos_control->get_vel_target_z(),
        climb_rate          	: climb_rate,
//		added by ZhangYong 20180111
		rangefinder_climb_rate 	: surface_tracking_climb_rate,
		mavlink_id33_rel_alt	: current_loc.alt
//		added end
    };
//		mavlink_id33_rel_alt	: (int32_t)(inertial_nav.get_altitude() * 10)
    DataFlash.WriteBlock(&pkt, sizeof(pkt));

//	added by zhangyong 20180115 alt_error
//	printf("Log_Write_Control_Tuning m:%4.2f\n", (inertial_nav.get_altitude() / 100.0f));	
//	added end
	
}

struct PACKED log_Performance {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint16_t num_long_running;
    uint16_t num_loops;
    uint32_t max_time;
    int16_t  pm_test;
    uint8_t i2c_lockup_count;
    uint16_t ins_error_count;
    uint32_t log_dropped;
    uint32_t mem_avail;
};

// Write a performance monitoring packet
void Copter::Log_Write_Performance()
{
    struct log_Performance pkt = {
        LOG_PACKET_HEADER_INIT(LOG_PERFORMANCE_MSG),
        time_us          : AP_HAL::micros64(),
        num_long_running : perf_info_get_num_long_running(),
        num_loops        : perf_info_get_num_loops(),
        max_time         : perf_info_get_max_time(),
        pm_test          : pmTest1,
        i2c_lockup_count : 0,
        ins_error_count  : ins.error_count(),
        log_dropped      : DataFlash.num_dropped() - perf_info_get_num_dropped(),
        hal.util->available_memory()
    };
    DataFlash.WriteCriticalBlock(&pkt, sizeof(pkt));
}

// Write an attitude packet
void Copter::Log_Write_Attitude()
{
    Vector3f targets = attitude_control->get_att_target_euler_cd();
    targets.z = wrap_360_cd(targets.z);
    DataFlash.Log_Write_Attitude(ahrs, targets);
    DataFlash.Log_Write_Rate(ahrs, *motors, *attitude_control, *pos_control);
    if (should_log(MASK_LOG_PID)) {
		//	shielded by ZhangYong 20170629
        //DataFlash.Log_Write_PID(LOG_PIDR_MSG, attitude_control->get_rate_roll_pid().get_pid_info());
        //DataFlash.Log_Write_PID(LOG_PIDP_MSG, attitude_control->get_rate_pitch_pid().get_pid_info());
        //DataFlash.Log_Write_PID(LOG_PIDY_MSG, attitude_control->get_rate_yaw_pid().get_pid_info());
		//	shielded end
		DataFlash.Log_Write_PID(LOG_PIDA_MSG, g.pid_accel_z.get_pid_info() );
    }
}

// Write an EKF and POS packet
void Copter::Log_Write_EKF_POS()
{
 #if OPTFLOW == ENABLED
    DataFlash.Log_Write_EKF(ahrs,optflow.enabled());
 #else
 	//	modified by ZhangYong 20170915
 	//	modified end
    DataFlash.Log_Write_EKF(ahrs,false);
 #endif
    DataFlash.Log_Write_AHRS2(ahrs);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    sitl.Log_Write_SIMSTATE(&DataFlash);
#endif
    DataFlash.Log_Write_POS(ahrs);
}

struct PACKED log_MotBatt {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float   lift_max;
    float   bat_volt;
    float   bat_res;
    float   th_limit;
};

// Write an rate packet
void Copter::Log_Write_MotBatt()
{
#if FRAME_CONFIG != HELI_FRAME
    struct log_MotBatt pkt_mot = {
        LOG_PACKET_HEADER_INIT(LOG_MOTBATT_MSG),
        time_us         : AP_HAL::micros64(),
        lift_max        : (float)(motors->get_lift_max()),
        bat_volt        : (float)(motors->get_batt_voltage_filt()),
        bat_res         : (float)(motors->get_batt_resistance()),
        th_limit        : (float)(motors->get_throttle_limit())
    };
    DataFlash.WriteBlock(&pkt_mot, sizeof(pkt_mot));
#endif
}

struct PACKED log_Event {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
};

// Wrote an event packet
void Copter::Log_Write_Event(uint8_t id)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Event pkt = {
            LOG_PACKET_HEADER_INIT(LOG_EVENT_MSG),
            time_us  : AP_HAL::micros64(),
            id       : id
        };
        DataFlash.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_Int16t {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
    int16_t data_value;
};

// Write an int16_t data packet
UNUSED_FUNCTION
void Copter::Log_Write_Data(uint8_t id, int16_t value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_Int16t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_INT16_MSG),
            time_us     : AP_HAL::micros64(),
            id          : id,
            data_value  : value
        };
        DataFlash.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_UInt16t {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
    uint16_t data_value;
};

// Write an uint16_t data packet
UNUSED_FUNCTION 
void Copter::Log_Write_Data(uint8_t id, uint16_t value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_UInt16t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_UINT16_MSG),
            time_us     : AP_HAL::micros64(),
            id          : id,
            data_value  : value
        };
        DataFlash.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_Int32t {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
    int32_t data_value;
};

// Write an int32_t data packet
void Copter::Log_Write_Data(uint8_t id, int32_t value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_Int32t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_INT32_MSG),
            time_us  : AP_HAL::micros64(),
            id          : id,
            data_value  : value
        };
        DataFlash.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_UInt32t {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
    uint32_t data_value;
};

// Write a uint32_t data packet
void Copter::Log_Write_Data(uint8_t id, uint32_t value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_UInt32t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_UINT32_MSG),
            time_us     : AP_HAL::micros64(),
            id          : id,
            data_value  : value
        };
        DataFlash.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_Float {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
    float data_value;
};

// Write a float data packet
UNUSED_FUNCTION
void Copter::Log_Write_Data(uint8_t id, float value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_Float pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_FLOAT_MSG),
            time_us     : AP_HAL::micros64(),
            id          : id,
            data_value  : value
        };
        DataFlash.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Error {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t sub_system;
    uint8_t error_code;
};

// Write an error packet
void Copter::Log_Write_Error(uint8_t sub_system, uint8_t error_code)
{
    struct log_Error pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ERROR_MSG),
        time_us       : AP_HAL::micros64(),
        sub_system    : sub_system,
        error_code    : error_code,
    };
    DataFlash.WriteCriticalBlock(&pkt, sizeof(pkt));
}

void Copter::Log_Write_Baro(void)
{
    if (!ahrs.have_ekf_logging()) {
        DataFlash.Log_Write_Baro(barometer);
    }
}

struct PACKED log_ParameterTuning {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  parameter;     // parameter we are tuning, e.g. 39 is CH6_CIRCLE_RATE
    float    tuning_value;  // normalized value used inside tuning() function
    int16_t  control_in;    // raw tune input value
    int16_t  tuning_low;    // tuning low end value
    int16_t  tuning_high;   // tuning high end value
};

void Copter::Log_Write_Parameter_Tuning(uint8_t param, float tuning_val, int16_t control_in, int16_t tune_low, int16_t tune_high)
{
    struct log_ParameterTuning pkt_tune = {
        LOG_PACKET_HEADER_INIT(LOG_PARAMTUNE_MSG),
        time_us        : AP_HAL::micros64(),
        parameter      : param,
        tuning_value   : tuning_val,
        control_in     : control_in,
        tuning_low     : tune_low,
        tuning_high    : tune_high
    };

    DataFlash.WriteBlock(&pkt_tune, sizeof(pkt_tune));
}

// log EKF origin and ahrs home to dataflash
void Copter::Log_Write_Home_And_Origin()
{
    // log ekf origin if set
    Location ekf_orig;
    if (ahrs.get_origin(ekf_orig)) {
        DataFlash.Log_Write_Origin(LogOriginType::ekf_origin, ekf_orig);
    }

    // log ahrs home if set
    if (ap.home_state != HOME_UNSET) {
        DataFlash.Log_Write_Origin(LogOriginType::ahrs_home, ahrs.get_home());
    }
}

// logs when baro or compass becomes unhealthy
void Copter::Log_Sensor_Health()
{
    // check baro
    if (sensor_health.baro != barometer.healthy()) {
        sensor_health.baro = barometer.healthy();
        Log_Write_Error(ERROR_SUBSYSTEM_BARO, (sensor_health.baro ? ERROR_CODE_ERROR_RESOLVED : ERROR_CODE_UNHEALTHY));
    }

    // check compass
    if (sensor_health.compass != compass.healthy()) {
        sensor_health.compass = compass.healthy();
        Log_Write_Error(ERROR_SUBSYSTEM_COMPASS, (sensor_health.compass ? ERROR_CODE_ERROR_RESOLVED : ERROR_CODE_UNHEALTHY));
    }

    // check primary GPS
    if (sensor_health.primary_gps != gps.primary_sensor()) {
        sensor_health.primary_gps = gps.primary_sensor();
        Log_Write_Event(DATA_GPS_PRIMARY_CHANGED);
    }
}

struct PACKED log_Heli {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float    desired_rotor_speed;
    float    main_rotor_speed;
};

#if FRAME_CONFIG == HELI_FRAME
// Write an helicopter packet
void Copter::Log_Write_Heli()
{
    struct log_Heli pkt_heli = {
        LOG_PACKET_HEADER_INIT(LOG_HELI_MSG),
        time_us                 : AP_HAL::micros64(),
        desired_rotor_speed     : motors->get_desired_rotor_speed(),
        main_rotor_speed        : motors->get_main_rotor_speed(),
    };
    DataFlash.WriteBlock(&pkt_heli, sizeof(pkt_heli));
}
#endif

// precision landing logging
struct PACKED log_Precland {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t healthy;
    uint8_t target_acquired;
    float pos_x;
    float pos_y;
    float vel_x;
    float vel_y;
};

// Write an optical flow packet
void Copter::Log_Write_Precland()
{
 #if PRECISION_LANDING == ENABLED
    // exit immediately if not enabled
    if (!precland.enabled()) {
        return;
    }

    Vector2f target_pos_rel = Vector2f(0.0f,0.0f);
    Vector2f target_vel_rel = Vector2f(0.0f,0.0f);
    precland.get_target_position_relative_cm(target_pos_rel);
    precland.get_target_velocity_relative_cms(target_vel_rel);

    struct log_Precland pkt = {
        LOG_PACKET_HEADER_INIT(LOG_PRECLAND_MSG),
        time_us         : AP_HAL::micros64(),
        healthy         : precland.healthy(),
        target_acquired : precland.target_acquired(),
        pos_x           : target_pos_rel.x,
        pos_y           : target_pos_rel.y,
        vel_x           : target_vel_rel.x,
        vel_y           : target_vel_rel.y
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
 #endif     // PRECISION_LANDING == ENABLED
}

// precision landing logging
struct PACKED log_GuidedTarget {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t type;
    float pos_target_x;
    float pos_target_y;
    float pos_target_z;
    float vel_target_x;
    float vel_target_y;
    float vel_target_z;
	uint8_t options;
};

// Write a Guided mode target
void Copter::Log_Write_GuidedTarget(uint8_t target_type, const Vector3f& pos_target, const Vector3f& vel_target, uint8_t fp_options)
{
    struct log_GuidedTarget pkt = {
        LOG_PACKET_HEADER_INIT(LOG_GUIDEDTARGET_MSG),
        time_us         : AP_HAL::micros64(),
        type            : target_type,
        pos_target_x    : pos_target.x,
        pos_target_y    : pos_target.y,
        pos_target_z    : pos_target.z,
        vel_target_x    : vel_target.x,
        vel_target_y    : vel_target.y,
        vel_target_z    : vel_target.z,
        options			: fp_options,
        
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// precision landing logging
struct PACKED log_Throw {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t stage;
    float velocity;
    float velocity_z;
    float accel;
    float ef_accel_z;
    uint8_t throw_detect;
    uint8_t attitude_ok;
    uint8_t height_ok;
    uint8_t pos_ok;
};

// Write a Throw mode details
void Copter::Log_Write_Throw(ThrowModeStage stage, float velocity, float velocity_z, float accel, float ef_accel_z, bool throw_detect, bool attitude_ok, bool height_ok, bool pos_ok)
{
    struct log_Throw pkt = {
        LOG_PACKET_HEADER_INIT(LOG_THROW_MSG),
        time_us         : AP_HAL::micros64(),
        stage           : (uint8_t)stage,
        velocity        : velocity,
        velocity_z      : velocity_z,
        accel           : accel,
        ef_accel_z      : ef_accel_z,
        throw_detect    : throw_detect,
        attitude_ok     : attitude_ok,
        height_ok       : height_ok,
        pos_ok          : pos_ok
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// Write proximity sensor distances
void Copter::Log_Write_Proximity()
{
#if PROXIMITY_ENABLED == ENABLED
    DataFlash.Log_Write_Proximity(g2.proximity);
    DataFlash.Log_Write_GKProx(g2.proximity);
#endif
}

// Write beacon position and distances
void Copter::Log_Write_Beacon()
{
    // exit immediately if feature is disabled
    if (!g2.beacon.enabled()) {
        return;
    }
    DataFlash.Log_Write_Beacon(g2.beacon);
}

void Copter::Log_Write_land_detector(struct Log_Land_Detector *fp_log_land_detector)
{
	//printf("Copter::Log_Write_land_detector\n");
	DataFlash.Log_Write_land_detector(fp_log_land_detector->log_land_complete, \
														fp_log_land_detector->log_land_complete_maybe, \
														fp_log_land_detector->log_motors_limit_throttle_lower, \
														fp_log_land_detector->log_att_is_throttle_mix_min, \
														fp_log_land_detector->log_land_accel_ef_filter_length, \
														fp_log_land_detector->log_descent_rate_low, \
														fp_log_land_detector->log_rangefinder_check);
}


struct PACKED log_DN_Tunning {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float log_daccelx;
	float log_daccely;
	uint8_t log_ldaccel;
	float log_ldaccelx;
	float log_ldaccely;
	float log_dvelx;
	float log_dvely;
	float log_drag_speed;
	float log_accel_min;
	float log_desired_2speed;
	float log_dvelbx;
	float log_dvelby;
	float log_dvelmx;
	float log_dvelmy;
};



//	added by zhangyong for desired loiter velocity analyse 20180918
void Copter::Log_Write_desired_navigation()
{
	DN_Tunning *lcl_dn_tunning = wp_nav->get_dn_tunning();

	struct log_DN_Tunning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_DN_MSG),
        time_us       		: AP_HAL::micros64(),					//	1
        log_daccelx   		: lcl_dn_tunning->daccelx,				//	2
        log_daccely   		: lcl_dn_tunning->daccely,				//	3
        log_ldaccel			: lcl_dn_tunning->ldaccel,				//	4
        log_ldaccelx  		: lcl_dn_tunning->ldaccelx,				//	5
        log_ldaccely  		: lcl_dn_tunning->ldaccely,				//	6
        log_dvelx   		: lcl_dn_tunning->dvelx,				//	7
        log_dvely   		: lcl_dn_tunning->dvely,				//	8
		log_drag_speed		: lcl_dn_tunning->drag_speed,			//	9
        log_accel_min		: lcl_dn_tunning->accel_min,			//	10
        log_desired_2speed	: lcl_dn_tunning->desired_2speed,		//	11
        log_dvelbx      	: lcl_dn_tunning->dvelbx,				//	12
        log_dvelby      	: lcl_dn_tunning->dvelby,				//	13
        log_dvelmx 			: lcl_dn_tunning->dvelmx,				//	14
        log_dvelmy 			: lcl_dn_tunning->dvelmy				//	15
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
	
}
//	added end



//	added by ZhangYong
#if BCBPMBUS == ENABLED
void Copter::Log_Write_BCBPMBus_Msg(uint8_t msg_type)
{
	uint8_t lcl_cnt;

    // exit immediately if feature is disabled
    if (!g2.bcbpmbus.enabled()) {
        return;
    }

	if(!g2.bcbpmbus.initialised())
	{
		return;
	}

	switch(msg_type)
	{
		case 1:
			for(lcl_cnt = 0; lcl_cnt < AC_BCBPMBUS_MODULE_MAX_COMPONENT; lcl_cnt++)
			{
				if(true == g2.bcbpmbus.should_log_module_slot_info(lcl_cnt))
				{
					//printf("AC_BCBPMBUS_MSG_ID_MODULES\n");
					DataFlash.Log_Write_BCBPMBus_Modules(lcl_cnt + LOG_PMBUS_MOD0_MSG, g2.bcbpmbus);
					g2.bcbpmbus.module_slot_info_logged(lcl_cnt);
				}
			}
		break;

		case 2:
			if(true == g2.bcbpmbus.should_log_voltages_info())
			{
//				printf("AC_BCBPMBUS_MSG_ID_VOLTAGES\n");
				DataFlash.Log_Write_BCBPMBus_Voltages(g2.bcbpmbus);
				g2.bcbpmbus.voltages_info_logged();
			}
			
			break;

		default:
			break;
	}

	
    
}
#endif


//	added end

//		modified by zhangyong 20180109 for avoidance usage
///      "NTUN", "Qffffffffff", "TUS,DPosX,DPosY,PosX,PosY,DVelX,DVelY,VelX,VelY,DAccX,DAccY" },
//	"CTUN", "Qffffffeccfhh", "TimeUS,ThI,ABst,ThO,ThH,DAt,At,BAt,DSAt,SAt,TAt,DCRt,CRt" },


const struct LogStructure Copter::log_structure[] = {
    LOG_COMMON_STRUCTURES,
#if AUTOTUNE_ENABLED == ENABLED
    { LOG_AUTOTUNE_MSG, sizeof(log_AutoTune),
      "ATUN", "QBBfffffff",       "TimeUS,Axis,TuneStep,Targ,Min,Max,RP,RD,SP,ddt" },
    { LOG_AUTOTUNEDETAILS_MSG, sizeof(log_AutoTuneDetails),
      "ATDE", "Qff",          "TimeUS,Angle,Rate" },
#endif
    { LOG_PARAMTUNE_MSG, sizeof(log_ParameterTuning),
      "PTUN", "QBfHHH",          "TimeUS,Param,TunVal,CtrlIn,TunLo,TunHi" },  
    { LOG_OPTFLOW_MSG, sizeof(log_Optflow),
      "OF",   "QBffff",   "TimeUS,Qual,flowX,flowY,bodyX,bodyY" },
    { LOG_NAV_TUNING_MSG, sizeof(log_Nav_Tuning),
      "NTUN", "Qffffffffffffff", "TUS,DPX,DPY,PX,PY,DVX,DVY,VX,VY,DAX,DAY,AX,AY,VBX,VBY" },
    { LOG_CONTROL_TUNING_MSG, sizeof(log_Control_Tuning),
      "CTUN", "Qffffffeccfhhfi", "TimeUS,TI,ABst,TO,TH,DAt,At,BAt,DSAt,SAt,TAt,DCRt,CRt,SCRt,MAt" },
    { LOG_PERFORMANCE_MSG, sizeof(log_Performance),
      "PM",  "QHHIhBHII",    "TimeUS,NLon,NLoop,MaxT,PMT,I2CErr,INSErr,LogDrop,Mem" },
    { LOG_MOTBATT_MSG, sizeof(log_MotBatt),
      "MOTB", "Qffff",  "TimeUS,LiftMax,BatVolt,BatRes,ThLimit" },
    { LOG_EVENT_MSG, sizeof(log_Event),
      "EV",   "QB",           "TimeUS,Id" },
    { LOG_DATA_INT16_MSG, sizeof(log_Data_Int16t),
      "D16",   "QBh",         "TimeUS,Id,Value" },
    { LOG_DATA_UINT16_MSG, sizeof(log_Data_UInt16t),
      "DU16",  "QBH",         "TimeUS,Id,Value" },
    { LOG_DATA_INT32_MSG, sizeof(log_Data_Int32t),         
      "D32",   "QBi",         "TimeUS,Id,Value" },
    { LOG_DATA_UINT32_MSG, sizeof(log_Data_UInt32t),         
      "DU32",  "QBI",         "TimeUS,Id,Value" },
    { LOG_DATA_FLOAT_MSG, sizeof(log_Data_Float),         
      "DFLT",  "QBf",         "TimeUS,Id,Value" },
    { LOG_ERROR_MSG, sizeof(log_Error),         
      "ERR",   "QBB",         "TimeUS,Subsys,ECode" },
    { LOG_HELI_MSG, sizeof(log_Heli),
      "HELI",  "Qff",         "TimeUS,DRRPM,ERRPM" },
    { LOG_PRECLAND_MSG, sizeof(log_Precland),
      "PL",    "QBBffff",    "TimeUS,Heal,TAcq,pX,pY,vX,vY" },
    { LOG_GUIDEDTARGET_MSG, sizeof(log_GuidedTarget),
      "GUID",  "QBffffffb",    "TimeUS,Type,pX,pY,pZ,vX,vY,vZ,op" },
    { LOG_THROW_MSG, sizeof(log_Throw),
      "THRO",  "QBffffbbbb",  "TimeUS,Stage,Vel,VelZ,Acc,AccEfZ,Throw,AttOk,HgtOk,PosOk" },
};

void Copter::Log_Write_Vehicle_Startup_Messages()
{
    // only 200(?) bytes are guaranteed by DataFlash
    DataFlash.Log_Write_MessageF("Frame: %s", get_frame_string());
    DataFlash.Log_Write_Mode(control_mode, control_mode_reason);
#if AC_RALLY
    DataFlash.Log_Write_Rally(rally);
#endif
    Log_Write_Home_And_Origin();
    gps.Write_DataFlash_Log_Startup_messages();
}


void Copter::log_init(void)
{
    DataFlash.Init(log_structure, ARRAY_SIZE(log_structure));
}

#else // LOGGING_ENABLED

void Copter::Log_Write_AutoTune(uint8_t axis, uint8_t tune_step, float meas_target, \
                                float meas_min, float meas_max, float new_gain_rp, \
                                float new_gain_rd, float new_gain_sp, float new_ddt) {}
void Copter::Log_Write_AutoTuneDetails(float angle_cd, float rate_cds) {}
#if SPRAYER == ENABLED
//	added by ZhangYong 20170405
void Copter::Log_Write_Sprayer(AC_Sprayer &para_sprayer, uint32_t wp_dist, uint8_t para_fm_warn, uint8_t para_pk_cnt, uint16_t view_flight_area) {}


//	added end
#endif
void Copter::Log_Write_Current() {}
void Copter::Log_Write_Nav_Tuning() {}
void Copter::Log_Write_Control_Tuning() {}
void Copter::Log_Write_Performance() {}
void Copter::Log_Write_Attitude(void) {}
void Copter::Log_Write_EKF_POS() {}
void Copter::Log_Write_MotBatt() {}
void Copter::Log_Write_Event(uint8_t id) {}
void Copter::Log_Write_Data(uint8_t id, int32_t value) {}
void Copter::Log_Write_Data(uint8_t id, uint32_t value) {}
void Copter::Log_Write_Data(uint8_t id, int16_t value) {}
void Copter::Log_Write_Data(uint8_t id, uint16_t value) {}
void Copter::Log_Write_Data(uint8_t id, float value) {}
void Copter::Log_Write_Error(uint8_t sub_system, uint8_t error_code) {}
void Copter::Log_Write_Baro(void) {}
void Copter::Log_Write_Parameter_Tuning(uint8_t param, float tuning_val, int16_t control_in, int16_t tune_low, int16_t tune_high) {}
void Copter::Log_Write_Home_And_Origin() {}
void Copter::Log_Sensor_Health() {}
void Copter::Log_Write_Precland() {}
void Copter::Log_Write_GuidedTarget(uint8_t target_type, const Vector3f& pos_target, const Vector3f& vel_target, uint8_t fp_options) {}
void Copter::Log_Write_Throw(ThrowModeStage stage, float velocity, float velocity_z, float accel, float ef_accel_z, bool throw_detect, bool attitude_ok, bool height_ok, bool pos_ok) {}
void Copter::Log_Write_Proximity() {}
void Copter::Log_Write_Beacon() {}
void Copter::Log_Write_land_detector(struct Log_Land_Detector *fp_log_land_detector) {}
//	added by zhangyong for desired navigation 20180918
void Copter::Log_Write_desired_navigation();
//	added end
void Copter::Log_Write_Vehicle_Startup_Messages() {}

#if FRAME_CONFIG == HELI_FRAME
void Copter::Log_Write_Heli() {}
#endif

#if OPTFLOW == ENABLED
void Copter::Log_Write_Optflow() {}
#endif

#if BCBPMBUS == ENABLED
void Copter::Log_Write_BCBPMBus_Msg(uint8_t msg_type) {}
#endif

void Copter::start_logging() {}


void Copter::log_init(void) {}

#endif // LOGGING_ENABLED
