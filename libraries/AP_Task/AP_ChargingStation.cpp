#include "AP_ChargingStation.h"
#include "./../ArduCopter/Copter.h"
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS.h>

#define BY_DEBUGGING 0

#if BY_DEBUGGING
#include <stdio.h>
#define Debug(fmt, args...)                                                    \
  do {                                                                         \
    ::printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ##args);              \
  } while (0)
#else
#define Debug(fmt, args...)
#endif

#if CHARGINGSTATION == ENABLED

#define TOUT_default 10
#define COMM_REP_default 3
#define REQ_L_REP_default 9
#define READY_L_REP_default 3
#define COMP_L_REP_default 3
#define REQ_F_REP_default 9
#define REA_F_REP_default 5
#define COMP_F_REP_default 3
#define AGA_L_REP_default 3
#define LNG_default 0
#define LAT_default 0
#define COMM_ALT_default 10
#define DISTANCE_default 6
#define STATION_USE_default 1
#define STATION_RANGE_default 100
#define ARM_CHECK_default 0
#define STA_TIME_OUT_default 300
#define COMM_FLY_ALT_default 300

const AP_Param::GroupInfo AP_ChargingStation::var_info[] = {
    // @Param: TOUT
    // @DisplayName: Instruction timeout time
    // @Description: The time interval for each retransmission instruction
    // @unit: s
    AP_GROUPINFO("TOUT", 0, AP_ChargingStation, Timeout, 10),

    // @Param: COMM_REP
    // @DisplayName: Set up communication repetitions
    // @Description: When the connection is established, the number of times the
    // communication is opened
    // @unit: Times
    AP_GROUPINFO("COMM_REP", 1, AP_ChargingStation, comm_repetition,
                 COMM_REP_default),

    // @Param: REQ_L_REP
    // @DisplayName: Request landing repetitions
    // @Description: The number of repetitions when sending a request drop
    // instruction
    // @unit: Times
    AP_GROUPINFO("REQ_L_REP", 2, AP_ChargingStation, Reql_repetition,
                 REQ_L_REP_default),

    // @Param: READY_L_REP
    // @DisplayName: Ready landing repetitions
    // @Description: The number of times to send the landing ready command
    // @unit: Times
    AP_GROUPINFO("READY_L_REP", 3, AP_ChargingStation, Real_repetition,
                 READY_L_REP_default),

    // @Param: COMP_L_REP
    // @DisplayName: complete landed waiting times
    // @Description: This parameter and the timeout parameter together determine
    // the waiting time
    // @unit: Times
    AP_GROUPINFO("COMP_L_REP", 4, AP_ChargingStation, coml_repetition,
                 COMP_L_REP_default),

    // @Param: REQ_F_REP
    // @DisplayName: Request to take off repetitions
    // @Description: The number of times to Request to take off repetitions
    // @unit: Times
    AP_GROUPINFO("REQ_F_REP", 5, AP_ChargingStation, reqf_repetition,
                 REQ_F_REP_default),

    // @Param: REA_F_REP
    // @DisplayName: ready fly complete repetitions
    // @Description: The number of times to send the ready fly complete command
    // @unit: Times
    AP_GROUPINFO("REA_F_REP", 6, AP_ChargingStation, reaf_repetition,
                 REA_F_REP_default),

    // @Param: COMP_F_REP
    // @DisplayName: complete fly waiting times
    // @Description: This parameter and the timeout parameter together determine
    // the waiting time
    // @unit: Times
    AP_GROUPINFO("COMP_F_REP", 7, AP_ChargingStation, comf_repetition,
                 COMP_F_REP_default),

    // @Param: AGA_L_REP
    // @DisplayName: Re-landed repetitions  Takeoff_control
    // @Description: The number of times to send Re-landed command
    // @unit: Times
    AP_GROUPINFO("AGA_L_REP", 8, AP_ChargingStation, agal_repetition,
                 AGA_L_REP_default),

    // @Param: LON
    // @DisplayName: station longitude
    // @Description: Charging base station latitude
    // @unit: degree
    AP_GROUPINFO("LNG_S", 10, AP_ChargingStation, lng_station, LNG_default),

    // @Param: LAT
    // @DisplayName: station latitude
    // @Description: Charging base station longitude
    // @unit: degree
    AP_GROUPINFO("LAT_S", 11, AP_ChargingStation, lat_station, LAT_default),

    // @Param: COMM_ALT
    // @DisplayName: Establish communication height
    // @Description: The aircraft and the base station began to establish the
    // height of the communication
    // @unit: cm
    AP_GROUPINFO("COMM_ALT", 12, AP_ChargingStation, comm_alt,
                 COMM_ALT_default),

    // @Param: DISTANCE
    // @DisplayName: Standby drop point to base station distance
    // @Description: Standby drop point to base station distance,the default
    // orientation is east
    // @unit: cm
    AP_GROUPINFO("DISTANCE", 13, AP_ChargingStation, distance,
                 DISTANCE_default),

    // @Param: DISTANCE
    // @DisplayName: Standby drop point to base station distance
    // @Description: Standby drop point to base station distance,the default
    // orientation is east
    // @unit: cm
    AP_GROUPINFO("STATION_USE", 14, AP_ChargingStation, Bstation_use,
                 STATION_USE_default),

    // @Param: STATION_RANGE
    // @DisplayName: Base station range
    // @Description: Aircraft coordinates more than the base station coordinates
    // STATION_RANGE, that is, the aircraft outside the base station
    // @unit: cm
    AP_GROUPINFO("STA_RANGE", 15, AP_ChargingStation, station_range,
                 STATION_RANGE_default),

    // @Param: ARM_CHECK
    // @DisplayName: Check before arming
    // @Description: Check whether the aircraft is in the base station,1:
    // checking,2: no checking
    // @unit:
    AP_GROUPINFO("ARM_CHECK", 16, AP_ChargingStation, arm_check,
                 ARM_CHECK_default),

    // @Param: STA_TIME_OUT
    // @DisplayName: station time out
    // @Description: Open the protective cover after the takeoff time.Beyond the
    // time, the ground station software must re-send the take-off command
    // @unit: second
    AP_GROUPINFO("STA_TIME_OUT", 17, AP_ChargingStation, sta_time_out,
                 STA_TIME_OUT_default),

    // @Param: COMM_FLY_ALT
    // @DisplayName: complete fly altitude
    // @Description: To determine whether the aircraft has taken off.
    // @unit: cm
    AP_GROUPINFO("COMM_FLY_ALT", 18, AP_ChargingStation, comm_fly_alt,
                 COMM_FLY_ALT_default),

    AP_GROUPEND};

// AP_ChargingStation::AP_ChargingStation(const AP_copter.ahrs &_copter.ahrs,
//                                        const AP_InertialNav &_copter.inertial_nav,
//                                        copter.DataFlash_Class &_copter.DataFlash)
//     : copter.ahrs(_copter.ahrs), copter.inertial_nav(_copter.inertial_nav), copter.DataFlash(_copter.DataFlash) {
//   AP_Param::setup_object_defaults(this, var_info);
//   _initialised = false;
// }


AP_ChargingStation::AP_ChargingStation()
{
  AP_Param::setup_object_defaults(this, var_info);
  _initialised = false;
}

bool AP_ChargingStation::init() {
  _port = copter.serial_manager.find_serial(
      AP_SerialManager::SerialProtocol_ChargingStation, 0);

  if (_port) {
    _num_error.Time_Head_error = 0;
    _num_error.Time_Invalid_data = 0;
    _num_error.Time_Parity_error = 0;
    _num_error.Fail_time = 0;
    _num_error.Timeout_time = 0;
    fli_status = flight_default;
    cmd_station = 0;
    send_cmd = 0;
    repeat = comm_repetition - 1;
    landtimes = 0;
    blastoff_flag = -1;
    timeout = 0;
    state_machine = 0;
    receive_complete_frag = 0;
    sumchkm = 0;
    length = 0;
    init_data();
    back_to_station_midair = false;
    land_alternate_posMidair = false;
    startlanded = false;
    chargingStation_open = false;
    MSG = flight_other;
    receive_takeoff = false;
    repeat_num[0] = comm_repetition - 1;
    repeat_num[1] = Reql_repetition - 1;
    repeat_num[2] = Real_repetition - 1;
    repeat_num[3] = coml_repetition - 1;
    repeat_num[4] = comm_repetition - 1;
    repeat_num[5] = reqf_repetition - 1;
    repeat_num[6] = reaf_repetition - 1;
    repeat_num[7] = comf_repetition - 1;
    repeat_num[8] = agal_repetition - 1;
    for(int i=0;i<512;i++){
    		copter.delay(1);
    		_port->write(0xff);
	  }
    _initialised = true;
    
    return true;
  }
  return false;
}

void AP_ChargingStation::time_check(){
  if(timeout>0){
		timeout--;
    }
}

AP_ChargingStation::~AP_ChargingStation() {}

void AP_ChargingStation::init_station_pos() {
  Location location;

  // Initialize the base station coordinates
  // Stored in eeprom

  //	copter.ahrs.get_position(location);
  location = copter.ahrs.get_home();
  if (location.lat == 0 && location.lng == 0) {
    gcs().send_text(MAV_SEVERITY_ERROR, "Init Pos False");
    return;
  }

  lat_station.set_and_save_ifchanged(location.lat);
  lng_station.set_and_save_ifchanged(location.lng);

  gcs().send_text(MAV_SEVERITY_ERROR, "Init Position");
  /*
          printf("Hlat %d\n",lat_station);
          printf("Hlng %d\n",lng_station);
  */
}

bool AP_ChargingStation::get_flight_permit() {
  if (!arm_check)
    return true;

  Location current_location;
  Location station_location;

  station_location.lat = lat_station.get();
  station_location.lng = lng_station.get();

  if (copter.ahrs.get_gps().status() >= AP_GPS::GPS_OK_FIX_3D)
    current_location = copter.ahrs.get_gps().location();
  else
    return false;

  uint32_t distance_cm =
      get_distance_cm(current_location, station_location); // unit:cm

  if (distance_cm > (uint32_t)station_range.get() || chargingStation_open ||
      !Bstation_use.get())
    return true;
  else
    return false;
}

void AP_ChargingStation::set_cmd(uint8_t type) {
  if (type == YAW) {

    cmd.id = MAV_CMD_CONDITION_YAW;
    cmd.content.yaw.angle_deg =
        0; // target angle in degrees (0 = north, 90 = east)
    cmd.content.yaw.turn_rate_dps =
        0; // turn rate in degrees / second (0 = use default)
    cmd.content.yaw.relative_angle =
        0; // 0 = absolute angle, 1 = relative angle

    if (copter.ahrs.yaw_sensor < 18000)
      cmd.content.yaw.direction = -1; // ccw(Counterclockwise)
    else
      cmd.content.yaw.direction = 1; // cw(Clockwise)
  } else if (type == POSITION) {

    // Set the command to return to the specified height(comm_alt) above the
    // station  Station coordinates(lat_station,lng_station)
    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.content.location.lat = lat_station;
    cmd.content.location.lng = lng_station;
    //		cmd.content.location.alt = comm_alt;  //unit: cm
    cmd.content.location.alt = (int)(copter.inertial_nav.get_position().z);
    cmd.content.location.flags.relative_alt = 1;
  }
}

void AP_ChargingStation::reset_CMD_MSG() {
  CMD_MSG.station_status = 0;
  CMD_MSG.command = 0;
  CMD_MSG.command_status = 0;
  CMD_MSG.command_result = 0;
}

void AP_ChargingStation::set_CMD_MSG(Communication_status _status) {
  switch (_status) {

  case comm_failure:
    CMD_MSG.station_status = 0;
    CMD_MSG.command = 1;
    CMD_MSG.command_status = 1;
    CMD_MSG.command_result = 2;
    break;
  case comm_noresponse:
    CMD_MSG.station_status = 0;
    CMD_MSG.command = 1;
    CMD_MSG.command_status = 1;
    CMD_MSG.command_result = 3;
    break;
  case comm_success:
    CMD_MSG.station_status = 0;
    CMD_MSG.command = 1;
    CMD_MSG.command_status = 2;
    CMD_MSG.command_result = 0;
    break;
  case reqF_failure:
    CMD_MSG.station_status = 0;
    CMD_MSG.command = 1;
    CMD_MSG.command_status = 2;
    CMD_MSG.command_result = 2;
    break;
  case reqF_noresponse:
    CMD_MSG.station_status = 0;
    CMD_MSG.command = 1;
    CMD_MSG.command_status = 2;
    CMD_MSG.command_result = 3;
    break;
  case reqF_success:
    CMD_MSG.station_status = 0;
    CMD_MSG.command = 1;
    CMD_MSG.command_status = 3;
    CMD_MSG.command_result = 0;
    break;
  case reaF_failure:
    CMD_MSG.station_status = 0;
    CMD_MSG.command = 1;
    CMD_MSG.command_status = 3;
    CMD_MSG.command_result = 2;
    break;
  case reaF_noresponse:
    CMD_MSG.station_status = 0;
    CMD_MSG.command = 1;
    CMD_MSG.command_status = 3;
    CMD_MSG.command_result = 3;
    break;
  case reaF_success:
    CMD_MSG.station_status = 1;
    CMD_MSG.command = 1;
    CMD_MSG.command_status = 4;
    CMD_MSG.command_result = 1;
    break;
  case comF_success:
    reset_CMD_MSG();
    break;
  case comF_noresponse:
    reset_CMD_MSG();
    break;
  case reqL_failure:
    CMD_MSG.station_status = 0;
    CMD_MSG.command = 1;
    CMD_MSG.command_status = 2;
    CMD_MSG.command_result = 2;
    break;
  case reqL_noresponse:
    CMD_MSG.station_status = 0;
    CMD_MSG.command = 1;
    CMD_MSG.command_status = 2;
    CMD_MSG.command_result = 3;
    break;
  case reqL_success:
    CMD_MSG.station_status = 0;
    CMD_MSG.command = 1;
    CMD_MSG.command_status = 3;
    CMD_MSG.command_result = 0;
    break;
  case reaL_failure:
    CMD_MSG.station_status = 0;
    CMD_MSG.command = 1;
    CMD_MSG.command_status = 3;
    CMD_MSG.command_result = 2;
    break;
  case reaL_noresponse:
    CMD_MSG.station_status = 0;
    CMD_MSG.command = 1;
    CMD_MSG.command_status = 3;
    CMD_MSG.command_result = 3;
    break;
  case reaL_success:
    CMD_MSG.station_status = 1;
    CMD_MSG.command = 1;
    CMD_MSG.command_status = 4;
    CMD_MSG.command_result = 1;
    break;
  case comL_failure:
    reset_CMD_MSG();
    break;
  case comL_noresponse:
    reset_CMD_MSG();
    break;
  case re_landed:
    //			reset_CMD_MSG();
    break;
  case comL_success:
    reset_CMD_MSG();
    break;
  case flight_other:
  case comm_end:
  case comL_other:
  case comF_other:
  case comm_refuse:
  case reqL_refuse:
  case reqF_refuse:
    break;
  }
}


bool AP_ChargingStation::do_gotostation() {
  if (copter.guided_init(false) && (lat_station.get() != 0) &&
      (lng_station.get() != 0)) {
    set_cmd(POSITION);
    if (copter.wp_nav->set_wp_destination(cmd.content.location)) {
      // Set heading
      set_cmd(YAW);
      copter.do_yaw(cmd);

      back_to_station_midair = true;
      MSG = back_to_station;
      gcs().send_text(MAV_SEVERITY_ERROR,
                                       "AUTO: the base station");
      return true;
    } else {
      // failure to set destination can only be because of missing terrain data
      copter.Log_Write_Error(ERROR_SUBSYSTEM_NAVIGATION,
                             ERROR_CODE_FAILED_TO_SET_DESTINATION);
      // failure is propagated to GCS with NAK
      MSG = end;
      return false;
    }
  } else {
    copter.rtl_init(false);
    MSG = end;
    return false;
  }
}

bool AP_ChargingStation::land_alternate_positionMidair() {
  Vector3f spare_position;
  Vector3f curr_position;
  curr_position = copter.inertial_nav.get_position();
  spare_position = copter.inertial_nav.get_position();

  spare_position.x = curr_position.x;
  spare_position.y = curr_position.y + distance;
  spare_position.z = curr_position.z;

  if (copter.set_mode(GUIDED, MODE_REASON_TX_COMMAND)) {
    if (copter.wp_nav->set_wp_destination(spare_position, false)) {
      gcs().send_text(MAV_SEVERITY_ERROR,
                                       "the alternate point");
      land_alternate_posMidair = true;
      return true;
    } else
      return false;
  } else
    return false;
}

bool AP_ChargingStation::do_takeoff() {

  if (!copter.motors->armed()) {
    if (copter.set_mode(GUIDED, MODE_REASON_TX_COMMAND)) {
      if (!copter.init_arm_motors(true))
        return false;
    } else
      return false;
  } else {
    if (!copter.set_mode(GUIDED, MODE_REASON_TX_COMMAND))
      return false;
  }

  float takeoff_alt = comm_alt; // unit: cm
  if (copter.do_user_takeoff(takeoff_alt, true)) {
    gcs().send_text(MAV_SEVERITY_ERROR, "do takeoff");
    return true;
  } else {
    gcs().send_text(MAV_SEVERITY_ERROR, "takeoff false");
    return false;
  }
}

void AP_ChargingStation::init_data(void) {
  _chargingStation_data.Instruction = 0xff;
  _chargingStation_data.source = 0xff;
  _chargingStation_data.D8 = 0xff;
  MSG = flight_other;
}

void AP_ChargingStation::sendCMD(uint8_t CMD_Data_H, uint8_t CMD_Data_L) {
  uint8_t checkByte = 0;

  _port->write(0x7A);
  checkByte += 0x7A;
  _port->write(0xA7);
  checkByte += 0xA7;
  _port->write(0x09);
  checkByte += 0x09;
  _port->write(Coordinator_Address_H);
  checkByte += Coordinator_Address_H;
  _port->write(Coordinator_Address_L);
  checkByte += Coordinator_Address_L;
  _port->write(Channel);
  checkByte += Channel;
  _port->write(CMD_Data_H);
  checkByte += CMD_Data_H;
  _port->write(CMD_Data_L);
  checkByte += CMD_Data_L;
  _port->write(checkByte & 0xFF);
}

void AP_ChargingStation::reset_flight_status() { fli_status = flight_default; }

void AP_ChargingStation::set_blastoff_flag() { blastoff_flag = 1; }

void AP_ChargingStation::set_receive_takeoff() {
  // printf("takeoff in %d\n",receive_takeoff);
  receive_takeoff = true;
  // printf("takeoff out %d\n",receive_takeoff);
}

void AP_ChargingStation::receiveData(uint8_t Data) {
  switch (state_machine) {
  case 0:
    //		init_data();
    if (Data == 0x7A) {
      state_machine = 1;
      sumchkm = Data;
    } else {
      _num_error.Time_Head_error++;
      state_machine = 0;
    }
    break;
  case 1:
    if (Data == 0xA7) {
      state_machine = 2;
      sumchkm += Data;
    } else {
      _num_error.Time_Head_error++;
      state_machine = 0;
    }
    break;
  case 2:
    length = Data;
    state_machine = 3;
    sumchkm += Data;
    break;
  case 3:
    if (Data == Broadcast_Address_H) {
      state_machine = 4;
      sumchkm += Data;
    } else {
      _num_error.Time_Invalid_data++;
      state_machine = 0;
    }
    break;
  case 4:
    if (Data == Broadcast_Address_L) {
      state_machine = 5;
      sumchkm += Data;
    } else {
      _num_error.Time_Invalid_data++;
      state_machine = 0;
    }
    break;
  case 5:
    if (Data == Channel) {
      state_machine = 6;
      sumchkm += Data;
    } else {
      _num_error.Time_Invalid_data++;
      state_machine = 0;
    }
    break;
  case 6:
    if ((Data == 0x80) || (Data == 0x00)) {
      state_machine = 7;
      _chargingStation_data.source = Data;
      sumchkm += Data;
    } else {
      _num_error.Time_Invalid_data++;
      state_machine = 0;
    }
    break;
  case 7:
    _chargingStation_data.Instruction = Data;
    sumchkm += Data;
    state_machine = 8;
    break;
  case 8:
    if (length > 0x09) {
      _chargingStation_data.D8 = Data;
      sumchkm += Data;
      state_machine = 9;
    } else {
      if (Data == (sumchkm & 0xFF)) {
        receive_complete_frag = 1;
      } else
        _num_error.Time_Parity_error++;
      state_machine = 0;
    }
    break;
  case 9:
    if (Data == (sumchkm & 0xFF)) //
    {
      receive_complete_frag = 1;
    } else
      _num_error.Time_Parity_error++;
    state_machine = 0;
    break;
  default:
    break;
  }
}

int AP_ChargingStation::data_deal_S() {
  uint8_t flight_status = flight_other;

  if (send_cmd == 1) {
    send_cmd = 0;
    if (cmd_station == 4) {
      sendCMD(0x00, 0x00);
    } else if (cmd_station > 4) {
      sendCMD(0x00, cmd_station - 1);
    } else {
      sendCMD(0x00, cmd_station);
    }
    //		_port->write(repeat);
    //		_port->write(Timeout);

    if (cmd_station == 7)
      timeout = 10;
    else
      timeout = Timeout * 10;

    flight_status = flight_other;
  }

  if (receive_complete_frag == 1) {
    //	    printf("AA\n");
    //		printf("%d\n",cmd_station);
    //		printf("%d\n",_chargingStation_data.Instruction);
    //		printf("\n");
    receive_complete_frag = 0;

    if ((cmd_station == 0 || cmd_station == 4) &&
        (_chargingStation_data.D8 != 0x00) &&
        (_chargingStation_data.source == 0x80)) {
      _num_error.Fail_time++;
      flight_status = comm_refuse;
      if (repeat == 0) {
        flight_status = comm_failure;
        cmd_station = 12; // End state machine
        gcs().send_text(MAV_SEVERITY_ERROR,
                                         "Connection refused");
      }
    } else if ((cmd_station == 1) && (_chargingStation_data.D8 != 0x00) &&
               (_chargingStation_data.source == 0x80)) {
      _num_error.Fail_time++;
      flight_status = reqL_refuse;
      if (repeat == 0) {
        flight_status = reqL_failure;
        cmd_station = 12; // End state machine
        gcs().send_text(MAV_SEVERITY_ERROR,
                                         "Request landing refused");
      }
    } else if ((cmd_station == 3) &&
               (_chargingStation_data.Instruction != 0x03) &&
               (_chargingStation_data.Instruction != 0x09) &&
               (_chargingStation_data.Instruction != 0x0A)) {
      _num_error.Fail_time++;
      flight_status = comL_other;
    } else if ((cmd_station == 3) &&
               (_chargingStation_data.Instruction == 0x03) &&
               (_chargingStation_data.source == 0x80)) {
      cmd_station = 12;
      landtimes = 0;
      flight_status = comL_success;
    } else if ((cmd_station == 3) &&
               (_chargingStation_data.Instruction == 0x09) &&
               (_chargingStation_data.source == 0x00)) {
      sendCMD(0x80, 0x09);
      landtimes++;
      againland_flag = 1;
      cmd_station = 11;
      chargingStation_open = true;
      flight_status = re_landed;

      /*
      if(landtimes>3){//More than the maximum number of landing  3
              cmd_station = 12;
              againland_flag = 0;
              flight_status= comL_failure;
              gcs().send_text(MAV_SEVERITY_ERROR, "Has reached
      the maximum number of re-takeoff");
      }
      */
    } else if ((cmd_station == 3) &&
               (_chargingStation_data.Instruction == 0x0A) &&
               (_chargingStation_data.source == 0x00)) {
      sendCMD(0x80, 0x0A);
      gcs().send_text(MAV_SEVERITY_ERROR, "Re-landing failed");
      cmd_station = 12; // End state machine
      flight_status = comL_failure;
    } else if ((cmd_station == 5) && (_chargingStation_data.D8 != 0x00) &&
               (_chargingStation_data.source == 0x80)) {
      _num_error.Fail_time++;
      flight_status = reqF_refuse;
      if (repeat == 0) {
        flight_status = reqF_failure;
        cmd_station = 12; // End state machine
        gcs().send_text(MAV_SEVERITY_ERROR,
                                         "The request to take off was refused");
      }
    } else if ((cmd_station == 7) &&
               (_chargingStation_data.Instruction == 0x06) &&
               (_chargingStation_data.source == 0x80)) {
      cmd_station = 12; // End state machine
      flight_status = comF_success;
    } else if ((cmd_station == 7) &&
               (_chargingStation_data.Instruction != 0x06)) {
      _num_error.Fail_time++;
      flight_status = comF_other;
    } else {
      if ((cmd_station == 0 || cmd_station == 4) &&
          (_chargingStation_data.Instruction == 0x00) &&
          (_chargingStation_data.source == 0x80)) {
        cmd_station++;
        repeat = repeat_num[cmd_station];
        send_cmd = 1;
        timeout = Timeout * 10;
        flight_status = comm_success;
      } else if ((cmd_station == 1) &&
                 (_chargingStation_data.Instruction == 0x01) &&
                 (_chargingStation_data.source == 0x80)) {
        cmd_station++;
        repeat = repeat_num[cmd_station];
        send_cmd = 1;
        timeout = Timeout * 10;
        flight_status = reqL_success;
      } else if ((cmd_station == 5) &&
                 (_chargingStation_data.Instruction == 0x04) &&
                 (_chargingStation_data.source == 0x80)) {
        cmd_station++;
        repeat = repeat_num[cmd_station];
        send_cmd = 1;
        timeout = Timeout * 10;
        flight_status = reqF_success;
      }
    }
  }

  if (timeout == 0) {
    _num_error.Timeout_time++;
    if (repeat > 0) {
      repeat--;
      send_cmd = 1;
      flight_status = flight_other;
    } else {
      switch (cmd_station) {
      case 0:
      case 4:
        gcs().send_text(MAV_SEVERITY_ERROR,
                                         "Connection not responding");
        cmd_station = 12; // End state machine
        flight_status = comm_noresponse;
        break;
      case 1:
        gcs().send_text(MAV_SEVERITY_ERROR,
                                         "Request landing not responding");
        cmd_station = 12; // End state machine
        flight_status = reqL_noresponse;
        break;
      case 3:
        gcs().send_text(
            MAV_SEVERITY_ERROR,
            "Landing successfully,didn't receive a response");
        cmd_station = 12; // End state machine
        flight_status = comL_noresponse;
        break;
      case 5:
        gcs().send_text(
            MAV_SEVERITY_ERROR,
            "Request to take off did not receive a response");
        cmd_station = 12; // End state machine
        flight_status = reqF_noresponse;
        break;
      case 7:
        gcs().send_text(
            MAV_SEVERITY_ERROR,
            "Take off successfully, didn't receive a response");
        cmd_station = 12; // End state machine
        flight_status = comF_noresponse;
        break;
      }
    }
  }

  return flight_status;
}

int AP_ChargingStation::data_deal_R() {
  uint8_t flight_status = flight_other;

  if (receive_complete_frag == 1) {
    receive_complete_frag = 0;

    if ((cmd_station == 2) && (_chargingStation_data.D8 == 0x00) &&
        (_chargingStation_data.Instruction == 0x02) &&
        (_chargingStation_data.source == 0x00)) {
      cmd_station =
          11; // Jump to the branch that detects the state of the aircraft
      readyland_flag = 1;
      flight_status = reaL_success;
      fli_status = flight_default;
      sendCMD(0x80, _chargingStation_data.Instruction);
    } else if ((cmd_station == 2) && (_chargingStation_data.D8 != 0x00) &&
               (_chargingStation_data.Instruction == 0x02) &&
               (_chargingStation_data.source == 0x00)) {
      _num_error.Fail_time++;
      flight_status = reaL_failure;
      cmd_station = 12; // End state machine
      sendCMD(0x80, _chargingStation_data.Instruction);
      gcs().send_text(MAV_SEVERITY_ERROR,
                                       "Landing ready process was refused");
    } else if ((cmd_station == 6) && (_chargingStation_data.D8 == 0x00) &&
               (_chargingStation_data.Instruction == 0x05) &&
               (_chargingStation_data.source == 0x00)) {
      cmd_station =
          11; ////Jump to the branch that detects the state of the aircraft
      readyfly_flag = 1;
      flight_status = reaF_success;
      fli_status = flight_default;
      sendCMD(0x80, _chargingStation_data.Instruction);
    } else if ((cmd_station == 6) && (_chargingStation_data.D8 != 0x00) &&
               (_chargingStation_data.Instruction == 0x05) &&
               (_chargingStation_data.source == 0x00)) {
      _num_error.Fail_time++;
      flight_status = reaF_failure;
      cmd_station = 12; // End state machine
      sendCMD(0x80, _chargingStation_data.Instruction);
      gcs().send_text(MAV_SEVERITY_ERROR,
                                       "The flight ready process was refused");
    }
  }

  if (timeout == 0) {
    _num_error.Timeout_time++;
    if (repeat > 0) {
      repeat--;
      flight_status = flight_other;
    } else { //锟截凤拷锟疥，锟斤拷锟斤拷通锟斤拷失锟杰ｏ拷repeat锟斤拷示锟截凤拷锟斤拷锟轿ｏ拷
      switch (cmd_station) {
      case 2:
        gcs().send_text(
            MAV_SEVERITY_ERROR, "The landing ready process is not responding");
        cmd_station = 12; // End state machine
        flight_status = reaL_noresponse;
        break;
      case 6:
        gcs().send_text(
            MAV_SEVERITY_ERROR, "The flight ready process is not responding");
        cmd_station = 12; // End state machine
        flight_status = reaF_noresponse;
        break;
      }
    }
    timeout = Timeout * 10;
  }

  return flight_status;
}

int AP_ChargingStation::analytic_protocol() {
  int status = flight_other;

  uint8_t mun;
  mun = _port->available();
  if (mun) {
    for (int i = 0; i < mun; i++)
      receiveData(_port->read());
  }

  if ((blastoff_flag == 1)) { // start fly
    startlanded = false;
    init_data();
    send_cmd = 1;
    landtimes = 0;
    cmd_station = 4;
    blastoff_flag = 0;
    state_machine = 0;
    repeat = comm_repetition - 1;
    MSG = flight_other;
    fli_status = flight_default;
    chargingStation_open = false;
    // Is communicating
    CMD_MSG.station_status = 0;
    CMD_MSG.command = 1;
    CMD_MSG.command_status = 1;
    CMD_MSG.command_result = 0;
  } else if (blastoff_flag == 2) { // start land
    startlanded = true;
    blastoff_flag = 0;
    cmd_station = 0;
    state_machine = 0;
    send_cmd = 1;
    repeat = repeat_num[cmd_station];
    timeout = Timeout * 10;
    landtimes = 0;
    init_data();
    MSG = flight_other;
    fli_status = flight_default;
    chargingStation_open = false;
    // Is communicating
    CMD_MSG.station_status = 0;
    CMD_MSG.command = 1;
    CMD_MSG.command_status = 1;
    CMD_MSG.command_result = 0;
  } else if (blastoff_flag == 0) {
    if (cmd_station == 12)
      status = -1;
    else if (cmd_station == 11) {
      // To detect whether the aircraft has completed landing
      if ((againland_flag) && (fli_status == flying_complete) &&
          (_chargingStation_data.Instruction == 0x09) && landtimes <= 3) {
        againland_flag = 0;
        cmd_station = 1;
        repeat = repeat_num[cmd_station];
        send_cmd = 1;
        init_data();
        startlanded = true;
      }

      // To detect whether the aircraft has completed takeoff or landing
      if (readyland_flag == 1 && fli_status == landing_complete) {
        readyland_flag = 0;
        cmd_station = 3;
        repeat = repeat_num[cmd_station];
        send_cmd = 1;
      }
      if (readyfly_flag == 1 && fli_status == flying_complete) {
        readyfly_flag = 0;
        cmd_station = 7;
        repeat = repeat_num[cmd_station];
        send_cmd = 1;
      }
    } else if (cmd_station < 11) {
      if ((cmd_station != 2) && (cmd_station != 6)) {
        status = data_deal_S();
      } else {
        status = data_deal_R();
      }
    }
  }

  return status;
}

void AP_ChargingStation::update() {
  // Debug("AP_ChargingStation messages: %d Current message: %d\n", 1, 1);
  if (!_initialised || (!Bstation_use.get())) {
    return;
  }
  
  time_check();
  
  int8_t status = analytic_protocol();

  set_CMD_MSG((Communication_status)status);

  switch (status) {
  case comm_failure:
  case comm_noresponse:
  case reqL_failure:
  case reqL_noresponse:
  case reaL_failure:
  case reaL_noresponse:
    if (startlanded) { // land
      startlanded = false;
      if (copter.set_mode(GUIDED, MODE_REASON_TX_COMMAND)) {
        if (land_alternate_positionMidair()) {
          MSG = land_spare_position;
        }
      }
    }
    break;
  case reaL_success:
    MSG = reaL_success;
    copter.set_mode(LAND, MODE_REASON_TX_COMMAND);
    break;
  case re_landed:
    MSG = re_landed;
    do_takeoff();
    break;
  case reaF_success:
    MSG = reaF_success;
    chargingStation_open = true;
    tasktime = AP_HAL::millis64();
    break;
  case comF_success:
  case comF_noresponse:
  case reqF_failure:
  case reqF_noresponse:
  case reaF_failure:
  case reaF_noresponse:
    // Add: the action after the Launch
    break;
  }

  switch (MSG) {
  // Back to the landing point, and start communication after 2 seconds
  case back_to_station:
    if (copter.wp_nav->reached_wp_destination()) {
      tasktime = AP_HAL::millis64();
      MSG = startcommunication;
      back_to_station_midair = false;
    }
    break;
  case startcommunication:
    if ((AP_HAL::millis64() - tasktime) > 2000) {
      start_communication();
      MSG = end;
    }
    break;
  // Fly to the reserve landing point, and after landing 2 seconds, began to
  // land
  case land_spare_position:
    if (copter.wp_nav->reached_wp_destination()) {
      tasktime = AP_HAL::millis64();
      MSG = startland;
      land_alternate_posMidair = false;
    }
    break;
  case startland:
    if ((AP_HAL::millis64() - tasktime) > 2000) {
      if (copter.set_mode(LAND, MODE_REASON_TX_COMMAND))
        MSG = land;
    }
    break;
  case land:
    if (!copter.motors->armed()) {
      landed();
      MSG = end;
      chargingStation_open = false;
    }
    break;
  // Re-take off, and after reaching the designated height, began to request
  // landing
  case re_landed:
    if (copter.wp_nav->reached_wp_destination()) {
      tasktime = AP_HAL::millis64();
      MSG = hover;
    }
    break;
  case hover:
    if ((AP_HAL::millis64() - tasktime) > 2000) {
      copter.guided_pos_control_start();
      fly();
      MSG = end;
    }
    break;
  // After the flight control command is received,
  // it is determined whether or not it has dropped to the ground by detecting
  // the state of the motor
  case reaL_success:
    if (!copter.motors->armed()) {
      landed();
      chargingStation_open = false;
      MSG = end;
    }
    break;
  // Upon receipt of the take-off, check whether the aircraft has reached the
  // specified altitude
  case reaF_success:
    /*
    Debug("reached %d\n",copter.wp_nav->reached_wp_destination());
    Debug("alt %d\n",copter.current_loc.alt);
    Debug("alt position %.3f\n",copter.inertial_nav.get_position().z);
    Debug("comm_fly_alt %d\n",comm_fly_alt);
    */
    // Debug("takeoff fly %d\n",receive_takeoff);
    if (receive_takeoff && (copter.wp_nav->reached_wp_destination() ||
                            (copter.current_loc.alt > comm_fly_alt) ||
                            fli_status == flying_complete)) {
      fly();
      MSG = end;
      receive_takeoff = false;
    } else if (!copter.motors->armed() && ((AP_HAL::millis64() - tasktime) >
                                           (uint64_t)(sta_time_out * 1000))) {
      reset_CMD_MSG();
      MSG = end;
      // receive_takeoff = false;
    } else if (!copter.motors->armed() &&
               ((AP_HAL::millis64() - tasktime) > 20000)) {
      reset_CMD_MSG();
      MSG = end;
      // receive_takeoff = false;
    }
    break;
  }

  copter.DataFlash.Log_Write_RTBS(copter.wp_nav->reached_wp_destination(),
                           back_to_station_midair, land_alternate_posMidair,
                           status, MSG);
}
#endif

