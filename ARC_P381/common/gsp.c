/*
 * gsp.c
 *
 * SPHERES Guest Scientist Program custom source code template.
 *
 * MIT Space Systems Laboratory
 * SPHERES Guest Scientist Program
 * http://ssl.mit.edu/spheres/
 *
 * Copyright 2005 Massachusetts Institute of Technology
 */

/*----------------------------------------------------------------------------*/
/*                         Do not modify this section.                        */
/*----------------------------------------------------------------------------*/
#include "ctrl_attitude.h"
#include "ctrl_position.h"
#include "find_state_error.h"
#include "ctrl_mix.h"
#include "gsutil_checkout.h"

#include "comm.h"
#include "comm_internal.h"
#include "comm_process_rx_packet.h"
#include "commands.h"
#include "control.h"
#include "exp_v2.h"
#include "fpga.h"
#include "gsp.h"
#include "gsp_task.h"
#include "housekeeping.h"
#include "math_matrix.h"
#include "pads.h"
#include "pads_internal.h"
#include "prop.h"
#include "smt335async.h"
#include "spheres_constants.h"
#include "spheres_physical_parameters.h"
#include "spheres_types.h"
#include "std_includes.h"
#include "system.h"
#include "util_memory.h"
#include <string.h>


/*----------------------------------------------------------------------------*/
/*                     Modify as desired below this point.                    */
/*----------------------------------------------------------------------------*/
#include "smartphone_comm_utils.h"
#include "math.h"

#define THRUSTER_TEST_TIME 120000
#define GSP_SETS_WAYPOINTS 18

// GLOBAL VARIABLES

int g_maneuver_nums[MAX_MANEUVERS];
int g_maneuver_num_index = 0;
static int g_test_class;
state_vector g_ctrl_state_target;  // Error vector is calculated against this.
char g_stop_at_end;
unsigned char g_target_reached = FALSE;
unsigned char g_sphere_error = FALSE;
unsigned short g_last_cmd = 0;
// for debugging will delete later
static int g_packets_from_phone =0;
static int g_state_packets_from_phone =0;
static int g_cmd_packets_from_phone =0;
float g_phone_pos_in_sphere_coords[3] = {0.15, 0.0, 0.0};
float g_phone_rigid_rot[4] = {0.0, 0.0, 0.0, 1.0};
int g_received_phone_command = FALSE;

int g_micro_maneuver_nums[MAX_MANEUVERS];
int g_micro_maneuver_index;


// FORWARD DECLARATIONS
void DifferentiatePhoneMessage(unsigned char channel,
                               unsigned char* buffer, unsigned int len);
void SendSOHPacketToPhone();
void SendEstimatePacketToPhone(unsigned int test_number);
void SendThrusterTimingsToPhone( prop_time *firing_times);
void SendTelemetryPacketToPhone();
void SendInertialPacketToPhone();
void CustomMixWLoc( prop_time *firing_times, float *control, float *state,
                    unsigned int minPulseWidth, float duty_cycle );

// FUNCTION IMPLEMENTATIONS

// Set satellite identity. The first primary interface function
// called.
void gspIdentitySet() {
  // set the logical identifier (SPHERE#) for this vehicle
  sysIdentitySet(SPHERE_ID);
}

// Initialize communications and other subsystems. Must contain
// certain initialization functions for multi-sphere operations to
// work correctly.
void gspInitProgram()
{
  // set the unique program identifier (to be assigned by MIT)
  sysProgramIDSet(THE_PROGRAM_NUMBER);

  // set up communications TDMA frames
  commTdmaStandardInit(COMM_CHANNEL_STL, sysIdentityGet(), NUM_SPHERES);
  commTdmaStandardInit(COMM_CHANNEL_STS, sysIdentityGet(), NUM_SPHERES);
  commTdmaStandardInit(COMM_CHANNEL_EXP, sysIdentityGet(), NUM_SPHERES);

  // enable communications channels
  commTdmaEnable(COMM_CHANNEL_STL);
  commTdmaEnable(COMM_CHANNEL_EXP);

  // allocate storage space for IMU samples
  padsInertialAllocateBuffers(50);

  // inform system of highest beacon number in use
  padsInitializeFPGA(NUM_BEACONS);

  /* custom program initialization goes below this point */
  g_maneuver_num_index = 0;

  *SMT335CP4 = 0x1104;// Talk to the Expansion Board at 250 kbps

  expv2_init(); // still needed? not in VERTIGO_ExpV2_Testing
  expv2_uart_cbk_register(1,&DifferentiatePhoneMessage);
  expv2_uart_baud_set(1,115200);
}

// Perform test-specific configuration. Called prior to starting each
// test.
void gspInitTest(unsigned int test_number)
{
  extern state_vector initState;

  ctrlPeriodSet(1000);

  padsEstimatorInitWaitAndSet(initState, 50, 200, 105,
                              PADS_INIT_THRUST_INT_ENABLE,
                              PADS_BEACONS_SET_1TO9); // ISS

  memset(g_ctrl_state_target,0,sizeof(state_vector));
  g_ctrl_state_target[POS_Z] = DEFAULT_Z;
  g_ctrl_state_target[QUAT_1] = BIAS_QX;
  g_ctrl_state_target[QUAT_2] = BIAS_QY;
  g_ctrl_state_target[QUAT_3] = BIAS_QZ;
  g_ctrl_state_target[QUAT_4] = BIAS_QW;

  g_maneuver_num_index = 0;
  g_test_class = NOT_CHECKOUT;

  switch (test_number) {
  case 1: // Quick Checkout
    g_test_class = CHECKOUT;
    gspInitTest_Checkout(test_number);
    break;
  case 2: // use SPHERES estimate
  case 3: // custom mixer
    g_target_reached = FALSE;
    g_sphere_error = FALSE;
    g_maneuver_nums[ 0] =  CONVERGE_MODE;
    g_maneuver_nums[ 1] =  WAYPOINT_MODE;//DRIFT_MODE; XXX
    g_maneuver_nums[ 2] =  WAYPOINT_MODE;
    g_stop_at_end = TRUE;

    // turn on the estimator
    padsEstimatorInitWaitAndSet(initState, 50, 200, 205,
                                PADS_INIT_THRUST_INT_ENABLE,PADS_BEACONS_SET_1TO9); // ISS
    break;
  case 4: // use Phone estimate
  case 5: // custom mixer
    g_target_reached = FALSE;
    g_sphere_error = FALSE;
    g_maneuver_nums[ 0] =  CONVERGE_MODE;
    g_maneuver_nums[ 1] =  WAYPOINT_MODE;//DRIFT_MODE; XXX
    g_maneuver_nums[ 2] =  WAYPOINT_MODE;
    g_stop_at_end = TRUE;

    // don't turn on the estimator
    // not sure if I actually need both of these
    // padsEstimatorInitWaitAndSet(initState, 50, SYS_FOREVER, SYS_FOREVER,
    //      PADS_INIT_THRUST_INT_ENABLE,PADS_BEACONS_SET_1TO9); // ISS

    padsEstimatorDisable();
    break;
  case 18: // i Trajectory 1

    g_maneuver_nums[ 0] =  CONVERGE_MODE;
    g_maneuver_nums[ 1] =  GSP_SETS_WAYPOINTS;
    
    g_micro_maneuver_nums[0] = 1; // 0 0 0
    g_micro_maneuver_nums[1] = 2; // .4 .4 .4
  break;
  
  case 19: // j Trajectory 2
    g_maneuver_nums[ 0] =  CONVERGE_MODE;
    g_maneuver_nums[ 1] =  GSP_SETS_WAYPOINTS;
    

    g_micro_maneuver_nums[0] = 1; // 0 0 0
    g_micro_maneuver_nums[1] = 3; // Pitch 90
    g_micro_maneuver_nums[2] = 4; // pitch 180
    g_micro_maneuver_nums[3] = 5; // pitch 270
    g_micro_maneuver_nums[4] = 6; // pitch 0
	break;
	
  case 20: // k Trajectory 3
    g_maneuver_nums[ 0] =  CONVERGE_MODE;
    g_maneuver_nums[ 1] =  GSP_SETS_WAYPOINTS;
    
    g_micro_maneuver_nums[0] = 1; // 0 0 0
    g_micro_maneuver_nums[1] = 7; // pitch -45
    g_micro_maneuver_nums[2] = 8; // .2 0 .2
  break;
  
  case 21: // l Trajectory 4
    g_maneuver_nums[ 0] =  CONVERGE_MODE;
    g_maneuver_nums[ 1] =  GSP_SETS_WAYPOINTS;
    
    g_micro_maneuver_nums[0] = 9; // 0, .3, -.3
    g_micro_maneuver_nums[1] = 10;// roll 45
  break;
  
  case 22: // m Trajectory 5
    g_maneuver_nums[ 0] =  CONVERGE_MODE;
    g_maneuver_nums[ 1] =  GSP_SETS_WAYPOINTS;
    
    g_micro_maneuver_nums[0] = 11; // 0 0 .3
    g_micro_maneuver_nums[1] = 12; // 0 -.3 .3
  break;
  
  case 23: // n Trajectory 6
    g_maneuver_nums[ 0] =  CONVERGE_MODE;
    g_maneuver_nums[ 1] =  GSP_SETS_WAYPOINTS;
    
    g_micro_maneuver_nums[0] = 13; // -.3 0 0 
    g_micro_maneuver_nums[1] = 14; // -.3 0 .3
  	break;
  
  // fall through intended for 6-17:
  case 6: // +X
  case 7: // -X
  case 8: // +Y
  case 9: // -Y
  case 10: // a +Z
  case 11: // b -Z
  case 12: // c +Roll
  case 13: // d -Roll
  case 14: // e +Pitch
  case 15: // f -Pitch
  case 16: // g +Yaw
  case 17: // h -Yaw
  default:
    g_last_cmd = 0;
    g_maneuver_nums[ 0] =  CONVERGE_MODE;
    g_maneuver_nums[ 1] =  test_number;

    // turn on the estimator
    padsEstimatorInitWaitAndSet(initState, 50, 200, 205,
                                PADS_INIT_THRUST_INT_ENABLE,PADS_BEACONS_SET_1TO9); // ISS
    break;
  }
  g_last_cmd = 0;
  g_micro_maneuver_index = 0;
}

// Specify task trigger mask
void gspInitTask() {
}

// Perform state estimate based on inertial data. Called periodically.
void gspPadsInertial(IMU_sample *accel, IMU_sample *gyro,
                     unsigned int num_samples) {
  if (g_test_class == CHECKOUT) {
    gspPadsInertial_Checkout(accel, gyro, num_samples);
    return;
  }

  SendInertialPacketToPhone();
}

// Record global data. Called at the end of each beacon's transmission
// period.
void gspPadsGlobal(unsigned int beacon,
                   beacon_measurement_matrix measurements) {
  SendTelemetryPacketToPhone();
}

// Event driven task for estimation, control, and
// communications. Called whenever a masked event occurs.
void gspTaskRun(unsigned int gsp_task_trigger, unsigned int extra_data) {
  switch (g_test_class) {
  case CHECKOUT:
    gspTaskRun_Checkout(gsp_task_trigger,extra_data);
    break;
  default:
    break;
  }
}

// Apply control laws and set thruster on-times. Called periodically.
void gspControl(unsigned int test_number,
                unsigned int test_time,
                unsigned int maneuver_number,
                unsigned int maneuver_time) {
  static state_vector curr_state; // current state vector of the sphere
  static state_vector ctrl_state_error; // difference btwn curr_state and
  static float ctrl_control[6];
  static prop_time firing_times;
  const int min_pulse = 10;
  extern const float KPattitudePD, KDattitudePD, KPpositionPD, KDpositionPD;
  dbg_float_packet dbg_target;
  dbg_short_packet dbg_error;
  float m00, m10, qx, qy, qz, qw;

  // Always send a SOH and Telemetry down the line
  SendSOHPacketToPhone();

  // Clear all uninitialized vectors
  memset(ctrl_control,0,sizeof(float)*6);
  memset(ctrl_state_error,0,sizeof(state_vector));
  memset(dbg_error,0,sizeof(dbg_short_packet));
  memset(dbg_target,0,sizeof(dbg_float_packet));

  // Get the current result for localization
  padsStateGet(curr_state);

  // Check for early exit condition
  if( g_test_class == CHECKOUT ) {
    gspControl_Checkout(test_number, test_time, maneuver_number, maneuver_time);
    return;
  }
  
  // set so we're ready for maneuvers 6 and up
  memset(firing_times.off_time, 0, sizeof(int)*12);
  memset(firing_times.on_time, 0, sizeof(int)*12);
  
  if (maneuver_number == CONVERGE_MODE) { //Estimator initialization
    if (test_time >= ESTIMATOR_TIME){
      // If it seems that it is about time that the estimator should
      // have converged. Then we'll command the sphere to hold current
      // position.
      g_maneuver_num_index++;
      ctrlManeuverNumSet(g_maneuver_nums[g_maneuver_num_index]);
      
      if(g_received_phone_command == FALSE) {
        //memcpy(g_ctrl_state_target, curr_state, sizeof(state_vector));
        g_ctrl_state_target[POS_X] = 0.0f;
    	g_ctrl_state_target[POS_Y] = 0.0f;
      	g_ctrl_state_target[POS_Z] = 0.0f;
      	g_ctrl_state_target[QUAT_1] = 0.0f;
      	g_ctrl_state_target[QUAT_2] = 0.0f;
      	g_ctrl_state_target[QUAT_3] = 0.0f;
        g_ctrl_state_target[QUAT_4] = 1.0f;
      }
      
      // we're never going to command a non-zero velocity, or rate
      g_ctrl_state_target[VEL_X] = 0.0f;
      g_ctrl_state_target[VEL_Y] = 0.0f;
      g_ctrl_state_target[VEL_Z] = 0.0f;
      g_ctrl_state_target[RATE_X] = 0.0f;
      g_ctrl_state_target[RATE_Y] = 0.0f;
      g_ctrl_state_target[RATE_Z] = 0.0f;
    }
  } else if (maneuver_number == DRIFT_MODE) {
    // do nothing - just drift!
  } else if (maneuver_number == WAYPOINT_MODE) {
    //Disable estimator during closed loop firing
    padsGlobalPeriodSet(SYS_FOREVER);

    findStateError(ctrl_state_error,curr_state,g_ctrl_state_target);

    //call controllers
    ctrlPositionPDgains(KPpositionPD, KDpositionPD, KPpositionPD,
                        KDpositionPD, KPpositionPD, KDpositionPD,
                        ctrl_state_error, ctrl_control);
    ctrlAttitudeNLPDwie(KPattitudePD, KDattitudePD, KPattitudePD,
                        KDattitudePD, KPattitudePD, KDattitudePD,
                        ctrl_state_error, ctrl_control);

    //mix forces/torques into thruster commands
    if ( test_number && 0x1 ) {
      // If the number is odd ...  (like 3 and 5) .. use the custom
      // mixer

      // Mix forces/torque into thruster commands. This is a custom
      // variant that compensates for the shift in center of gravity due
      // to the PeanutMM.
      CustomMixWLoc(&firing_times, ctrl_control, curr_state,
                    min_pulse, 20.0f);
    } else {
      ctrlMixWLoc(&firing_times, ctrl_control, curr_state,
                  min_pulse, 20.0f, FORCE_FRAME_INERTIAL);
    }

#ifdef LAB_VERSION
    // Don't bother using the Z thrusters. They're aligned with the
    // granite table.

    // This can't be performed with ctrl_control because those
    // measurements are done in the global coordinate frame and then
    // get rotated to the body frame. Meaning it will still try to
    // fire in Z direction.

    // Thrusters aligned with axis Z have indices 5,6,11,12. See Mark
    // Hilstad's thesis.
    firing_times.off_time[4] = firing_times.on_time[4] = 0;
    firing_times.off_time[5] = firing_times.on_time[5] = 0;
    firing_times.off_time[10] = firing_times.on_time[10] = 0;
    firing_times.off_time[11] = firing_times.on_time[11] = 0;
#endif

    // Set firing times
    propSetThrusterTimes(&firing_times);
    padsGlobalPeriodSetAndWait(200,205);

    // Tell the Phone about our firing solutions
    SendThrusterTimingsToPhone(&firing_times);

    // termination conditions
    if (maneuver_time > MIN_MANEUVER_TIME) {
      if ((smtAtPositionRotation(ctrl_state_error))) {
        // if rotation test, just need x correct
        // if maneuver 20 or not rotation test, need x and x_dot correct
        if ((!g_stop_at_end) || (smtAtZeroVelocity(ctrl_state_error))) {

          //  we got there
          g_target_reached = TRUE;
        }
      }
    }
    if (maneuver_time > MANEUVER_TIME_OUT) {
      g_target_reached = TIMED_OUT;
    }
  } // closes WAYPOINT_MODE
  //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  else if (maneuver_number == 6) { 
    // @@@@@@@@@@@@@@@@@@@@@@@@@@  GO TO +X
    //Disable estimator during closed loop firing
    padsGlobalPeriodSet(SYS_FOREVER);

    firing_times.off_time[0] = 200;
    firing_times.off_time[1] = 200;
   
    // Set firing times
    propSetThrusterTimes(&firing_times);
    padsGlobalPeriodSetAndWait(200,205);

    // Tell the Phone about our firing solutions
    SendThrusterTimingsToPhone(&firing_times);

    // termination conditions
    if (maneuver_time > THRUSTER_TEST_TIME) {
    	SendSOHPacketToPhone();
        ctrlTestTerminate(TEST_RESULT_NORMAL);
    }
  } else if (maneuver_number == 7) { 
    // @@@@@@@@@@@@@@@@@@@@@@@@@@@@  GO TO -X
    //Disable estimator during closed loop firing
    padsGlobalPeriodSet(SYS_FOREVER);

    firing_times.off_time[6] = 200;
    firing_times.off_time[7] = 200;
   
    // Set firing times
    propSetThrusterTimes(&firing_times);
    padsGlobalPeriodSetAndWait(200,205);

    // Tell the Phone about our firing solutions
    SendThrusterTimingsToPhone(&firing_times);

    // termination conditions
    if (maneuver_time > THRUSTER_TEST_TIME) {
    	SendSOHPacketToPhone();
        ctrlTestTerminate(TEST_RESULT_NORMAL);
    }
  } else if (maneuver_number == 8) { 
    // @@@@@@@@@@@@@@@@@@@@@@@@@@@@  GO TO +Y
    //Disable estimator during closed loop firing
    padsGlobalPeriodSet(SYS_FOREVER);

    firing_times.off_time[2] = 200;
    firing_times.off_time[3] = 200;
   
    // Set firing times
    propSetThrusterTimes(&firing_times);
    padsGlobalPeriodSetAndWait(200,205);

    // Tell the Phone about our firing solutions
    SendThrusterTimingsToPhone(&firing_times);

    	SendSOHPacketToPhone();
    // termination conditions
    if (maneuver_time > THRUSTER_TEST_TIME) {
        ctrlTestTerminate(TEST_RESULT_NORMAL);
    }
  } else if (maneuver_number == 9) { 
    // @@@@@@@@@@@@@@@@@@@@@@@@@@@@  GO TO -Y
    //Disable estimator during closed loop firing
    padsGlobalPeriodSet(SYS_FOREVER);

    firing_times.off_time[8] = 200;
    firing_times.off_time[9] = 200;
   
    // Set firing times
    propSetThrusterTimes(&firing_times);
    padsGlobalPeriodSetAndWait(200,205);

    // Tell the Phone about our firing solutions
    SendThrusterTimingsToPhone(&firing_times);

    // termination conditions
    if (maneuver_time > THRUSTER_TEST_TIME) {
    	SendSOHPacketToPhone();
        ctrlTestTerminate(TEST_RESULT_NORMAL);
    }
  } else if (maneuver_number == 10) { // a 
    // @@@@@@@@@@@@@@@@@@@@@@@@@@@@  GO TO +Z
    //Disable estimator during closed loop firing
    padsGlobalPeriodSet(SYS_FOREVER);

    firing_times.off_time[4] = 200;
    firing_times.off_time[5] = 200;
   
    // Set firing times
    propSetThrusterTimes(&firing_times);
    padsGlobalPeriodSetAndWait(200,205);

    // Tell the Phone about our firing solutions
    SendThrusterTimingsToPhone(&firing_times);

    // termination conditions
    if (maneuver_time > THRUSTER_TEST_TIME) {
    	SendSOHPacketToPhone();
        ctrlTestTerminate(TEST_RESULT_NORMAL);
    }
  } else if (maneuver_number == 11) { // b
    // @@@@@@@@@@@@@@@@@@@@@@@@@@@@  GO TO -Z
    //Disable estimator during closed loop firing
    padsGlobalPeriodSet(SYS_FOREVER);

    firing_times.off_time[10] = 200;
    firing_times.off_time[11] = 200;
   
    // Set firing times
    propSetThrusterTimes(&firing_times);
    padsGlobalPeriodSetAndWait(200,205);

    // Tell the Phone about our firing solutions
    SendThrusterTimingsToPhone(&firing_times);

    // termination conditions
    if (maneuver_time > THRUSTER_TEST_TIME) {
    	SendSOHPacketToPhone();
        ctrlTestTerminate(TEST_RESULT_NORMAL);
    }
  } else if (maneuver_number == 12) { // c
    // @@@@@@@@@@@@@@@@@@@@@@@@@@@@  Rotate +Roll
    //Disable estimator during closed loop firing
    padsGlobalPeriodSet(SYS_FOREVER);

    firing_times.off_time[4] = 200;
    firing_times.off_time[11] = 200;
   
    // Set firing times
    propSetThrusterTimes(&firing_times);
    padsGlobalPeriodSetAndWait(200,205);

    // Tell the Phone about our firing solutions
    SendThrusterTimingsToPhone(&firing_times);

    // termination conditions
    if (maneuver_time > THRUSTER_TEST_TIME) {
    	SendSOHPacketToPhone();
        ctrlTestTerminate(TEST_RESULT_NORMAL);
    }
  } else if (maneuver_number == 13) { // d
    // @@@@@@@@@@@@@@@@@@@@@@@@@@@@  Rotate -Roll
    //Disable estimator during closed loop firing
    padsGlobalPeriodSet(SYS_FOREVER);

    firing_times.off_time[5] = 200;
    firing_times.off_time[10]= 200;
   
    // Set firing times
    propSetThrusterTimes(&firing_times);
    padsGlobalPeriodSetAndWait(200,205);

    // Tell the Phone about our firing solutions
    SendThrusterTimingsToPhone(&firing_times);

    // termination conditions
    if (maneuver_time > THRUSTER_TEST_TIME) {
    	SendSOHPacketToPhone();
        ctrlTestTerminate(TEST_RESULT_NORMAL);
    }
  } else if (maneuver_number == 14) { // e
    // @@@@@@@@@@@@@@@@@@@@@@@@@@@@  Rotate +pitch
    //Disable estimator during closed loop firing
    padsGlobalPeriodSet(SYS_FOREVER);

    firing_times.off_time[0] = 200;
    firing_times.off_time[7] = 200;
   
    // Set firing times
    propSetThrusterTimes(&firing_times);
    padsGlobalPeriodSetAndWait(200,205);

    // Tell the Phone about our firing solutions
    SendThrusterTimingsToPhone(&firing_times);

    // termination conditions
    if (maneuver_time > THRUSTER_TEST_TIME) {
    	SendSOHPacketToPhone();
        ctrlTestTerminate(TEST_RESULT_NORMAL);
    }
  } else if (maneuver_number == 15) { // f
    // @@@@@@@@@@@@@@@@@@@@@@@@@@@@  Rotate -pitch
    //Disable estimator during closed loop firing
    padsGlobalPeriodSet(SYS_FOREVER);

    firing_times.off_time[1] = 200;
    firing_times.off_time[6] = 200;
   
    // Set firing times
    propSetThrusterTimes(&firing_times);
    padsGlobalPeriodSetAndWait(200,205);

    // Tell the Phone about our firing solutions
    SendThrusterTimingsToPhone(&firing_times);

    // termination conditions
    if (maneuver_time > THRUSTER_TEST_TIME) {
    	SendSOHPacketToPhone();
        ctrlTestTerminate(TEST_RESULT_NORMAL);
    }
  } else if (maneuver_number == 16) { // g 
    // @@@@@@@@@@@@@@@@@@@@@@@@@@@@  Rotate +yaw
    //Disable estimator during closed loop firing
    padsGlobalPeriodSet(SYS_FOREVER);

    firing_times.off_time[2] = 200;
    firing_times.off_time[9] = 200;
   
    // Set firing times
    propSetThrusterTimes(&firing_times);
    padsGlobalPeriodSetAndWait(200,205);

    // Tell the Phone about our firing solutions
    SendThrusterTimingsToPhone(&firing_times);

    // termination conditions
    if (maneuver_time > THRUSTER_TEST_TIME) {
    	SendSOHPacketToPhone();
        ctrlTestTerminate(TEST_RESULT_NORMAL);
    }
  } else if (maneuver_number == 17) { // h
    // @@@@@@@@@@@@@@@@@@@@@@@@@@@@  Rotate -yaw
    //Disable estimator during closed loop firing
    padsGlobalPeriodSet(SYS_FOREVER);

    firing_times.off_time[3] = 200;
    firing_times.off_time[8] = 200;
   
    // Set firing times
    propSetThrusterTimes(&firing_times);
    padsGlobalPeriodSetAndWait(200,205);

    // Tell the Phone about our firing solutions
    SendThrusterTimingsToPhone(&firing_times);

    // termination conditions
    if (maneuver_time > THRUSTER_TEST_TIME) {
    	SendSOHPacketToPhone();
        ctrlTestTerminate(TEST_RESULT_NORMAL);
    }
  } else if(maneuver_number == GSP_SETS_WAYPOINTS) {
  	 g_ctrl_state_target[QUAT_1] = 0.0f;
	 g_ctrl_state_target[QUAT_2] = 0.0f;
	 g_ctrl_state_target[QUAT_3] = 0.0f;
	 g_ctrl_state_target[QUAT_4] = 1.0f;
    // set target based on micro_maneuver_index
    switch(g_micro_maneuver_nums[g_micro_maneuver_index]) {
      case 1: // center
	      g_ctrl_state_target[POS_X] = 0.0f;
	      g_ctrl_state_target[POS_Y] = 0.0f;
	      g_ctrl_state_target[POS_Z] = 0.0f;
	      break;
      case 2: // 0.4, 0.4, 0.4
	      g_ctrl_state_target[POS_X] = 0.4f;
	      g_ctrl_state_target[POS_Y] = 0.4f;
	      g_ctrl_state_target[POS_Z] = 0.4f;
	      break;
      case 3: // pitch 90
	      g_ctrl_state_target[QUAT_1] = 0.0f;
	      g_ctrl_state_target[QUAT_2] = 0.71f;
	      g_ctrl_state_target[QUAT_3] = 0.0f;
	      g_ctrl_state_target[QUAT_4] = 0.71f;
	      break;
      case 4: // pitch 180
          g_ctrl_state_target[QUAT_1] = 0.0f;
	      g_ctrl_state_target[QUAT_2] = 1.0f;
	      g_ctrl_state_target[QUAT_3] = 0.0f;
	      g_ctrl_state_target[QUAT_4] = 0.0f;
	      break;
      case 5: // pitch 270
       	  g_ctrl_state_target[QUAT_1] = 0.0f;
	      g_ctrl_state_target[QUAT_2] = 0.71f;
	      g_ctrl_state_target[QUAT_3] = 0.0f;
	      g_ctrl_state_target[QUAT_4] = -0.71f;
	      break;
      case 6: // pitch 0
      	  g_ctrl_state_target[QUAT_1] = 0.0f;
	      g_ctrl_state_target[QUAT_2] = 0.0f;
	      g_ctrl_state_target[QUAT_3] = 0.0f;
	      g_ctrl_state_target[QUAT_4] = 1.0f;
	      break;
	  case 7: // pitch -45
      	  g_ctrl_state_target[QUAT_1] = 0.0f;
	      g_ctrl_state_target[QUAT_2] =-0.38f;
	      g_ctrl_state_target[QUAT_3] = 0.0f;
	      g_ctrl_state_target[QUAT_4] = 0.92f;
	      break;
	  case 8: // translate to 0.2, 0.0, 0.2
	      g_ctrl_state_target[POS_X] = 0.2f;
	      g_ctrl_state_target[POS_X] = 0.0f;
		  g_ctrl_state_target[POS_X] = 0.2f;
		  break;
      case 9: // translate to 0.0, 0.3, -0.3
	      g_ctrl_state_target[POS_X] = 0.0f;
	      g_ctrl_state_target[POS_X] = 0.3f;
		  g_ctrl_state_target[POS_X] =-0.3f;
		  break;
	  case 10: // roll 45
	  	  g_ctrl_state_target[QUAT_1] = 0.38f;
	      g_ctrl_state_target[QUAT_2] = 0.0f;
	      g_ctrl_state_target[QUAT_3] = 0.0f;
	      g_ctrl_state_target[QUAT_4] = 0.92f;
		  break;
      case 11: //0, 0, 0.3
          g_ctrl_state_target[POS_X] = 0.0f;
	      g_ctrl_state_target[POS_X] = 0.0f;
		  g_ctrl_state_target[POS_X] = 0.3f;
		  break;
      case 12: //0, -0.3, 0.3
          g_ctrl_state_target[POS_X] = 0.0f;
	      g_ctrl_state_target[POS_X] =-0.3f;
		  g_ctrl_state_target[POS_X] = 0.3f;
		  break;
	  case 13: //-0.3, 0, 0
          g_ctrl_state_target[POS_X] =-0.3f;
	      g_ctrl_state_target[POS_X] = 0.0f;
		  g_ctrl_state_target[POS_X] = 0.0f;
		  break;
      case 14: //-0.3, 0, -0.3
          g_ctrl_state_target[POS_X] =-0.3f;
	      g_ctrl_state_target[POS_X] = 0.0f;
		  g_ctrl_state_target[POS_X] =-0.3f;
		  break;
    }

    //Disable estimator during closed loop firing
    padsGlobalPeriodSet(SYS_FOREVER);

    findStateError(ctrl_state_error,curr_state,g_ctrl_state_target);

    //call controllers
    ctrlPositionPDgains(KPpositionPD, KDpositionPD, KPpositionPD,
                        KDpositionPD, KPpositionPD, KDpositionPD,
                        ctrl_state_error, ctrl_control);
    ctrlAttitudeNLPDwie(KPattitudePD, KDattitudePD, KPattitudePD,
                        KDattitudePD, KPattitudePD, KDattitudePD,
                        ctrl_state_error, ctrl_control);

    //mix forces/torques into thruster commands
    ctrlMixWLoc(&firing_times, ctrl_control, curr_state,
                min_pulse, 20.0f, FORCE_FRAME_INERTIAL);
                  
    // Set firing times
    propSetThrusterTimes(&firing_times);
    padsGlobalPeriodSetAndWait(200,205);

    // Tell the Phone about our firing solutions
    SendThrusterTimingsToPhone(&firing_times);

    // termination conditions
    if (maneuver_time > MIN_MANEUVER_TIME) {
      if ((smtAtPositionRotation(ctrl_state_error))) {
        // if rotation test, just need x correct
        // if maneuver 20 or not rotation test, need x and x_dot correct
        if ((!g_stop_at_end) || (smtAtZeroVelocity(ctrl_state_error))) {

          //  we got there
          g_micro_maneuver_index++;
        }
      }
    }
    if (maneuver_time > MANEUVER_TIME_OUT) {
      g_target_reached = TIMED_OUT;
    }
  } // closes GSP_SETS_WAYPOINTS
  
  // Display:
  //  - current position [X, Y, Z]
  //  - position error
  //  - degree error
  //  - velocity error
  //  - rate error
  //  - degree error projected in Z.
  dbg_target[0] = g_ctrl_state_target[POS_X];
  dbg_target[1] = g_ctrl_state_target[POS_Y];
  dbg_target[2] = g_ctrl_state_target[POS_Z];
  dbg_target[3] =
    sqrt(ctrl_state_error[POS_X]*ctrl_state_error[POS_X] +
         ctrl_state_error[POS_Y]*ctrl_state_error[POS_Y]);
  dbg_target[4] =
    smtGetQuaternionMagnitude(ctrl_state_error) * 180 / 3.14159;
  dbg_target[5] =
    sqrt(ctrl_state_error[VEL_X]*ctrl_state_error[VEL_X] +
         ctrl_state_error[VEL_Y]*ctrl_state_error[VEL_Y] +
         ctrl_state_error[VEL_Z]*ctrl_state_error[VEL_Z]);
  dbg_target[6] =
    sqrt(ctrl_state_error[RATE_X]*ctrl_state_error[RATE_X] +
         ctrl_state_error[RATE_Y]*ctrl_state_error[RATE_Y] +
         ctrl_state_error[RATE_Z]*ctrl_state_error[RATE_Z]);
  // Different method for calculating angular error.
  qx = ctrl_state_error[QUAT_1];
  qy = ctrl_state_error[QUAT_2];
  qz = ctrl_state_error[QUAT_3];
  qw = ctrl_state_error[QUAT_4];
  m00 = 1 - 2*qy*qy - 2*qz*qz;
  m10 = 2*qx*qy + 2*qz*qw;
  dbg_target[7] =
    fabs(atan2(m10,m00)) * 180 / 3.14159;

  dbg_error[0] = maneuver_time/1000;
  dbg_error[1] = smtAtPositionRotation(ctrl_state_error);
  dbg_error[2] = smtAtZeroVelocity(ctrl_state_error);
  dbg_error[3] = 0;
  dbg_error[4] = 0;
  dbg_error[5] = g_ctrl_state_target[QUAT_1]*1000;
  dbg_error[6] = g_ctrl_state_target[QUAT_2]*1000;
  dbg_error[7] = g_ctrl_state_target[QUAT_3]*1000;
  dbg_error[8] = g_ctrl_state_target[QUAT_4]*1000;
  dbg_error[9] = 0;
  dbg_error[10] = 0;
  dbg_error[11] = 0;

  commSendRFMPacket(COMM_CHANNEL_STL, GROUND,
                    COMM_CMD_DBG_FLOAT, (unsigned char *) dbg_target, 0);
  commSendRFMPacket(COMM_CHANNEL_STL, GROUND,
                    COMM_CMD_DBG_SHORT, (unsigned char *) dbg_error, 0);
}

void SendEstimatePacketToPhone(unsigned int test_number) {
  comm_payload_state_estimate my_state;
  state_vector curr_state;

  padsStateGet(curr_state);

  // Fill the packet
  my_state.timestamp = 0;//sysSphereTimeGet();
  memcpy( &my_state.pos[0], &curr_state[POS_X], 3*sizeof(float) );
  memcpy( &my_state.vel[0], &curr_state[VEL_X], 3*sizeof(float) );
  memcpy( &my_state.quat[0], &curr_state[QUAT_1], 4*sizeof(float) );
  memcpy( &my_state.rate[0], &curr_state[RATE_X], 3*sizeof(float) );

  // Send it out.
  smtExpV2UARTSendWHETHeader
    (EXPv2_CH1_HWID,
     sizeof(comm_payload_state_estimate),
     (unsigned char*)&my_state, 0x42);
}

void SendThrusterTimingsToPhone( prop_time *firing_times) {
  comm_payload_het_thruster packet;
  int i = 0;

  // Fill the packet
  packet.time = sysSphereTimeGet();
  for ( i = 0; i < 12; i++ ) {
    packet.on_time[i] = (unsigned char)firing_times->on_time[i];
  }
  for ( i = 0; i < 12; i++ ) {
    packet.off_time[i] = (unsigned char)firing_times->off_time[i];
  }

  // Send it out. The call will prepend a header and checksum
  smtExpV2UARTSendWHETHeader
    (EXPv2_CH1_HWID,
     sizeof(comm_payload_het_thruster),
     (unsigned char*)&packet, 0x43);
}

void SendInertialPacketToPhone(){
  comm_payload_het_inertial packet;
  static unsigned int accel_lp[3], gyro_lp[3];

  // The incoming accel and gyro are from a circlar buffer. There is
  // no way for us to determine if it is current or stale since the
  // index is hidden. Instead, I'll just pull from FPGA's memory the
  // current accel and gyro measurement unfiltered.
  gyro_lp[0] = 0xfff - (A2D_Gyros[0] & 0xfff);
  gyro_lp[1] = A2D_Gyros[1] & 0xfff;
  gyro_lp[2] = A2D_Gyros[2] & 0xfff;

  accel_lp[0] = 0xfff - (A2D_Accel[0] & 0xfff);
  if (commHWAddrGet() != 0x32)
    accel_lp[1] = 0xfff - (A2D_Accel[2] & 0xfff);
  else
    accel_lp[1] = A2D_Accel[2] & 0xfff;
  accel_lp[2] = A2D_Accel[1] & 0xfff;

  // Here we only send a short when we have an uint32 because the fill
  // the packet
  packet.time = sysSphereTimeGet();
  packet.accel[0] = (unsigned short)accel_lp[0];
  packet.accel[1] = (unsigned short)accel_lp[1];
  packet.accel[2] = (unsigned short)accel_lp[2];
  packet.gyro[0] = (unsigned short)gyro_lp[0];
  packet.gyro[1] = (unsigned short)gyro_lp[1];
  packet.gyro[2] = (unsigned short)gyro_lp[2];

  // Send it out. The call will prepend a header and checksum
  smtExpV2UARTSendWHETHeader
    (EXPv2_CH1_HWID, sizeof(comm_payload_het_inertial),
     (unsigned char*)&packet, 0x44 );
}

void SendSOHPacketToPhone() {
  comm_payload_het_soh packet;

  // get my SOH information
  commBackgroundSOHGet(SPHERE_ID, &packet.soh);

  // set the fields I need
  packet.soh.unused[0] = g_target_reached;
  packet.soh.unused[1] = g_sphere_error;
  packet.soh.unused[3] = (g_last_cmd>>8) & 0xFF;
  packet.soh.unused[2] = g_last_cmd & 0xFF;

  // send it
  smtExpV2UARTSendWHETHeader
    (EXPv2_CH1_HWID, sizeof(comm_payload_het_soh),
     (unsigned char *)&packet, COMM_CMD_SOH);
}

void SendTelemetryPacketToPhone() {
  comm_payload_het_telemetry packet;

  // Get Telemetry Information
  commBackgroundPayloadPack(&packet.telemetry);

  // send it out
  smtExpV2UARTSendWHETHeader
    (EXPv2_CH1_HWID, sizeof(comm_payload_het_telemetry),
     (unsigned char *)&packet, COMM_CMD_BACKGROUND);
}

void ProcessPhoneCommandFloat(unsigned char channel,
                              unsigned char* buffer, unsigned int len) {
  phone_cmd_float* cmd = (phone_cmd_float*)buffer;

  dbg_short_packet dbg_error;
  memset(dbg_error,0,sizeof(dbg_short_packet));

  dbg_error[0] = 123;
  dbg_error[1] = 123;
  dbg_error[2] = 123;
  dbg_error[3] = cmd->x;
  dbg_error[4] = cmd->y;
  dbg_error[5] = cmd->z;
  dbg_error[6] = cmd->qx;
  dbg_error[7] = cmd->qy;
  dbg_error[8] = cmd->qz;
  dbg_error[9] = cmd->qw;
  dbg_error[10] = 0;
  dbg_error[11] = 0;

  commSendRFMPacket(COMM_CHANNEL_STL, GROUND,
                    COMM_CMD_DBG_SHORT, (unsigned char *) dbg_error, 0);

  g_cmd_packets_from_phone++;

  // TODO: This looks dangerous. What happens when this wraps due to a
  // lot of commands?
  // if we see 32,787 commands in one spheres run, something else is 
  // seriously wrong
  if(cmd->seq_num <= g_last_cmd)
    return; // I've seen this command before

  // acknowledge that I received this command correctly
  g_last_cmd = cmd->seq_num;

  switch(cmd->cmd) {
  case GO_TO_XYZ:
    // aim for commanded position, and last commanded orientation
    g_ctrl_state_target[POS_X] = cmd->x;
    g_ctrl_state_target[POS_Y] = cmd->y;
    g_ctrl_state_target[POS_Z] = cmd->z;

    g_target_reached = FALSE;
    g_stop_at_end = cmd->stop_at_end;
    g_received_phone_command = TRUE;
    ctrlManeuverNumSet(WAYPOINT_MODE);
    break;

  case GO_TO_QUAT:
    // Updates quat that goes in the commandStateVector. Meaning ...
    // we are going to over-ride all past relative/manual movements.

    // aim for commanded orientation, and last commanded position
    g_ctrl_state_target[QUAT_1] = cmd->qx;
    g_ctrl_state_target[QUAT_2] = cmd->qy;
    g_ctrl_state_target[QUAT_3] = cmd->qz;
    g_ctrl_state_target[QUAT_4] = cmd->qw;

    g_target_reached = FALSE;
    g_stop_at_end = cmd->stop_at_end;
    g_received_phone_command = TRUE;
    ctrlManeuverNumSet(WAYPOINT_MODE);
    break;

  case GO_TO_STATE_VECTOR:
    g_ctrl_state_target[POS_X] = cmd->x;
    g_ctrl_state_target[POS_Y] = cmd->y;
    g_ctrl_state_target[POS_Z] = cmd->z;
    
    g_ctrl_state_target[QUAT_1] = cmd->qx;
    g_ctrl_state_target[QUAT_2] = cmd->qy;
    g_ctrl_state_target[QUAT_3] = cmd->qz;
    g_ctrl_state_target[QUAT_4] = cmd->qw;

    g_target_reached = FALSE;
    g_stop_at_end = cmd->stop_at_end;
    g_received_phone_command = TRUE;
    ctrlManeuverNumSet(WAYPOINT_MODE);
    break;

  case JUST_DRIFT:
    //ctrlManeuverNumSet(DRIFT_MODE);
    SendSOHPacketToPhone();

    ctrlTestTerminate(TEST_RESULT_NORMAL);
    break;
  }
}

void ProcessPhoneStateEstimate(unsigned char channel, unsigned char* buffer, unsigned int len) {

  comm_payload_state_estimate* pkt = (comm_payload_state_estimate*)buffer;
  static state_vector phone_estimate;

  if(ctrlTestNumGet() < 4) {
    return;
  }

  memcpy( &phone_estimate[POS_X],  &pkt->pos[0], 3*sizeof(float) );
  memcpy( &phone_estimate[VEL_X],  &pkt->vel[0], 3*sizeof(float) );
  memcpy( &phone_estimate[QUAT_1], &pkt->quat[0], 4*sizeof(float) );
  memcpy( &phone_estimate[RATE_X], &pkt->rate[0], 3*sizeof(float) );

  padsStateSet(phone_estimate, ctrlTestTimeGet());

  g_state_packets_from_phone++;
}

void DifferentiatePhoneMessage(unsigned char channel,
                               unsigned char* buffer, unsigned int len) {
  het_header* header = (het_header*)buffer;

  dbg_short_packet dbg_error;
  memset(dbg_error,0,sizeof(dbg_short_packet));






  g_packets_from_phone++;

  // check the checksum
  if(!smtChecksumVerify(buffer, len)) {
  
    dbg_error[0] = 111;
  dbg_error[1] = 111;
  dbg_error[2] = 111;
  dbg_error[3] = header->cmd;
  dbg_error[4] = header->cmd;
  dbg_error[5] = header->cmd;
  dbg_error[6] = header->cmd;
  dbg_error[7] = header->cmd;
  dbg_error[8] = header->cmd;
  dbg_error[9] = header->cmd;
  dbg_error[10] = header->cmd;
  dbg_error[11] = 0;
  
    commSendRFMPacket(COMM_CHANNEL_STL, GROUND,
                    COMM_CMD_DBG_SHORT, (unsigned char *) dbg_error, 0);
    return;
  }
  
   
  dbg_error[0] = 222;
  dbg_error[1] = 222;
  dbg_error[2] = 222;
  dbg_error[3] = 222;
  dbg_error[4] = 222;
  dbg_error[5] = 222;
  dbg_error[6] = 222;
  dbg_error[7] = 222;
  dbg_error[8] = 222;
  dbg_error[9] = 222;
  dbg_error[10] = 222;
  dbg_error[11] = 222;
  
    commSendRFMPacket(COMM_CHANNEL_STL, GROUND,
                    COMM_CMD_DBG_SHORT, (unsigned char *) dbg_error, 0);
  

  switch(header->cmd) {
  case PHONE_ESTIMATE_PKT:
    ProcessPhoneStateEstimate(channel, buffer, len);
    break;
  case COMM_COMMAND_FLOAT:
    ProcessPhoneCommandFloat(channel, buffer, len);
    break;
  default:
    break; // don't recognize this
  }
}

void CustomMixWLoc( prop_time *firing_times, float *control, float *state,
                    unsigned int minPulseWidth, float duty_cycle ) {
  float slopeForce;
  float u[12] = {0.0f}, scale, maxu;
  float b2g[3][3]; // body to global rotation matrix
  float MixingMat[6][6] =
    {{0.5f, 0.0f, -0.10362694f, 0.0f, 5.18134715f, 0.0f},
     {0.5f, 0.0f, 0.10362694f, 0.0f, -5.18134715f, 0.0f},
     {0.0f, 0.60362694f, 0.0f, 0.0f, 0.0f, 5.18134715f},
     {0.0f, 0.39637306f, 0.0f, 0.0f, 0.0f, -5.18134715f},
     {0.0f, 0.0f, 0.5f, 5.18134715f, 0.0f, 0.0f},
     {0.0f, 0.0f, 0.5f, -5.18134715f, 0.0f, 0.0f}};
  float controlPair[6] = {0};
  float controlBody[6] = {0};
  int   i, u_over_2, half_width, add_one;
  float thruster_force = 0.0f;
  unsigned int maxPulseWidth = 0;

  maxPulseWidth = (unsigned int) (ctrlPeriodGet() * 0.01 * duty_cycle);
  half_width =  (maxPulseWidth >> 1);

  thruster_force = (duty_cycle*0.01f) * VEHICLE_THRUST_FORCE;

  slopeForce  = ((float) maxPulseWidth)/thruster_force;

  // change forces from global frame to body frame
  mathBody2Global(b2g, state);
  controlBody[0] = b2g[0][0]*control[0] + b2g[1][0]*control[1] + b2g[2][0]*control[2];
  controlBody[1] = b2g[0][1]*control[0] + b2g[1][1]*control[1] + b2g[2][1]*control[2];
  controlBody[2] = b2g[0][2]*control[0] + b2g[1][2]*control[1] + b2g[2][2]*control[2];

  // torques are already in body frame
  controlBody[3] = control[3];
  controlBody[4] = control[4];
  controlBody[5] = control[5];

  // determine thruster pair forces as specified in Mark Hilstad thesis p.38-39
  mathMatVecMult(controlPair, (float **)MixingMat, controlBody, 6, 6);

  // determine thruster duration from thruster pair forces
  for ( i = 0; i < 6; i++ ) {
    if (controlPair[i] > 0) {
      u[i] = slopeForce * controlPair[i];
    } else {
      u[i+6] = - slopeForce * controlPair[i];
    }
  }

  // find maximum thruster command
  maxu = u[0];
  for (i=1; i<12; i++) if (u[i] > maxu) maxu = u[i];

  // scale thruster vector to preserve direction
  if (maxu > ((float) maxPulseWidth))
    scale = ((float) maxPulseWidth)/maxu;
  else
    scale = 1.0;

  // set thruster on-times
  for (i=0; i<12; i++) {
    // thruster on-time in milliseconds
    u[i] = scale*u[i];

    // implement deadband
    if (u[i] < ((float) minPulseWidth)) {
      firing_times->off_time[i] = 0;
      firing_times->on_time[i] = 0;
    }
    // if thruster is on almost full time, make it on full time
    else if ((1.05*u[i]) > ((float) maxPulseWidth)) {
      firing_times->on_time[i]  = 0;
      firing_times->off_time[i] = (int) maxPulseWidth;
    }
    // set remaining thruster on-times and center pulses
    else {
      u_over_2 = (int) (u[i]);
      add_one  = u_over_2 & 0x1;
      u_over_2 = (u_over_2 >> 1);
      firing_times->on_time[i]  = half_width - u_over_2;
      firing_times->off_time[i] = half_width + u_over_2 + add_one;
    }
  }
}

// necessary to compile
void gspProcessRXData() {}
