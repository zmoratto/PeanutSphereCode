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
#include "commands.h"
#include "comm_process_rx_packet.h"
#include "control.h"
#include "gsp.h"
#include "gsp_task.h"
#include "housekeeping.h"
#include "pads.h"
#include "pads_internal.h"
#include "prop.h"
#include "spheres_constants.h"
#include "spheres_physical_parameters.h"
#include "spheres_types.h"
#include "std_includes.h"
#include "system.h"
#include "util_memory.h"
#include "smt335async.h"
#include "exp_v2.h"
#include <string.h>


/*----------------------------------------------------------------------------*/
/*                     Modify as desired below this point.                    */
/*----------------------------------------------------------------------------*/
#include "smartphone_comm_utils.h"
#include "math.h"


// GLOBAL VARIABLES

int g_maneuver_nums[MAX_MANEUVERS];
int g_maneuver_num_index;
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
state_vector g_phone_state_estimate;
float g_phone_pos_in_sphere_coords[3] = {0.15, 0.0, 0.0};
float g_phone_rigid_rot[4] = {0.0, 0.0, 0.0, 1.0};

// FORWARD DECLARATIONS
void DifferentiatePhoneMessage(unsigned char channel,
                               unsigned char* buffer, unsigned int len);
void SendSOHPacketToPhone();
void SendEstimatePacketToPhone(unsigned int test_number);

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

  switch (test_number)
    {
    case 1: // Quick Checkout
      g_test_class = CHECKOUT;
      gspInitTest_Checkout(test_number);
      break;
    case USE_SPHERES_ESTIMATE: // use SPHERES estimate
      g_target_reached = FALSE;
      g_sphere_error = FALSE;
      g_last_cmd = 0;
      g_maneuver_nums[ 0] =  CONVERGE_MODE;
      g_maneuver_nums[ 1] =  WAYPOINT_MODE;//DRIFT_MODE; XXX
      g_maneuver_nums[ 2] =  WAYPOINT_MODE;
      g_stop_at_end = TRUE;

      // turn on the estimator
      padsEstimatorInitWaitAndSet(initState, 50, 200, 405,
                                  PADS_INIT_THRUST_INT_ENABLE,PADS_BEACONS_SET_1TO9); // ISS
      break;
    case USE_PHONE_ESTIMATE: // use Phone estimate
      g_target_reached = FALSE;
      g_sphere_error = FALSE;
      g_last_cmd = 0;
      g_maneuver_nums[ 0] =  CONVERGE_MODE;
      g_maneuver_nums[ 1] =  WAYPOINT_MODE;//DRIFT_MODE; XXX
      g_maneuver_nums[ 2] =  WAYPOINT_MODE;
      g_stop_at_end = TRUE;

      // don't turn on the estimator
      // not sure if I actually need both of these
      //                padsEstimatorInitWaitAndSet(initState, 50, SYS_FOREVER, SYS_FOREVER,
      //        PADS_INIT_THRUST_INT_ENABLE,PADS_BEACONS_SET_1TO9); // ISS

      padsEstimatorDisable();
      break;
    }
}

// Specify task trigger mask
void gspInitTask() {
}

// Perform state estimate based on inertial data. Called periodically.
void gspPadsInertial(IMU_sample *accel, IMU_sample *gyro,
                     unsigned int num_samples) {
  switch (g_test_class) {
  case CHECKOUT:
    gspPadsInertial_Checkout(accel, gyro, num_samples);
    break;
  default:
    break;
  }
}

// Record global data. Called at the end of each beacon's transmission
// period.
void gspPadsGlobal(unsigned int beacon,
                   beacon_measurement_matrix measurements) {
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
  state_vector curr_state; // current state vector of the sphere
  state_vector ctrl_state_error; // difference btwn curr_state and
  // g_ctrl_state_target
  float ctrl_control[6];
  prop_time firing_times;
  const int min_pulse = 10;
  extern const float KPattitudePD, KDattitudePD, KPpositionPD, KDpositionPD;
  dbg_float_packet dbg_target;
  dbg_short_packet dbg_error;

  // Clear all uninitialized vectors
  memset(ctrl_control,0,sizeof(float)*6);
  memset(ctrl_state_error,0,sizeof(state_vector));
  memset(dbg_error,0,sizeof(dbg_short_packet));
  memset(dbg_target,0,sizeof(dbg_float_packet));

  // Get an estimate for where we are
  if ( test_number == USE_PHONE_ESTIMATE ) {
    memcpy(curr_state, g_phone_state_estimate, sizeof(state_vector));
  } else {
    padsStateGet(curr_state);
  }

  // Send out a telemetry packet that shows what we're operating on.
  SendEstimatePacketToPhone(test_number);

  // Check for early exit condition
  if( g_test_class == CHECKOUT ) {
    SendSOHPacketToPhone();
    gspControl_Checkout(test_number, test_time, maneuver_number, maneuver_time);
    return;
  }

  if (maneuver_number == CONVERGE_MODE) { //Estimator initialization
    if (test_time >= ESTIMATOR_TIME){
      // If it seems that it is about time that the estimator should
      // have converged. Then we'll command the sphere to hold current
      // position.
      g_maneuver_num_index++;
      ctrlManeuverNumSet(g_maneuver_nums[g_maneuver_num_index]);
      memcpy(g_ctrl_state_target, curr_state, sizeof(state_vector));
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

#ifdef LAB_VERSION
    // Don't bother trying to control the sphere in translation Z or
    // rotation about X, Y, since we are on a granite table.
    ctrl_control[FORCE_Z] = 0.0;
    ctrl_control[TORQUE_X] = 0.0;
    ctrl_control[TORQUE_Y] = 0.0;
#endif

    //mix forces/torques into thruster commands
    ctrlMixWLoc(&firing_times, ctrl_control, curr_state,
                min_pulse, 20.0f, FORCE_FRAME_INERTIAL);

    //Set firing times
    propSetThrusterTimes(&firing_times);

    padsGlobalPeriodSetAndWait(200,405);

    // termination conditions
    if(maneuver_time > MIN_MANEUVER_TIME) {
      if ((smtAtPositionRotation(ctrl_state_error))) {
        // if rotation test, just need x correct
        // if maneuver 20 or not rotation test, need x and x_dot correct
        if((!g_stop_at_end) || (smtAtZeroVelocity(ctrl_state_error))) {

          //  we got there
          g_target_reached = TRUE;
        }
      }
    }
    if(maneuver_time > MANEUVER_TIME_OUT) {
      g_target_reached = TIMED_OUT;
    }
  }

  dbg_target[0] = maneuver_time;
  dbg_target[1] = g_ctrl_state_target[POS_X];
  dbg_target[2] = g_ctrl_state_target[POS_Y];
  dbg_target[3] = g_ctrl_state_target[POS_Z];
  dbg_target[4] = g_ctrl_state_target[QUAT_1];
  dbg_target[5] = g_ctrl_state_target[QUAT_2];
  dbg_target[6] = g_ctrl_state_target[QUAT_3];
  dbg_target[7] = g_ctrl_state_target[QUAT_4];

  dbg_error[0] = maneuver_time/1000;
  dbg_error[1] = ctrl_state_error[POS_X]*1000;
  dbg_error[2] = ctrl_state_error[POS_Y]*1000;
  dbg_error[3] = ctrl_state_error[POS_Z]*1000;
  dbg_error[4] = 0;
  dbg_error[5] = ctrl_state_error[QUAT_1]*1000;
  dbg_error[6] = ctrl_state_error[QUAT_2]*1000;
  dbg_error[7] = ctrl_state_error[QUAT_3]*1000;
  dbg_error[8] = ctrl_state_error[QUAT_4]*1000;
  dbg_error[9] = fabs(smtGetQuaternionMagnitude(ctrl_state_error))*1000.0;
  dbg_error[10] = QUAT_AXIS_MARGIN*1000.0;
  dbg_error[11] = g_target_reached;

  SendSOHPacketToPhone();

  commSendRFMPacket(COMM_CHANNEL_STL, GROUND,
                    COMM_CMD_DBG_FLOAT, (unsigned char *) dbg_target, 0);
  commSendRFMPacket(COMM_CHANNEL_STL, GROUND,
                    COMM_CMD_DBG_SHORT, (unsigned char *) dbg_error, 0);
}

void SendEstimatePacketToPhone(unsigned int test_number) {
  comm_payload_state_estimate my_state;
  state_vector curr_state;

  // Get an estimate for where we are
  if ( test_number == USE_PHONE_ESTIMATE ) {
    memcpy(curr_state, g_phone_state_estimate, sizeof(state_vector));
  } else {
    padsStateGet(curr_state);
  }

  // Fill the packet
  my_state.timestamp = sysSphereTimeGet();
  memcpy( &my_state.pos[0], &curr_state[POS_X], 3*sizeof(float) );
  memcpy( &my_state.vel[0], &curr_state[VEL_X], 3*sizeof(float) );
  memcpy( &my_state.quat[0], &curr_state[QUAT_1], 4*sizeof(float) );
  memcpy( &my_state.rate[0], &curr_state[RATE_X], 3*sizeof(float) );

  // Send it out. There is some weirdness happening here since
  // comm_payload_state_estimate includes space for SPHERE/HET header
  // when the next function call is going to generate that for me.
  smtExpV2UARTSendWHETHeader
    (EXPv2_CH1_HWID,
     sizeof(comm_payload_state_estimate) - sizeof(het_header),
     (unsigned char*)&my_state + sizeof(het_header),
     0x42);
}

void SendSOHPacketToPhone() {
  comm_payload_soh my_soh;

  // get my SOH information
  commBackgroundSOHGet(SPHERE_ID, &my_soh);

  // set the fields I need
  my_soh.unused[0] = g_target_reached;
  my_soh.unused[1] = g_sphere_error;
  my_soh.unused[3] = (g_last_cmd>>8) & 0xFF;
  my_soh.unused[2] = g_last_cmd & 0xFF;

  // send it
  smtExpV2UARTSendWHETHeader(EXPv2_CH1_HWID, sizeof(comm_payload_soh),
                             (unsigned char *)&my_soh, COMM_CMD_SOH);
}

void ProcessPhoneCommandFloat(unsigned char channel,
                              unsigned char* buffer, unsigned int len) {
  phone_cmd_float* cmd = (phone_cmd_float*)buffer;

  g_cmd_packets_from_phone++;

  // TODO: This looks dangerous. What happens when this wraps due to a
  // lot of commands?
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
  float rotated_position[3] = {0.0, 0.0, 0.0};
  float rotated_rotation[4] = {0.0, 0.0, 0.0, 0.0};
  float qdot[4] = {0.0, 0.0, 0.0, 0.0};
  float rotMatDot[3][3] = {{0.0, 0.0, 0.0},{0.0, 0.0, 0.0},{0.0, 0.0, 0.0}};
  float velFromRot[3] = {0.0, 0.0, 0.0};

  if(ctrlTestNumGet() != USE_PHONE_ESTIMATE) {
    return;
  }

  // correct for offset between phone and sphere
  smtRotatePhonePositionByQuaternion(pkt->quat, g_phone_pos_in_sphere_coords,
                                     rotated_position);

  // maybe want to use padsStateSet for debugging (pads_internal.h)
  g_phone_state_estimate[POS_X] = pkt->pos[0] - rotated_position[0];
  g_phone_state_estimate[POS_Y] = pkt->pos[1] - rotated_position[1];
  g_phone_state_estimate[POS_Z] = pkt->pos[2] - rotated_position[2];

  // correct for rotation between phone and sphere
  smtRotateByQuaternion(pkt->quat[0], pkt->quat[1],
                        pkt->quat[2], pkt->quat[3],
                        g_phone_rigid_rot[0], g_phone_rigid_rot[1],
                        g_phone_rigid_rot[2], g_phone_rigid_rot[3],
                        rotated_rotation);

  g_phone_state_estimate[QUAT_1]= rotated_rotation[0];
  g_phone_state_estimate[QUAT_2]= rotated_rotation[1];
  g_phone_state_estimate[QUAT_3]= rotated_rotation[2];
  g_phone_state_estimate[QUAT_4]= rotated_rotation[3];

  // find velocity caused by rotation
  smtFindQDot(pkt->quat, pkt->rate, qdot);

  smtQuatMatrixDerivative(pkt->quat, qdot, rotMatDot);

  velFromRot[0] =
    rotMatDot[0][0] * g_phone_pos_in_sphere_coords[0] +
    rotMatDot[0][1] * g_phone_pos_in_sphere_coords[1] +
    rotMatDot[0][2] * g_phone_pos_in_sphere_coords[2];
  velFromRot[1] =
    rotMatDot[1][0] * g_phone_pos_in_sphere_coords[0] +
    rotMatDot[1][1] * g_phone_pos_in_sphere_coords[1] +
    rotMatDot[1][2] * g_phone_pos_in_sphere_coords[2];
  velFromRot[2] =
    rotMatDot[2][0] * g_phone_pos_in_sphere_coords[0] +
    rotMatDot[2][1] * g_phone_pos_in_sphere_coords[1] +
    rotMatDot[2][2] * g_phone_pos_in_sphere_coords[2];

  g_phone_state_estimate[VEL_X] = pkt->vel[0] - velFromRot[0];
  g_phone_state_estimate[VEL_Y] = pkt->vel[1] - velFromRot[1];
  g_phone_state_estimate[VEL_Z] = pkt->vel[2] - velFromRot[2];

  // will be identical for phone and sphere because they are rigidly connected
  g_phone_state_estimate[RATE_X]= pkt->rate[0]; // phi-dot, about Xphone
  g_phone_state_estimate[RATE_Y]= pkt->rate[1];
  g_phone_state_estimate[RATE_Z]= pkt->rate[2]; // theta-dot, about Zphone

  padsStateSet(g_phone_state_estimate, ctrlTestTimeGet());

  g_state_packets_from_phone++;
}

void DifferentiatePhoneMessage(unsigned char channel,
                               unsigned char* buffer, unsigned int len) {
  het_header* header = (het_header*)buffer;

  g_packets_from_phone++;

  // check the checksum
  if(!smtChecksumVerify(buffer, len)) {
    return;
  }

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

// necessary to compile
void gspProcessRXData() {}
