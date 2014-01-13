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

// pick ONE
#define LAB_VERSION
//#define ISS_VERSION

#ifdef ISS_VERSION
#define DEFAULT_Z 0.0
#define THE_PROGRAM_NUMBER 410
#else //LAB_VERSION
#define DEFAULT_Z -0.65
#define THE_PROGRAM_NUMBER 223
#endif

#define QX 0
#define QY 1
#define QZ 2
#define QW 3
#define MAX_MANEUVERS 21
#define ESTIMATOR_TIME 10000
#define MANEUVER_TIME_OUT 600000
#define MIN_MANEUVER_TIME 1000

#define BIAS_QX 0.0
#define BIAS_QY 0.0
#define BIAS_QZ 0.0
#define BIAS_QW 1.0

// translation margin could be 0.05 in orbit, but with the sphere sideways, want it bigger
// especially for testing now
// was 0.09
#define TRANSLATION_MARGIN 0.05
#ifdef ISS_VERSION
#define X_MARGIN TRANSLATION_MARGIN
#else // lab version
#define X_MARGIN TRANSLATION_MARGIN
#endif

#define VELOCITY_MARGIN 0.05
// 0.35 rad ~ 20 degrees
#define QUAT_AXIS_MARGIN 0.35
//#define QUAT_ANGLE_MARGIN 0.99
#define RATE_MARGIN 0.1
#define EPSILON 0.01
#define TIMED_OUT 2

#define CHECKOUT 33
#define NOT_CHECKOUT 44

#define CONVERGE_MODE 1
#define DRIFT_MODE 2
#define WAYPOINT_MODE 3

#define USE_SPHERES_ESTIMATE 2
#define USE_PHONE_ESTIMATE 3
#define PHONE_ESTIMATE_PKT (0x41)

#ifndef _TMS320C6X
#define DEBUG(arg)  mexprintf arg
#else
#define DEBUG(arg)
#endif

// GLOBAL VARIABLES

int g_maneuver_nums[MAX_MANEUVERS];
int g_maneuver_num_index;
static int g_test_class;
state_vector g_ctrl_state_target;
state_vector g_cmd_state_target;
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
extern state_vector g_init_state;

// FUNCTION IMPLEMENTATIONS

// callback function prototype
void DifferentiatePhoneMessage(unsigned char channel,
                                  unsigned char* buffer, unsigned int len);

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
  ctrlPeriodSet(1000);

  padsEstimatorInitWaitAndSet(g_init_state, 50, 200, 105,
                              PADS_INIT_THRUST_INT_ENABLE,
                              PADS_BEACONS_SET_1TO9); // ISS

  memset(g_ctrl_state_target,0,sizeof(state_vector));
  g_ctrl_state_target[POS_Z] = DEFAULT_Z;
  g_ctrl_state_target[QUAT_1] = BIAS_QX;
  g_ctrl_state_target[QUAT_2] = BIAS_QY;
  g_ctrl_state_target[QUAT_3] = BIAS_QZ;
  g_ctrl_state_target[QUAT_4] = BIAS_QW;
  memcpy(g_cmd_state_target, g_ctrl_state_target, sizeof(state_vector));

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
      padsEstimatorInitWaitAndSet(g_init_state, 50, 200, 405,
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
      //                padsEstimatorInitWaitAndSet(g_init_state, 50, SYS_FOREVER, SYS_FOREVER,
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
  switch (g_test_class)
    {
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
  switch (g_test_class)
    {
    case CHECKOUT:
      gspTaskRun_Checkout(gsp_task_trigger,extra_data);
      break;
    default:
      break;
    }
}

float getQuaternionMagnitude(float qw) {
  return 2*acos(qw);
}


int atPositionRotation(state_vector error) {
#ifdef ISS_VERSION
  if( (fabs(error[POS_Y]) < TRANSLATION_MARGIN) && (fabs(error[POS_Z]) < TRANSLATION_MARGIN) &&
      (fabs(error[POS_X]) < TRANSLATION_MARGIN) &&
      (fabs(getQuaternionMagnitude(error[QUAT_4])) < QUAT_AXIS_MARGIN))
#else // on ground, only care about roll
    if( (fabs(error[POS_X]) < TRANSLATION_MARGIN) && (fabs(error[POS_Y]) < TRANSLATION_MARGIN) &&
        (fabs(getQuaternionMagnitude(error[QUAT_4])) < QUAT_AXIS_MARGIN))
#endif
      {
        return TRUE;
      } else {
      return FALSE;
    }
}

int atZeroVelocity(state_vector error) {
#ifdef ISS_VERSION
  if( (fabs(error[VEL_X]) < VELOCITY_MARGIN) && (fabs(error[VEL_Y]) < VELOCITY_MARGIN) &&
      (fabs(error[VEL_Z]) < VELOCITY_MARGIN) &&
      (fabs(error[RATE_X]) < RATE_MARGIN) && (fabs(error[RATE_Y]) < RATE_MARGIN) &&
      (fabs(error[RATE_Z]) < RATE_MARGIN) )
#else
    if( (fabs(error[VEL_X]) < VELOCITY_MARGIN) && (fabs(error[VEL_Y]) < VELOCITY_MARGIN) &&
        (fabs(error[RATE_Z]) < RATE_MARGIN) )
#endif
      {
        return TRUE;
      } else {
      return FALSE;
    }
}

void send_SOH_packet_to_phone() {
  comm_payload_soh my_soh;

  // get my SOH information
  commBackgroundSOHGet(SPHERE_ID, &my_soh);

  // set the fields I need
  my_soh.unused[0]              = g_target_reached;
  my_soh.unused[1]              = g_sphere_error;
  my_soh.unused[3]              = (g_last_cmd>>8) & 0xFF; // <<-- is this right?
  my_soh.unused[2]              = g_last_cmd & 0xFF;

  // send it
  expv2_uart_send_w_het_header(EXPv2_CH1_HWID, sizeof(comm_payload_soh),
                               (unsigned char *)&my_soh, COMM_CMD_SOH);
}

// Apply control laws and set thruster on-times. Called periodically.
void gspControl(unsigned int test_number,
                unsigned int test_time,
                unsigned int maneuver_number,
                unsigned int maneuver_time) {
  state_vector ctrlState; // current state vector of the sphere
  state_vector ctrlStateError; // difference btwn ctrlState and
                               // g_ctrl_state_target
  float ctrlControl[6];
  prop_time firing_times;
  const int min_pulse = 10;
  extern const float KPattitudePD, KDattitudePD, KPpositionPD, KDpositionPD;
  dbg_float_packet dbg_target;
  dbg_short_packet dbg_error;

  memset(dbg_error,0,sizeof(dbg_short_packet));
  //Clear all uninitialized vectors
  memset(ctrlControl,0,sizeof(float)*6);
  memset(ctrlStateError,0,sizeof(state_vector));

  memset(dbg_target,0,sizeof(dbg_float_packet));

  //    memset(my_soh,0,sizeof(comm_payload_soh));
  //    global_test_time = test_time;

  padsStateGet(ctrlState);

  send_SOH_packet_to_phone();

  dbg_target[0] = maneuver_time;

  //    commSendRFMPacket(COMM_CHANNEL_STL, GROUND, COMM_CMD_DBG_SHORT, (unsigned char *) dbg_error, 0);
  //    commSendRFMPacket(COMM_CHANNEL_STL, GROUND, COMM_CMD_DBG_FLOAT, (unsigned char *) dbg_target, 0);

  if(g_test_class == CHECKOUT) {
    gspControl_Checkout(test_number, test_time, maneuver_number, maneuver_time);
    return;
  }

  if (maneuver_number == CONVERGE_MODE) //Estimator initialization
    {
      if (test_time >= ESTIMATOR_TIME){
        g_maneuver_num_index++;
        ctrlManeuverNumSet(g_maneuver_nums[g_maneuver_num_index]);
        memcpy(g_ctrl_state_target,ctrlState,sizeof(state_vector));
        g_ctrl_state_target[VEL_X]=0.0f;
        g_ctrl_state_target[VEL_Y]=0.0f;
        g_ctrl_state_target[VEL_Z]=0.0f;
        g_ctrl_state_target[RATE_X]=0.0f;
        g_ctrl_state_target[RATE_Y]=0.0f;
        g_ctrl_state_target[RATE_Z]=0.0f;
        memcpy(g_cmd_state_target,g_ctrl_state_target,sizeof(state_vector));
      }

    } else if (maneuver_number == DRIFT_MODE) {
    // do nothing - just drift!
  } else if (maneuver_number == WAYPOINT_MODE) {
    //Disable estimator during closed loop firing
    padsGlobalPeriodSet(SYS_FOREVER);

    //find error
    if(test_number == USE_PHONE_ESTIMATE) {
      findStateError(ctrlStateError,g_phone_state_estimate,g_ctrl_state_target);
    } else {
      findStateError(ctrlStateError,ctrlState,g_ctrl_state_target);
    }

    //call controllers
    ctrlPositionPDgains(KPpositionPD, KDpositionPD, KPpositionPD,
                        KDpositionPD, KPpositionPD, KDpositionPD,
                        ctrlStateError, ctrlControl);
    ctrlAttitudeNLPDwie(KPattitudePD, KDattitudePD, KPattitudePD,
                        KDattitudePD, KPattitudePD, KDattitudePD,
                        ctrlStateError, ctrlControl);

    //mix forces/torques into thruster commands
    ctrlMixWLoc(&firing_times, ctrlControl, ctrlState,
                min_pulse, 20.0f, FORCE_FRAME_INERTIAL);

    //Set firing times
    propSetThrusterTimes(&firing_times);

    padsGlobalPeriodSetAndWait(200,405);

    // termination conditions
    if(maneuver_time > MIN_MANEUVER_TIME) {
      if ((atPositionRotation(ctrlStateError)))
        {
          // if rotation test, just need x correct
          // if maneuver 20 or not rotation test, need x and x_dot correct
          if((!g_stop_at_end) || (atZeroVelocity(ctrlStateError)))
            {
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
  dbg_error[1] = ctrlStateError[POS_X]*1000;
  dbg_error[2] = ctrlStateError[POS_Y]*1000;
  dbg_error[3] = ctrlStateError[POS_Z]*1000;
  dbg_error[4] = 0;
  dbg_error[5] = ctrlStateError[QUAT_1]*1000;
  dbg_error[6] = ctrlStateError[QUAT_2]*1000;
  dbg_error[7] = ctrlStateError[QUAT_3]*1000;
  dbg_error[8] = ctrlStateError[QUAT_4]*1000;
  dbg_error[9] = fabs(getQuaternionMagnitude(ctrlStateError[QUAT_4]))*1000.0;
  dbg_error[10] = QUAT_AXIS_MARGIN*1000.0;
  dbg_error[11] = g_target_reached;

  send_SOH_packet_to_phone();

  commSendRFMPacket(COMM_CHANNEL_STL, GROUND,
                    COMM_CMD_DBG_FLOAT, (unsigned char *) dbg_target, 0);
  commSendRFMPacket(COMM_CHANNEL_STL, GROUND,
                    COMM_CMD_DBG_SHORT, (unsigned char *) dbg_error, 0);
}

// rotates quaternion 1 by quaternion 2 and returns as total (xyzw)
void gspRotateByQuaternion(float x2, float y2, float z2, float w2,
                           float x1, float y1, float z1, float w1, float* answer) {
  // quats are xyzw
  answer[0] = w1*x2 + x1*w2 + y1*z2 - z1*y2;
  answer[1] = w1*y2 - x1*z2 + y1*w2 + z1*x2;
  answer[2] = w1*z2 + x1*y2 - y1*x2 + z1*w2;
  answer[3] = w1*w2 - x1*x2 - y1*y2 - z1*z2;
}

int checksumChecks(unsigned char* buffer, unsigned int len) {
  int i;
  unsigned short bigcheck=0;

  het_header* hdr = (het_header*) buffer;

  if(len != (8 + hdr->len) )
    return FALSE;

  for(i=8; i<len; i++)
    bigcheck += buffer[i];

  if(bigcheck != hdr->chk)
    return FALSE;

  return TRUE;
}

void ProcessPhoneCommandFloat(unsigned char channel,
                                 unsigned char* buffer, unsigned int len) {
  state_vector ctrlState; // current state vector of the sphere
  phone_cmd_float* cmd = (phone_cmd_float*)buffer;
  float x2, y2, z2, w2;
  float target_quat[4];

  g_cmd_packets_from_phone++;

  if(cmd->seq_num <= g_last_cmd)
    return; // I've seen this command before

  // acknowledge that I received this command correctly
  g_last_cmd = cmd->seq_num;

  if(cmd->cmd == JUST_DRIFT) {
    //ctrlManeuverNumSet(DRIFT_MODE);
    send_SOH_packet_to_phone();

    ctrlTestTerminate(TEST_RESULT_NORMAL);
    //ctrlTestTerminate(1);
    return;
  }

  // find out where we (think we) are
  padsStateGet(ctrlState);

  switch(cmd->cmd) {
  case GO_TO_XYZ:
    // aim for commanded position, and last commanded orientation
    g_cmd_state_target[POS_X] = cmd->x;
    g_cmd_state_target[POS_Y] = cmd->y;
    g_cmd_state_target[POS_Z] = cmd->z;

    g_target_reached = FALSE;
    memcpy( g_ctrl_state_target, g_cmd_state_target,
            sizeof(state_vector));
    g_stop_at_end = cmd->stop_at_end;
    ctrlManeuverNumSet(WAYPOINT_MODE);
    break;

  case GO_TO_QUAT:
    // Updates quat that goes in the commandStateVector. Meaning ...
    // we are going to over-ride all past relative/manual movements.

    // aim for commanded orientation, and last commanded position
    g_cmd_state_target[QUAT_1] = cmd->qx;
    g_cmd_state_target[QUAT_2] = cmd->qy;
    g_cmd_state_target[QUAT_3] = cmd->qz;
    g_cmd_state_target[QUAT_4] = cmd->qw;

    g_target_reached = FALSE;
    memcpy( g_ctrl_state_target, g_cmd_state_target, sizeof(state_vector));
    g_stop_at_end = cmd->stop_at_end;
    ctrlManeuverNumSet(WAYPOINT_MODE);
    break;

  case RELATIVE_XYZ:
    // This operates relative to actual sphere location.

    // aim for current position plus command, and don't change
    // orientation goal
    g_ctrl_state_target[POS_X] = ctrlState[POS_X] + cmd->x;
    g_ctrl_state_target[POS_Y] = ctrlState[POS_Y] + cmd->y;
    g_ctrl_state_target[POS_Z] = ctrlState[POS_Z] + cmd->z;

    g_target_reached = FALSE;
    g_stop_at_end = cmd->stop_at_end;
    ctrlManeuverNumSet(WAYPOINT_MODE);
    break;

  case RELATIVE_QUAT:
    // This operates relative to actual sphere location.

    // aim for current position

    // aim for current orientation plus command, and don't change
    // position goal
    x2 = cmd->qx;
    y2 = cmd->qy;
    z2 = cmd->qz;
    w2 = cmd->qw;

    gspRotateByQuaternion(x2, y2, z2, w2,
                          ctrlState[QUAT_1], ctrlState[QUAT_2],
                          ctrlState[QUAT_3], ctrlState[QUAT_4],
                          target_quat);

    g_ctrl_state_target[QUAT_1] = target_quat[0];
    g_ctrl_state_target[QUAT_2] = target_quat[1];
    g_ctrl_state_target[QUAT_3] = target_quat[2];
    g_ctrl_state_target[QUAT_4] = target_quat[3];

    g_target_reached = FALSE;
    g_stop_at_end = cmd->stop_at_end;
    ctrlManeuverNumSet(WAYPOINT_MODE);
    break;

  case HOLD_POSITION:
    // This tells us to hold actual sphere location

    // We DO need to read the current state for this
    g_ctrl_state_target[POS_X] = ctrlState[POS_X];
    g_ctrl_state_target[POS_Y] = ctrlState[POS_Y];
    g_ctrl_state_target[POS_Z] = ctrlState[POS_Z];

    g_ctrl_state_target[QUAT_1] = ctrlState[QUAT_1];
    g_ctrl_state_target[QUAT_2] = ctrlState[QUAT_2];
    g_ctrl_state_target[QUAT_3] = ctrlState[QUAT_3];
    g_ctrl_state_target[QUAT_4] = ctrlState[QUAT_4];

    g_target_reached = FALSE;
    g_stop_at_end = cmd->stop_at_end;
    ctrlManeuverNumSet(WAYPOINT_MODE);
    break;

  case POS_AND_HOLD:
    // aim for new position, and don't change orientation goal
    g_ctrl_state_target[POS_X] = cmd->x;
    g_ctrl_state_target[POS_Y] = cmd->y;
    g_ctrl_state_target[POS_Z] = cmd->z;

    g_target_reached = FALSE;
    g_stop_at_end = cmd->stop_at_end;
    ctrlManeuverNumSet(WAYPOINT_MODE);
    break;

  case ORIENT_AND_HOLD:
    // aim for new orientation, and don't change position goal
    g_ctrl_state_target[QUAT_1] = cmd->qx;
    g_ctrl_state_target[QUAT_2] = cmd->qy;
    g_ctrl_state_target[QUAT_3] = cmd->qz;
    g_ctrl_state_target[QUAT_4] = cmd->qw;

    g_target_reached = FALSE;
    g_stop_at_end = cmd->stop_at_end;
    ctrlManeuverNumSet(WAYPOINT_MODE);
    break;
  }
}


void rotatePhonePositionByQuaternion(float q[4], float res[3]) {
  // http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm
  float m00, m01, m02, m10, m11, m12, m20, m21, m22;
  float qx, qy, qz, qw;
  float v0, v1, v2;
  qw = q[3];
  qx = q[0];
  qy = q[1];
  qz = q[2];
  v0 = g_phone_pos_in_sphere_coords[0];
  v1 = g_phone_pos_in_sphere_coords[1];
  v2 = g_phone_pos_in_sphere_coords[2];

  m00 = 1 - 2*qy*qy - 2*qz*qz;
  m01 = 2*qx*qy - 2*qz*qw;
  m02 = 2*qx*qz - 2*qy*qw;
  m10 = 2*qx*qy + 2*qz*qw;
  m11 = 1 - 2*qx*qx - 2*qz*qz;
  m12 = 2*qy*qz - 2*qx*qw;
  m20 = 2*qx*qz - 2*qy*qw;
  m21 = 2*qy*qz + 2*qx*qw;
  m22 = 1 - 2*qx*qx - 2*qy*qy;

  res[0] = m00*v0 + m01*v1 + m02*v2;
  res[1] = m10*v0 + m11*v1 + m12*v2;
  res[2] = m20*v0 + m21*v1 + m22*v2;
}

void gspFindQDot(float q[4], float rotVel[3], float qdot[4]) {
  float wx, wy, wz;
  float o00, o01, o02, o03;
  float o10, o11, o12, o13;
  float o20, o21, o22, o23;
  float o30, o31, o32, o33;

  wx = rotVel[0];
  wy = rotVel[1];
  wz = rotVel[2];

  o00 =   0;
  o10 = -wz;
  o20 =  wy;
  o30 = -wx;

  o01 =  wz;
  o11 =   0;
  o21 = -wx;
  o31 = -wy;

  o02 = -wy;
  o12 =  wx;
  o22 =   0;
  o32 = -wz;

  o03 =  wx;
  o13 =  wy;
  o23 =  wz;
  o33 =   0;

  qdot[0] = o00*q[0] + o01*q[1] + o02*q[2] + o03*q[3];
  qdot[1] = o10*q[0] + o11*q[1] + o12*q[2] + o13*q[3];
  qdot[2] = o20*q[0] + o21*q[1] + o22*q[2] + o23*q[3];
  qdot[3] = o30*q[0] + o31*q[1] + o32*q[2] + o33*q[3];
}

void gspQuatMatrixDerivative(float quat[4], float qdot[4], float matrix[3][3]) {
  float qx, qy, qz, qw;
  float qxdot, qydot, qzdot, qwdot;
  float m00dot, m01dot, m02dot, m10dot, m11dot, m12dot, m20dot, m21dot, m22dot;

  qx = quat[0];
  qy = quat[1];
  qz = quat[2];
  qw = quat[3];

  qxdot = qdot[0];
  qydot = qdot[1];
  qzdot = qdot[2];
  qwdot = qdot[3];

  m00dot = -4*(qy*qydot + qz*qzdot);

  m01dot = -2*qz*qwdot - 2*qw*qzdot + 2*qy*qxdot + 2*qx*qydot;

  m02dot = 2*qy*qwdot + 2*qw*qydot + 2*qz*qxdot + 2*qx*qzdot;

  m10dot = 2*qz*qwdot + 2*qw*qzdot + 2*qy*qxdot + 2*qx*qydot;

  m11dot = -4*(qx*qxdot + qz*qzdot);

  m12dot = -2*qx*qwdot - 2*qw*qxdot + 2*qz*qydot + 2*qy*qzdot;

  m20dot =  -2*qy*qwdot - 2*qw*qydot + 2*qz*qxdot + 2*qx*qzdot;

  m21dot = 2*qx*qwdot + 2*qw*qxdot + 2*qz*qydot + 2*qy*qzdot;

  m22dot =  -4*(qx*qxdot + qy*qydot);

  matrix[0][0] = m00dot;
  matrix[0][1] = m01dot;
  matrix[0][2] = m02dot;

  matrix[1][0] = m10dot;
  matrix[1][1] = m11dot;
  matrix[1][2] = m12dot;

  matrix[2][0] = m20dot;
  matrix[2][1] = m21dot;
  matrix[2][2] = m22dot;
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
  rotatePhonePositionByQuaternion(pkt->quat, rotated_position);

  // maybe want to use padsStateSet for debugging (pads_internal.h)
  g_phone_state_estimate[POS_X] = pkt->pos[0] - rotated_position[0];
  g_phone_state_estimate[POS_Y] = pkt->pos[1] - rotated_position[1];
  g_phone_state_estimate[POS_Z] = pkt->pos[2] - rotated_position[2];

  // correct for rotation between phone and sphere
  gspRotateByQuaternion(pkt->quat[0], pkt->quat[1],
                        pkt->quat[2], pkt->quat[3],
                        g_phone_rigid_rot[0], g_phone_rigid_rot[1],
                        g_phone_rigid_rot[2], g_phone_rigid_rot[3],
                        rotated_rotation);

  g_phone_state_estimate[QUAT_1]= rotated_rotation[0];
  g_phone_state_estimate[QUAT_2]= rotated_rotation[1];
  g_phone_state_estimate[QUAT_3]= rotated_rotation[2];
  g_phone_state_estimate[QUAT_4]= rotated_rotation[3];

  // find velocity caused by rotation
  gspFindQDot(pkt->quat, pkt->rate, qdot);

  gspQuatMatrixDerivative(pkt->quat, qdot, rotMatDot);

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
  if(!checksumChecks(buffer, len)) {
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
