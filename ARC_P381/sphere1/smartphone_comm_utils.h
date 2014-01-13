/*
 *  smartphone_comm_utils.h
 *
 *
 *  Created by Wheeler, DW  (ARC-TI)[Stinger Ghaffarian Technologies Inc. (SGT Inc.)] on 3/14/12.
 *  Copyright 2012 Intelligent Robotics Group. All rights reserved.
 *
 */

#ifndef SMARTPHONE_COMM_UTILS
#define SMARTPHONE_COMM_UTILS

// Should the commands below be an enum?
#define GO_TO_XYZ       1
#define GO_TO_QUAT      2
#define RELATIVE_XYZ    3
#define RELATIVE_QUAT   4
#define HOLD_POSITION   5
#define JUST_DRIFT      6
#define POS_AND_HOLD    8
#define ORIENT_AND_HOLD 9
#define POS_FROM_PHONE  10
#define COMM_COMMAND_FLOAT (0x40)
#define PHONE_ESTIMATE_PKT (0x41)
#define COMM_PAYLOAD_STATE_EST (0x42)

#define SERIAL_DEVICE_1 1
#define SERIAL_DEVICE_2 2

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


typedef struct{
  unsigned char preamble[4];
  short chk;
  unsigned char cmd;
  unsigned char len;
} het_header;

typedef struct{
  het_header hdr;
  short x; // target X coordinate in m [-0.45,0.45]
  short y; // target Y coordinate in m [-0.80,0.80]
  short z; // target Z coordinate in m [-0.65,0.65]
  short qx; // target Qx
  short qy; // target Qy
  short qz; // target Qz
  short qw; // target Qw
  unsigned short seq_num; // sequence number of this command
  unsigned char stop_at_end; // include zero velocity as termination criteria
  unsigned char cmd; // identifies which command

} phone_cmd;

typedef struct{
  het_header hdr;
  float x; // target X coordinate in m
  float y; // target Y coordinate in m
  float z; // target Z coordinate in m
  float qx; // target Qx
  float qy; // target Qy
  float qz; // target Qz
  float qw; // target Qw
  unsigned short seq_num; // sequence number of this command
  unsigned char stop_at_end; // include zero velocity as termination criteria
  unsigned char cmd; // identifies which command
} phone_cmd_float;

/// Packet payload overlay for COMM_CMD_BACKGROUND and COMM_CMD_SOH_STATE
typedef struct _comm_payload_state_estimate {
  het_header     hdr;
  unsigned int   time_and_id_field;
  float          pos[3];
  float          vel[3];
  float          quat[4];
  float          rate[3];
  unsigned short source;
} comm_payload_state_estimate;


void smtExpV2UARTSendWHETHeader(unsigned char channel, unsigned char len,
                                unsigned char *data, unsigned char cmd);

float smtGetQuaternionMagnitude(float qw);

int smtAtPositionRotation(state_vector error);

int smtAtZeroVelocity(state_vector error);

// rotates quaternion 1 by quaternion 2 and returns as total (xyzw)
void smtRotateByQuaternion(float x2, float y2, float z2, float w2,
                           float x1, float y1, float z1, float w1,
                           float* answer);

int smtChecksumVerify(unsigned char* buffer, unsigned int len);

void smtRotatePhonePositionByQuaternion(float q[4], float position[3],
                                        float res[3]);

void smtFindQDot(float q[4], float rotVel[3], float qdot[4]);

void smtQuatMatrixDerivative(float quat[4], float qdot[4],
                             float matrix[3][3]);


#endif
