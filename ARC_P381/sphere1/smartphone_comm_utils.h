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

#include <commands.h>

// Should the commands below be an enum?
#define GO_TO_XYZ       	1
#define GO_TO_QUAT      	2
#define GO_TO_STATE_VECTOR 11
#define JUST_DRIFT      	6
#define COMM_COMMAND_FLOAT (0x40)
#define PHONE_ESTIMATE_PKT (0x41)
#define COMM_PAYLOAD_STATE_EST (0x42)

#define SERIAL_DEVICE_1 1
#define SERIAL_DEVICE_2 2

// pick ONE
//#define VOMIT_VERSION
//#define ISS_VERSION
#define LAB_VERSION

#ifdef LAB_VERSION
#define DEFAULT_Z -0.79
#define THE_PROGRAM_NUMBER 222
#endif /* LAB_VERSION */

#ifdef ISS_VERSION
#define DEFAULT_Z 0.0
#define THE_PROGRAM_NUMBER 410
#endif /* ISS_VERSION */

#ifdef VOMIT_VERSION
#define DEFAULT_Z 0.0
#define THE_PROGRAM_NUMBER 225
#endif /* VOMIT_VERSION */


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

// translation margin could be 0.05 in orbit, but with the sphere
// sideways, want it bigger. Was 0.09
#define TRANSLATION_MARGIN 0.06
#ifdef ISS_VERSION
#define X_MARGIN TRANSLATION_MARGIN
#else // lab version
#define X_MARGIN TRANSLATION_MARGIN
#endif

#define VELOCITY_MARGIN 0.04
// 0.43633 rad ~ 25 degrees
#define QUAT_AXIS_MARGIN 0.43633232
// 6 degrees per second
#define RATE_MARGIN 0.1
#define EPSILON 0.01
#define TIMED_OUT 2

#define CHECKOUT 33
#define NOT_CHECKOUT 44

#define CONVERGE_MODE 1
#define DRIFT_MODE 2
#define WAYPOINT_MODE 3

typedef struct{
  unsigned char preamble[4];
  unsigned short chk;
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

/// Packet payload overlay for COMM_CMD_BACKGROUND and
/// COMM_CMD_SOH_STATE
typedef struct _comm_payload_state_estimate {
  het_header     hdr;
  unsigned int   timestamp;
  float          pos[3];
  float          vel[3];
  float          quat[4];
  float          rate[3];
} comm_payload_state_estimate;

// Used in SendSOHPacketToPhone
typedef struct _comm_payload_het_soh {
  het_header hdr;
  comm_payload_soh soh;
} comm_payload_het_soh;

// Used in SendTelemetryPacketToPhone
typedef struct _comm_payload_het_telemetry {
  het_header hdr;
  comm_payload_telemetry telemetry;
} comm_payload_het_telemetry;

// Used in SendInertialPacketToPhone
typedef struct _comm_payload_het_inertial {
  het_header hdr;
  unsigned int time;
  unsigned short accel[3];
  unsigned short gyro[3];
} comm_payload_het_inertial;

// Used in SendThrusterTimingsToPhone
typedef struct _comm_payload_het_thruster {
  het_header hdr;
  unsigned int time;
  unsigned char on_time[12];
  unsigned char off_time[12];
} comm_payload_het_thruster;

// WARNING: The data pointer is expected to have space for an
// het_header and then the raw data. Ideally you would be using the
// comm_payload_het structs above.
void smtExpV2UARTSendWHETHeader(unsigned char channel, unsigned char len,
                                unsigned char *data, unsigned char cmd);

float smtGetQuaternionMagnitude(state_vector error);

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
