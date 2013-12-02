/*
 *  smartphone_comm_utils.h
 *  
 *
 *  Created by Wheeler, DW  (ARC-TI)[Stinger Ghaffarian Technologies Inc. (SGT Inc.)] on 3/14/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef SMARTPHONE_COMM_UTILS
#define SMARTPHONE_COMM_UTILS

#define GO_TO_XYZ		1
#define GO_TO_QUAT		2
#define RELATIVE_XYZ	3
#define RELATIVE_QUAT	4
#define HOLD_POSITION	5
#define JUST_DRIFT		6
#define POS_AND_HOLD	8
#define ORIENT_AND_HOLD	9
#define POS_FROM_PHONE  10
#define COMM_COMMAND_FLOAT 0x40
#define COMM_PAYLOAD_STATE_EST 0x42

#define SERIAL_DEVICE_1	1
#define SERIAL_DEVICE_2	2

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


void expv2_uart_send_w_het_header(unsigned char channel, unsigned char len, unsigned char *data, unsigned char cmd);	


#endif
