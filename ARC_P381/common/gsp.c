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
//#define LAB_VERSION
#define ISS_VERSION

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

// for debugging
#define SPIRAL_SIZE 0.4
#define BIAS_QX 0.0
#define BIAS_QY 0.0
#define BIAS_QZ 0.0
#define BIAS_QW 1.0
//////////////////

#define CHECKOUT 33
#define NOT_CHECKOUT 44
#define ROTATION_TEST 55

#define CONVERGE_MODE 1
#define DRIFT_MODE 2
#define WAYPOINT_MODE 3

#define USE_PHONE_ESTIMATE 3
#define PHONE_ESTIMATE_PKT (0x40)

#ifndef _TMS320C6X
	#define DEBUG(arg)  mexprintf arg
#else
	#define DEBUG(arg)
#endif
// then write: DEBUG(("  ", ui));

int maneuver_nums[MAX_MANEUVERS];
int maneuver_num_index;

static int testclass;
state_vector ctrlStateTarget;
state_vector phoneStateEstimate;
state_vector commandStateTarget;
char stopAtEnd;
unsigned char global_target_reached = FALSE;
unsigned char global_sphere_error = FALSE;
unsigned short global_last_cmd = 0;
unsigned char global_add_translate = TRUE;
int packetsFromPhone =0;

// callback function prototype
void gspDifferentiatePhoneMessage(unsigned char channel, unsigned char* buffer, unsigned int len);

void gspIdentitySet()
{
   // set the logical identifier (SPHERE#) for this vehicle
   sysIdentitySet(SPHERE_ID);
}


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
   commTdmaEnable(COMM_CHANNEL_STS);
   commTdmaEnable(COMM_CHANNEL_EXP);
   
   // allocate storage space for IMU samples
   padsInertialAllocateBuffers(50);

   // inform system of highest beacon number in use
   padsInitializeFPGA(NUM_BEACONS);

   /* custom program initialization goes below this point */	
   maneuver_num_index = 0;
   
   *SMT335CP4 = 0x1104;// Talk to the Expansion Board at 250 kbps
	
   expv2_init(); // still needed? not in VERTIGO_ExpV2_Testing
   expv2_uart_cbk_register(1,&gspDifferentiatePhoneMessage);
   expv2_uart_baud_set(1,9600);
}

unsigned char cnt;
unsigned char gpio_out[16];
extern state_vector initState;
    	
// static unsigned char sent = 0;
void gspInitTest(unsigned int test_number)
{
	cnt = 0;
    
    //expv2_init();
    memset(gpio_out,0,sizeof(gpio_out));

    #if (SPHERE_ID == SPHERE1)
    	padsEstimatorInitWaitAndSet(initState, 50, 200, 405,
        PADS_INIT_THRUST_INT_ENABLE,PADS_BEACONS_SET_1TO9); // ISS 
    #else
        padsEstimatorInitWaitAndSet(initState, 50, SYS_FOREVER, SYS_FOREVER,
        PADS_INIT_THRUST_INT_ENABLE,PADS_BEACONS_SET_1TO9); // ISS
    #endif

   // propSetThrusterBypassIR(FALSE);
    
    ctrlPeriodSet(1000);
    
    memset(ctrlStateTarget,0,sizeof(state_vector));
	ctrlStateTarget[POS_Z] = DEFAULT_Z;
    ctrlStateTarget[QUAT_1] = BIAS_QX;
    ctrlStateTarget[QUAT_2] = BIAS_QY;
    ctrlStateTarget[QUAT_3] = BIAS_QZ;
    ctrlStateTarget[QUAT_4] = BIAS_QW;
    memcpy(commandStateTarget, ctrlStateTarget, sizeof(state_vector));
    
    maneuver_num_index = 0;
	testclass = NOT_CHECKOUT;
	
    switch (test_number)
    {
    case 1: // Quick Checkout
		testclass = CHECKOUT;
		gspInitTest_Checkout(test_number);
		break;
 	case 2: // use SPHERES estimate
 	case 3: // use Phone estimate
		global_target_reached = FALSE;
		global_sphere_error = FALSE;
		global_last_cmd = 0;
 		maneuver_nums[ 0] =  CONVERGE_MODE;
    	maneuver_nums[ 1] =  WAYPOINT_MODE;//DRIFT_MODE; XXX
    	maneuver_nums[ 2] =  WAYPOINT_MODE;
		stopAtEnd = TRUE;
    	// I don't think the sphere ever needs to terminate the test itself
		break;
	}
}

void gspInitTask()
{
}

void gspPadsInertial(IMU_sample *accel, IMU_sample *gyro, unsigned int num_samples)
{
	switch (testclass)
	{
	case CHECKOUT:
		gspPadsInertial_Checkout(accel, gyro, num_samples);
		break;
	default:
		break;
	}
}

void gspPadsGlobal(unsigned int beacon, beacon_measurement_matrix measurements)
{
}

void gspTaskRun(unsigned int gsp_task_trigger, unsigned int extra_data)
{
	switch (testclass)
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


int atPositionRotation(state_vector error)
{
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

int atZeroVelocity(state_vector error)
{
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
 	comm_payload_telemetry my_position;
 	
	// get my SOH information
	commBackgroundSOHGet(SPHERE_ID, &my_soh);	
	
	// set the fields I need
	my_soh.unused[0]		= global_target_reached;
	my_soh.unused[1]		= global_sphere_error;
	my_soh.unused[3]		= (global_last_cmd>>8) & 0xFF; // <<-- is this right?
	my_soh.unused[2]		= global_last_cmd & 0xFF;
	
	// send it
//	expv2_uart_send_w_het_header(EXPv2_CH1_HWID, sizeof(comm_payload_soh), (unsigned char *)&my_soh, COMM_CMD_SOH);


	commBackgroundPayloadPack(&my_position);
//	expv2_uart_send_w_het_header(EXPv2_CH1_HWID, sizeof(comm_payload_telemetry), (unsigned char *)&my_position, COMM_CMD_BACKGROUND);
}

void gspControl(unsigned int test_number, unsigned int test_time, unsigned int maneuver_number, unsigned int maneuver_time)
{	
    state_vector ctrlState; // current state vector of the sphere
    state_vector ctrlStateError; // difference btwn ctrlState and ctrlStateTarget
    float ctrlControl[6];
    prop_time firing_times;
    const int min_pulse = 10;
    extern const float KPattitudePD, KDattitudePD, KPpositionPD, KDpositionPD;
	dbg_float_packet dbg_target;
    dbg_short_packet dbg_error;
    extern state_vector initState;
	
    //Clear all uninitialized vectors
    memset(ctrlControl,0,sizeof(float)*6);
    memset(ctrlStateError,0,sizeof(state_vector));

    memset(dbg_target,0,sizeof(dbg_float_packet));
    memset(dbg_error,0,sizeof(dbg_short_packet));
    
//    memset(my_soh,0,sizeof(comm_payload_soh));
   
    padsStateGet(ctrlState);
    
    send_SOH_packet_to_phone();

	dbg_target[0] = maneuver_time;

	dbg_error[0] = maneuver_time/1000;

	commSendRFMPacket(COMM_CHANNEL_STL, GROUND, COMM_CMD_DBG_FLOAT, (unsigned char *) dbg_target, 0);
//	commSendRFMPacket(COMM_CHANNEL_STL, GROUND, COMM_CMD_DBG_SHORT, (unsigned char *) dbg_error, 0);
    
/*    if(testclass == CHECKOUT) {
    	gspControl_Checkout(test_number, test_time, maneuver_number, maneuver_time);
    	return;
    }
	
    if (maneuver_number == CONVERGE_MODE) //Estimator initialization
	{
		if (test_time >= ESTIMATOR_TIME){
			maneuver_num_index++;
	    	ctrlManeuverNumSet(maneuver_nums[maneuver_num_index]);
	    	// SPHERE2 must always be in the center
#if (SPHERE_ID == SPHERE1)
			memcpy(ctrlStateTarget,ctrlState,sizeof(state_vector));
			ctrlStateTarget[VEL_X]=0.0f;
			ctrlStateTarget[VEL_Y]=0.0f;
			ctrlStateTarget[VEL_Z]=0.0f;
			ctrlStateTarget[RATE_X]=0.0f;
			ctrlStateTarget[RATE_Y]=0.0f;
			ctrlStateTarget[RATE_Z]=0.0f;
			memcpy(commandStateTarget,ctrlStateTarget,sizeof(state_vector));
#endif
		}
		
	} else if (maneuver_number == DRIFT_MODE) {
		// do nothing - just drift!
	} else if (maneuver_number == WAYPOINT_MODE) {
   	    //Disable estimator during closed loop firing
    	padsGlobalPeriodSet(SYS_FOREVER);

        //find error
        if(test_number == USE_PHONE_ESTIMATE) {
        	findStateError(ctrlStateError,phoneStateEstimate,ctrlStateTarget);
        } else {
	        findStateError(ctrlStateError,ctrlState,ctrlStateTarget);
		}

        //call controllers
        ctrlPositionPDgains(KPpositionPD, KDpositionPD, KPpositionPD,
        	KDpositionPD, KPpositionPD, KDpositionPD, ctrlStateError, ctrlControl);
        ctrlAttitudeNLPDwie(KPattitudePD,KDattitudePD,KPattitudePD,KDattitudePD,KPattitudePD,KDattitudePD,ctrlStateError,ctrlControl);

        //mix forces/torques into thruster commands
        ctrlMixWLoc(&firing_times, ctrlControl, ctrlState, min_pulse, 20.0f, FORCE_FRAME_INERTIAL);

        //Set firing times
        propSetThrusterTimes(&firing_times);

        #if (SPHERE_ID == SPHERE1)
           padsGlobalPeriodSetAndWait(200,405);
        #endif

   		// termination conditions
    	if(maneuver_time > MIN_MANEUVER_TIME) {
    		if ((atPositionRotation(ctrlStateError)))
    		{
    			// if rotation test, just need x correct
    			// if maneuver 20 or not rotation test, need x and x_dot correct
    			if((!stopAtEnd) || (atZeroVelocity(ctrlStateError)))
    			{
    				//  we got there
    				global_target_reached = TRUE;
    			}
			}
		} 
		if(maneuver_time > MANEUVER_TIME_OUT) {
			global_target_reached = TIMED_OUT;
		}	
	}
	
	// sphere2 might not hear the test ended
	#if (SPHERE_ID == SPHERE2)
		commBackgroundSOHGet(SPHERE1, &soh_partner);
		if(soh_partner.test_number == 0) {
			sphereOneIsDead++;
			
		}	
		if(sphereOneIsDead > 2) {
			ctrlTestTerminate(123);
			return;
		}
	#endif
	
	if(global_target_reached==TRUE) {
		dbg_target[0] = 555;
		dbg_target[1] = 555;
		dbg_target[2] = 555;
		dbg_target[3] = 0;
		dbg_target[4] = 0;
		dbg_target[5] = 0;
		dbg_target[6] = 0;
		dbg_target[7] = 555;	
		
	} else {
		
	dbg_target[0] = maneuver_time;
	dbg_target[1] = ctrlStateTarget[POS_X];
	dbg_target[2] = ctrlStateTarget[POS_Y];	
	dbg_target[3] = ctrlStateTarget[POS_Z];
	dbg_target[4] = ctrlStateTarget[QUAT_1];
	dbg_target[5] = ctrlStateTarget[QUAT_2];
	dbg_target[6] = ctrlStateTarget[QUAT_3];
	dbg_target[7] = ctrlStateTarget[QUAT_4];		
 	}
 	
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
	dbg_error[11] = global_target_reached;*/

/*	send_SOH_packet_to_phone();

	commSendRFMPacket(COMM_CHANNEL_STL, GROUND, COMM_CMD_DBG_FLOAT, (unsigned char *) dbg_target, 0);
	commSendRFMPacket(COMM_CHANNEL_STL, GROUND, COMM_CMD_DBG_SHORT, (unsigned char *) dbg_error, 0);*/
}

// rotates quaternion 1 by quaternion 2 and returns as total (xyzw)
void gspRotateByQuaternion(float x2, float y2, float z2, float w2,
						   float x1, float y1, float z1, float w1, float* answer)
{
	// quats are xyzw
	answer[0] = w1*x2 + x1*w2 + y1*z2 - z1*y2;
	answer[1] = w1*y2 - x1*z2 + y1*w2 + z1*x2;
	answer[2] = w1*z2 + x1*y2 - y1*x2 + z1*w2;
	answer[3] = w1*w2 - x1*x2 - y1*y2 - z1*z2;
}

int checksumChecks(unsigned char* buffer, unsigned int len) {
	int i;
	unsigned short bigcheck=0;
	
 //   dbg_short_packet dbg_error;
	phone_cmd* cmd = (phone_cmd*) buffer;
	
	if(len != sizeof(phone_cmd))
		return FALSE;
		
	for(i=8; i<sizeof(phone_cmd); i++)
		bigcheck += buffer[i];
	
//	dbg_error[0] = bigcheck;
//	dbg_error[1] = cmd->hdr.chk;
	
//	commSendRFMPacket(COMM_CHANNEL_STL, GROUND, COMM_CMD_DBG_SHORT_UNSIGNED, (unsigned char *) dbg_error, 0);

	if(bigcheck == cmd->hdr.chk)
		return TRUE;
		
	return FALSE;
}

void gspProcessPhoneCommandFloat(unsigned char channel, unsigned char* buffer, unsigned int len)
{
	state_vector ctrlState; // current state vector of the sphere
	phone_cmd_float* cmd = (phone_cmd_float*)buffer;
	float x2, y2, z2, w2;
	float target_quat[4];
	dbg_float_packet dbg_target;
	
//    dbg_short_packet dbg_error;
    //expv2_uart_send_w_het_header(EXPv2_CH1_HWID, len, buffer, COMM_CMD_SOH);
		
//	commSendRFMPacket(COMM_CHANNEL_STL, GROUND, COMM_CMD_DBG_FLOAT, (unsigned char *) dbg_target, 0);

	// check the checksum
	if(!checksumChecks(buffer, len)) {
		return;
	}

	if(cmd->seq_num <= global_last_cmd)
		return; // I've seen this command before

	// acknowledge that I received this command correctly
	global_last_cmd = cmd->seq_num;

	if(cmd->cmd == JUST_DRIFT)
	{
		//ctrlManeuverNumSet(DRIFT_MODE);
		send_SOH_packet_to_phone();
		
		ctrlTestTerminate(TEST_RESULT_NORMAL);
		//ctrlTestTerminate(1);
		return;
	}
		
	// find out where we (think we) are
	padsStateGet(ctrlState);
	
	switch(cmd->cmd)
	{
		case GO_TO_XYZ:
			// aim for commanded position, and last commanded orientation
			commandStateTarget[POS_X] = cmd->x;
			commandStateTarget[POS_Y] = cmd->y;
			commandStateTarget[POS_Z] = cmd->z;
				
			global_target_reached = FALSE;
			memcpy( ctrlStateTarget, commandStateTarget, sizeof(state_vector));
			stopAtEnd = cmd->stop_at_end;
			ctrlManeuverNumSet(WAYPOINT_MODE);
			break;
				
		case GO_TO_QUAT:
			// Updates quat that goes in the commandStateVector. Meaning ... 
			// we are going to over-ride all past relative/manual movements.

			// aim for commanded orientation, and last commanded position
			commandStateTarget[QUAT_1] = cmd->qx;
			commandStateTarget[QUAT_2] = cmd->qy;
			commandStateTarget[QUAT_3] = cmd->qz;
			commandStateTarget[QUAT_4] = cmd->qw;
			
			global_target_reached = FALSE;
			memcpy( ctrlStateTarget, commandStateTarget, sizeof(state_vector));
			stopAtEnd = cmd->stop_at_end;
			ctrlManeuverNumSet(WAYPOINT_MODE);
			break;

		case RELATIVE_XYZ:
			// This operates relative to actual sphere location.
		
			// aim for current position plus command, and don't change orientation goal
			ctrlStateTarget[POS_X] = ctrlState[POS_X] + cmd->x;
			ctrlStateTarget[POS_Y] = ctrlState[POS_Y] + cmd->y;
			ctrlStateTarget[POS_Z] = ctrlState[POS_Z] + cmd->z;
				
			global_target_reached = FALSE;
			stopAtEnd = cmd->stop_at_end;
			ctrlManeuverNumSet(WAYPOINT_MODE);
			break;
				
		case RELATIVE_QUAT:
			// This operates relative to actual sphere location.
		
			// aim for current position

			// aim for current orientation plus command, and don't change position goal
			x2 = cmd->qx;
			y2 = cmd->qy;
			z2 = cmd->qz;
			w2 = cmd->qw;
			
			gspRotateByQuaternion(x2, y2, z2, w2,						   
						   ctrlState[QUAT_1], ctrlState[QUAT_2], ctrlState[QUAT_3], ctrlState[QUAT_4], 
						   target_quat);

			ctrlStateTarget[QUAT_1] = target_quat[0];
			ctrlStateTarget[QUAT_2] = target_quat[1];
			ctrlStateTarget[QUAT_3] = target_quat[2];
			ctrlStateTarget[QUAT_4] = target_quat[3];

			global_target_reached = FALSE;
			stopAtEnd = cmd->stop_at_end;
			ctrlManeuverNumSet(WAYPOINT_MODE);
			break;
			
		case HOLD_POSITION:
			// This tells us to hold actual sphere location
		
			// We DO need to read the current state for this
			ctrlStateTarget[POS_X] = ctrlState[POS_X];
			ctrlStateTarget[POS_Y] = ctrlState[POS_Y];
			ctrlStateTarget[POS_Z] = ctrlState[POS_Z];
				
			ctrlStateTarget[QUAT_1] = ctrlState[QUAT_1];
			ctrlStateTarget[QUAT_2] = ctrlState[QUAT_2];
			ctrlStateTarget[QUAT_3] = ctrlState[QUAT_3];
			ctrlStateTarget[QUAT_4] = ctrlState[QUAT_4];
			
			global_target_reached = FALSE;
			stopAtEnd = cmd->stop_at_end;
			ctrlManeuverNumSet(WAYPOINT_MODE);
			break;
		
		case POS_AND_HOLD:
			// aim for new position, and don't change orientation goal
			ctrlStateTarget[POS_X] = cmd->x;
			ctrlStateTarget[POS_Y] = cmd->y;
			ctrlStateTarget[POS_Z] = cmd->z;
				
			global_target_reached = FALSE;
			stopAtEnd = cmd->stop_at_end;
			ctrlManeuverNumSet(WAYPOINT_MODE);
			break;
			
		case ORIENT_AND_HOLD:
			// aim for new orientation, and don't change position goal
			ctrlStateTarget[QUAT_1] = cmd->qx;
			ctrlStateTarget[QUAT_2] = cmd->qy;
			ctrlStateTarget[QUAT_3] = cmd->qz;
			ctrlStateTarget[QUAT_4] = cmd->qw;
			
			global_target_reached = FALSE;
			stopAtEnd = cmd->stop_at_end;
			ctrlManeuverNumSet(WAYPOINT_MODE);
			break;
		}
 }

void gspProcessPhoneStateEstimate(unsigned char channel, unsigned char* buffer, unsigned int len) {
		comm_payload_telemetry_float* pkt = (comm_payload_telemetry_float*)buffer;
		dbg_short_packet dbg_error;
		int i;
		memset(dbg_error,0,sizeof(dbg_short_packet));
		
		  
		phoneStateEstimate[POS_X] = pkt->pos[0];
		phoneStateEstimate[POS_Y] = pkt->pos[1];
		phoneStateEstimate[POS_Z] = pkt->pos[2];
		phoneStateEstimate[VEL_X] = pkt->vel[0];
		phoneStateEstimate[VEL_Y] = pkt->vel[1];
		phoneStateEstimate[VEL_Z] = pkt->vel[2];
		phoneStateEstimate[QUAT_1]= pkt->quat[0];
		phoneStateEstimate[QUAT_2]= pkt->quat[1];
		phoneStateEstimate[QUAT_3]= pkt->quat[2];
		phoneStateEstimate[QUAT_4]= pkt->quat[3];
		phoneStateEstimate[RATE_X]= pkt->rate[0];
		phoneStateEstimate[RATE_Y]= pkt->rate[1];
		phoneStateEstimate[RATE_Z]= pkt->rate[2];
		
		packetsFromPhone++;
		for(i=1; i< 12; i++) {
			dbg_error[i] = packetsFromPhone;
		}

	commSendRFMPacket(COMM_CHANNEL_STL, GROUND, COMM_CMD_DBG_SHORT, (unsigned char *) dbg_error, 0);
}

void gspDifferentiatePhoneMessage(unsigned char channel, unsigned char* buffer, unsigned int len) {
	het_header* header = (het_header*)buffer;
	
	switch(header->cmd) {
		case PHONE_ESTIMATE_PKT:
			gspProcessPhoneStateEstimate(channel, buffer, len);
			break;
		default:
			gspProcessPhoneCommandFloat(channel, buffer, len);
			break;
	}
}


// necessary to compile
void gspProcessRXData() {}