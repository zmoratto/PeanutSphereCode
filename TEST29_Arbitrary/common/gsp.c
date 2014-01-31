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

//C:\Documents and Settings\Administrator\Desktop\spheres\TestProjects2\ISS_TS029\CombinedCameraTest_091511\common
/*----------------------------------------------------------------------------*/
/*                         Do not modify this section.                        */
/*----------------------------------------------------------------------------*/

#include "comm.h"
#include "commands.h"
#include "control.h"
#include "gsp.h"
#include "gsp_task.h"
#include "pads.h"
#include "prop.h"
#include "spheres_constants.h"
#include "spheres_physical_parameters.h"
#include "spheres_types.h"
#include "std_includes.h"
#include "system.h"
#include "util_memory.h"
#include "math.h"
#include "math_matrix.h"
/*----------------------------------------------------------------------------*/
/*                     Modify as desired below this point.                    */
/*----------------------------------------------------------------------------*/
//#include "mex.h"
#include "ctrl_attitude.h"
#include "ctrl_position.h"
#include "find_state_error.h"
#include "ctrl_mix.h"
#include <string.h>
#include "gsutil_checkout.h"

// pick ONE
#define LAB_VERSION
//#define ISS_VERSION

#ifdef ISS_VERSION
#define BEACON_X 0.70358
#define DEFAULT_X 0.0
#define THE_PROGRAM_NUMBER 331
#define BIAS_Q1 1.0
#define BIAS_Q2 0.0
#define BIAS_Q3 0.0
#define BIAS_Q4 0.0
#else //LAB_VERSION
#define BEACON_X DEFAULT_X
#define DEFAULT_X 0
#define DEFAULT_Z -0.7
#define BIAS_Q1 0.0
#define BIAS_Q2 0.0
#define BIAS_Q3 0.0
#define BIAS_Q4 1.0
#define THE_PROGRAM_NUMBER 1117
#endif

#define QX 0
#define QY 1
#define QZ 2
#define QW 3
#define MAX_MANEUVERS 21
#define ESTIMATOR_TIME 10000
#define MANEUVER_TIME_OUT 30000

#define TRANSLATION_MARGIN 0.2
#define VELOCITY_MARGIN 0.05
#define QUAT_AXIS_MARGIN 0.25
#define QUAT_ANGLE_MARGIN 0.99
#define RATE_MARGIN 0.2
#define EPSILON 0.01

// this is beacon 5
#define BEACON_Y 1.0414
#define BEACON_Z 0.93167


// X: 0.71
// Y: 1.04
// Z: 0.95
#define X_EXTENT 0.45
#define Y_EXTENT 0.8
#define Z_EXTENT 0.65

#define CHECKOUT 33
#define NOT_CHECKOUT 44
#define ROTATION_TEST 55

#ifndef _TMS320C6X
	#define DEBUG(arg)  mexPrintf arg
#else
	#define DEBUG(arg)
#endif
// DEBUG(("  ", ui));

int maneuver_nums[MAX_MANEUVERS];
int maneuver_num_index;

static int testclass;

void gspIdentitySet()
{
   // set the logical identifier (SPHERE#) for this vehicle
   sysIdentitySet(SPHERE_ID);
}

void gspInitProgram()
{
   // set the unique program identifier (to be assigned by MIT)
   sysProgramIDSet(THE_PROGRAM_NUMBER);
   // this is the Camera Test, 2D version

   // set up communications TDMA frames
   commTdmaStandardInit(COMM_CHANNEL_STL, sysIdentityGet(), NUM_SPHERES);
   commTdmaStandardInit(COMM_CHANNEL_STS, sysIdentityGet(), NUM_SPHERES);

   // enable communications channels
   commTdmaEnable(COMM_CHANNEL_STL);
   commTdmaEnable(COMM_CHANNEL_STS);

   // allocate storage space for IMU samples
   padsInertialAllocateBuffers(50);

   // inform system of highest beacon number in use
   padsInitializeFPGA(NUM_BEACONS);

   /* custom program initialization goes below this point */
	maneuver_num_index = 0;
}

state_vector ctrlStateTarget;

void gspInitTest(unsigned int test_number)
{
	extern state_vector initState;
	
    #if (SPHERE_ID == SPHERE1)
    	if (test_number > 1){
	    	padsEstimatorInitWaitAndSet(initState, 50, 200, 105,
	        PADS_INIT_THRUST_INT_ENABLE,PADS_BEACONS_SET_1TO9); // ISS 
	    }
    #else
    	if (test_number > 1) {
	        padsEstimatorInitWaitAndSet(initState, 50, SYS_FOREVER, SYS_FOREVER,
	        PADS_INIT_THRUST_INT_ENABLE,PADS_BEACONS_SET_1TO9); // ISS
        }
    #endif

    ctrlPeriodSet(1000);
    memset(ctrlStateTarget,0,sizeof(state_vector));
    maneuver_num_index = 0;
	testclass = NOT_CHECKOUT;
	
    switch(test_number) {
		case 1: // Quick Checkout
			testclass = CHECKOUT;
			gspInitTest_Checkout(test_number);
			break;
    	case 2: // 3D translations 
			maneuver_nums[0] =  1; // estimator convergence
			maneuver_nums[1] =  3; // center
    		
    		maneuver_nums[2] =  2; // 1m +x
    		maneuver_nums[3] =  3; // recenter
    		maneuver_nums[4] =  4; // 1m +y
    		maneuver_nums[5] =  3; // recenter
			maneuver_nums[6] =  5; // 1m +z
    		maneuver_nums[7] = 20; // recenter and stop
    		
    		maneuver_nums[8] = 15; // terminate
            // Simulator test end time = 96 sec
    		break;
    	case 3: // 3D rotations
    		maneuver_nums[ 0] =  1; // estimator convergence
    		maneuver_nums[ 1] =  3; // center
    			
    		maneuver_nums[ 2] =  6; //  90 deg about X axis
    		maneuver_nums[ 3] =  7; // 180 deg about X axis
    		maneuver_nums[ 4] =  8; // 270 deg about X axis
    		maneuver_nums[ 5] =  3; // recenter
    		
    		maneuver_nums[ 6] =  9; //  90 deg about Y axis
    		maneuver_nums[ 7] = 10; // 180 deg about Y axis
    		maneuver_nums[ 8] = 11; // 270 deg about Y axis
    		maneuver_nums[ 9] =  3; // recenter
    		
    		maneuver_nums[10] = 12; //  90 deg about Z axis
    		maneuver_nums[11] = 13; // 180 deg about Z axis
    		maneuver_nums[12] = 14; // 270 deg about Z axis
    		maneuver_nums[13] = 20; // recenter and stop
    		
    		maneuver_nums[14] = 15; // terminate
			testclass = ROTATION_TEST;
            // Simulator test end time = 97 sec
    		break;
   		case 4: // 3D move while looking at target
    		maneuver_nums[0] = 1; // estimator convergence
    		maneuver_nums[1] = 3; // center
    		
    		maneuver_nums[2] = 16; // center, viewing target
    		maneuver_nums[3] = 17; // out along +X, viewing target
    		maneuver_nums[4] = 18; // out along +Y, viewing target
    		maneuver_nums[5] = 19; // out along +Z, viewing target
    		maneuver_nums[6] = 16; // center, viewing target
    		maneuver_nums[7] = 20; // center, face forward, stop
    		
    		maneuver_nums[8] = 15; // terminate
            // Simulator test end time = 93 sec
    		break;	
    	case 5: // scan -X wall along Y
    		maneuver_nums[0] = 1; // estimator convergence
    		maneuver_nums[1] = 3; // center
    		maneuver_nums[2] = 21; // out to -x, +y
    		maneuver_nums[3] = 22; // across to -x, -y 
    		maneuver_nums[4] = 20; // back to center, stop
    		maneuver_nums[5] = 15; // terminate
            // Simulator test end time = 64 sec
			break;
    	case 6: // scan -X wall along Z
    		maneuver_nums[0] = 1; // estimator convergence
    		maneuver_nums[1] = 3; // center
    		maneuver_nums[2] = 23; // out to -x, -z
    		maneuver_nums[3] = 24; // across to -x, +z
    		maneuver_nums[4] = 20; // back to center, stop
    		maneuver_nums[5] = 15; // terminate
            // Simulator test end time = 69 sec
			break;
		case 7: // scan +X wall along Y
    		maneuver_nums[0] = 1; // estimator convergence
    		maneuver_nums[1] = 3; // center
    		maneuver_nums[2] = 25; // out to +x, +y
    		maneuver_nums[3] = 26; // across to +x, -y 
    		maneuver_nums[4] = 20; // back to center, stop
    		maneuver_nums[5] = 15; // terminate
            // Simulator test end time = 67 sec
			break;
		case 8: // scan +X wall along Z
    		maneuver_nums[0] = 1; // estimator convergence
    		maneuver_nums[1] = 3; // center
    		maneuver_nums[2] = 27; // out to +x, -z
    		maneuver_nums[3] = 28; // across to +x, +z
    		maneuver_nums[4] = 20; // back to center, stop
    		maneuver_nums[5] = 15; // terminate
            // Simulator test end time = 77 sec
			break;
			
		case 9: // X translations
	    	maneuver_nums[0] =  1; // estimator convergence
	    	maneuver_nums[1] =  3; // center
	    	
    		maneuver_nums[2] =  2; // 1m +x
    		maneuver_nums[3] = 20; // recenter
    		maneuver_nums[4] = 15; // terminate
    		break;
    	case 10: //(a) Y translations
    		maneuver_nums[0] =  1; // estimator convergence
    		maneuver_nums[1] =  3; // center
    		
    		maneuver_nums[2] =  4; // 1m +y
    		maneuver_nums[3] = 20; // recenter
    		maneuver_nums[4] = 15; // terminate
    		break;
    	case 11: //(b) Z translations
    		maneuver_nums[0] =  1; // estimator convergence
    		maneuver_nums[1] =  3; // center
    		
    		maneuver_nums[2] =  5; // 1m +z
    		maneuver_nums[3] = 20; // recenter
    		maneuver_nums[4] = 15;// terminate
    		break;
    	case 12: //(c) rotations about X
    		maneuver_nums[0] =  1; // estimator convergence
    		maneuver_nums[1] =  3; // center
    		
    		maneuver_nums[2] =  6; //  90 deg about X axis
    		maneuver_nums[3] =  7; // 180 deg about X axis
    		maneuver_nums[4] =  8; // 270 deg about X axis
    		maneuver_nums[5] = 20; // recenter and stop
    		maneuver_nums[6] = 15; // terminate
			testclass = ROTATION_TEST;
    		break;
    	case 13: //(d) rotations about Y
    		maneuver_nums[0] =  1; // estimator convergence
    		maneuver_nums[1] =  3; // center
    	
    		maneuver_nums[2] =  9; //  90 deg about Y axis
    		maneuver_nums[3] = 10; // 180 deg about Y axis
    		maneuver_nums[4] = 11; // 270 deg about Y axis
    		maneuver_nums[5] = 20; // recenter and stop
    		testclass = ROTATION_TEST;

    		maneuver_nums[6] = 15; // terminate
    		break;
    	case 14: // (e) rotations about Z
			maneuver_nums[0] =  1; // estimator convergence
			maneuver_nums[1] =  3; // center
			
    		maneuver_nums[2] = 12; //  90 deg about X axis
    		maneuver_nums[3] = 13; // 180 deg about X axis
    		maneuver_nums[4] = 14; // 270 deg about X axis
    		maneuver_nums[5] = 20; // recenter and stop
    		maneuver_nums[6] = 15; // terminate
			testclass = ROTATION_TEST;
    		break;
 		case 15: // (f) move to +x while looking at target
    		maneuver_nums[0] = 1; // estimator convergence
    		maneuver_nums[1] = 3; // center
    		
    		maneuver_nums[2] = 16; // center, viewing target
    		maneuver_nums[3] = 17; // out along +X, viewing target
    		maneuver_nums[4] = 16; // center, viewing target
    		maneuver_nums[5] = 20; // center, face forward, stop
    		
    		maneuver_nums[6] = 15; // terminate
            // Simulator test end time = 62 sec
    		break;
	case 16: // (g) move to +y while looking at target
    		maneuver_nums[0] = 1; // estimator convergence
    		maneuver_nums[1] = 3; // center
    		
    		maneuver_nums[2] = 16; // center, viewing target
    		maneuver_nums[3] = 18; // out along +Y, viewing target
    		maneuver_nums[4] = 16; // center, viewing target
    		maneuver_nums[5] = 20; // center, face forward, stop
    		
    		maneuver_nums[6] = 15; // terminate
            // Simulator test end time = 56 sec
    		break;
	case 17: // (h) move to +z while looking at target
    		maneuver_nums[0] = 1; // estimator convergence
    		maneuver_nums[1] = 3; // center
    		
    		maneuver_nums[2] = 16; // center, viewing target
    		maneuver_nums[3] = 19; // out along +Z, viewing target
    		maneuver_nums[4] = 16; // center, viewing target
    		maneuver_nums[5] = 20; // center, face forward, stop
    		
    		maneuver_nums[6] = 15; // terminate
            // Simulator test end time = 59 sec
    		break;
    case 18: // (i) 2D translations and rotations
    		maneuver_nums[ 0] =  1; // estimator convergence
    		maneuver_nums[ 1] =  3; // center
    	
    		maneuver_nums[ 2] =  2; // 1m +x
    		maneuver_nums[ 3] =  3; // recenter
    		maneuver_nums[ 4] =  4; // 1m +y
    		maneuver_nums[ 5] =  3; // recenter
    		
    		maneuver_nums[ 6] = 12; //  90 deg about Z axis
    		maneuver_nums[ 7] = 13; // 180 deg about Z axis
    		maneuver_nums[ 8] = 14; // 270 deg about Z axis
    		maneuver_nums[ 9] = 20; // recenter and stop
    		
    		maneuver_nums[10] = 15; // terminate

			testclass = ROTATION_TEST;
    		break;
    case 19: // (j) 2D translations
			maneuver_nums[0] =  1; // estimator convergence
			maneuver_nums[1] =  3; // center
    		
    		maneuver_nums[2] =  4; // 0.5m +y
    		maneuver_nums[3] =  3; // recenter
    		maneuver_nums[4] =  5; // 0.5m +z
    		maneuver_nums[5] =  3; // recenter
    		
    		maneuver_nums[6] = 15; // terminate
    		break;
    case 20: // (k) 2D rotations
    		maneuver_nums[0] =  1; // estimator convergence
    		maneuver_nums[1] =  3; // center
    		
    		maneuver_nums[2] = 12; //  90 deg about Z axis
    		maneuver_nums[3] = 13; // 180 deg about Z axis
    		maneuver_nums[4] = 14; // 270 deg about Z axis
    		maneuver_nums[5] = 20; // recenter and stop
    		
    		maneuver_nums[6] = 15; // terminate
			testclass = ROTATION_TEST;
    		break;
   	case 21: // (l) 2D move while looking at target
    		maneuver_nums[0] = 1; // estimator convergence
    		maneuver_nums[1] = 3; // center
    		
    		maneuver_nums[2] = 16; // center, viewing target
    		maneuver_nums[3] = 18; // out along +Y, viewing target
    		maneuver_nums[4] = 19; // out along +Z, viewing target
    		maneuver_nums[5] = 16; // center, viewing target
    		maneuver_nums[6] = 20; // center, face forward, stop
    		maneuver_nums[7] = 15; // terminate
    		break;  		
	case 22: // (m) 2D diamond
			maneuver_nums[0] =  1; // estimator convergence
			maneuver_nums[1] =  3; // center
    		
    		maneuver_nums[2] =  4; // 0.5m +y
    		maneuver_nums[4] =  5; // 0.5m +z
    		maneuver_nums[5] =  3; // recenter
    		
    		maneuver_nums[6] = 15; // terminate
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

// returns a normalized vector for axis and return value is angle in  Radians
float quaternionToAngleAxis(float quat[], float axis[])
{
  float angle;
  float qw = quat[QW];
  if(fabs(qw - 1.0) < 0.01)
  {
   angle = 0.0f;
   axis[0] = 0.0f;
   axis[1] = 0.0f;
   axis[2] = 1.0f;
  }
  angle = (float)(2.0 * acos(qw));
  axis[0] = quat[QX] / sqrt(1-qw*qw);
  axis[1] = quat[QY] / sqrt(1-qw*qw);
  axis[2] = quat[QZ] / sqrt(1-qw*qw);
  mathVecNormalize(axis, 3);

  return angle;
}

// x,y,z is the axis
// theta is in RADIANS
void angleAxisToQuaternion(float x, float y, float z, float theta,  
float retQuat[])
{
  	int i;
  	float xyz[] ={ 0.0f, 0.0f, 0.0f};
    float thetaByTwo = theta/2.0;
    float sumOfSquares = 0;
    float divisor;
    xyz[0] = x;
    xyz[1] = y;
    xyz[2] = z;

     mathVecNormalize(xyz, 3);

     retQuat[QX] = xyz[0] * sin(thetaByTwo);
     retQuat[QY] = xyz[1] * sin(thetaByTwo);
     retQuat[QZ] = xyz[2] * sin(thetaByTwo);
     retQuat[QW] = cos(thetaByTwo);

     // now normalize
     for(i=0; i<4; i++)
       sumOfSquares += retQuat[i] * retQuat[i];

     divisor = sqrt(sumOfSquares);

     for(i=0; i<4; i++)
       retQuat[i] /= divisor;
}


// vec - length 3
// quat - length 4
// res - length 3
void rotateVectorByQuaternion(float vec[], float quat[], float res[])
{
	//use Pout = q * Pin * conj(q)

  float vecQuat[] = { 0.0f, 0.0f, 0.0f, 0.0f};
  float quatConj[] = { 0.0f, 0.0f, 0.0f, 0.0f};
  float temp[] = { 0.0f, 0.0f, 0.0f, 0.0f};
  float qRes[] = { 0.0f, 0.0f, 0.0f, 0.0f};
  mathVecNormalize(vec, 3);

  vecQuat[0] = vec[0];
  vecQuat[1] = vec[1];
  vecQuat[2] = vec[2];
  vecQuat[3] = 0.0;

  quatConj[0] = -quat[0];
  quatConj[1] = -quat[1];
  quatConj[2] = -quat[2];
  quatConj[3] =  quat[3];

  quatMult(temp, vecQuat, quatConj);
  quatMult(qRes, quat, temp);
  
  res[0] = qRes[0];
  res[1] = qRes[1];
  res[2] = qRes[2];

  mathVecNormalize(res,3);
}

// target: xyz of point to look at (with -X)
// currPos: xyz of current sphere location
// retQuat: the direction the sphere needs to point
void lookAt(float target[], float currPos[], float retQuat[])
{
  // O = sphere axes aligned with lab axes
  // B = pointing to the target
  // C = B, but with top of camera pointing up (ie in the goal position, retQuat)
  float vectorToTarget[] = { 0.0f, 0.0f, 0.0f}; // S_-x when at B
  float fromOtoB_Angle;
  float fromOtoB_Axis[] = { 0.0f, 0.0f, 0.0f};
  float fromOtoB_Quat[] = { 0.0f, 0.0f, 0.0f, 0.0f};
  float plusZatB[] = { 0.0f, 0.0f, 0.0f};

  // projection matrix to sphere's yz plane at B
  float vvT[3][3] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  float I[3][3] = { {1, 0, 0}, {0, 1, 0}, {0, 0, 1} };
  float proj[3][3] = { {0, 0, 0}, {0, 0, 0}, {0, 0, 0} };
  float projectedZ[] = { 0.0f, 0.0f, 0.0f};
  float fromBtoC_Angle;
  float fromBtoC_Quat[] = { 0.0f, 0.0f, 0.0f, 0.0f};
  float negativeX[3] = {-1.0f, 0.0f, 0.0f};
  float positiveZ[3] = { 0.0f, 0.0f, 1.0f};
  float cameraUp[3] = { 0.0f, 0.0f, -1.0f}; // negative Z

  // vector from current spheres position to the target
  mathVecSubtract(vectorToTarget, target, currPos, 3);
  mathVecNormalize(vectorToTarget, 3);

  // find the axis perpendicular to current direction and target direction
  mathVecCross(fromOtoB_Axis, negativeX, vectorToTarget);
  // find rotation about that axis to point
  // (both vectors are already normalized)
  fromOtoB_Angle = mathVecInner(negativeX, vectorToTarget, 3);
  fromOtoB_Angle = acos(fromOtoB_Angle);

  // make the quaternion that moves from O to B
  angleAxisToQuaternion(fromOtoB_Axis[0], fromOtoB_Axis[1], fromOtoB_Axis[2],
   fromOtoB_Angle, fromOtoB_Quat);

  // almost the answer, but the camera's "up" is arbitrary
  // make the projection matrix
  mathVecOuter((float**)vvT,vectorToTarget,vectorToTarget,3,3);
  mathVecSubtract((float*)proj, (float*)I, (float*)vvT, 9);
  mathMatVecMult(projectedZ, (float**)proj, positiveZ, 3, 3);

  // now, where is the rotated camera +Z pointing?
  // the positiveZ vector is the camera's "up"
  // so without doing anything, "up" when pointing at target is:
  rotateVectorByQuaternion(cameraUp, fromOtoB_Quat, plusZatB);
  // but I want it to be: projectedZ
  // I already know that I will be rotating around vectorToTarget
  // at B, vectorToTarget is negativeX
  // how far do I need to rotate?
  fromBtoC_Angle = mathVecInner(plusZatB, projectedZ, 3);
  fromBtoC_Angle = acos(fromBtoC_Angle);

  // make the quaternion
  angleAxisToQuaternion(negativeX[0], negativeX[1], negativeX[2],
   fromBtoC_Angle, fromBtoC_Quat);
  // so, our answer is fromOtoB_Quat * fromBtoC_Quat
  quatMult(retQuat, fromOtoB_Quat, fromBtoC_Quat);
}

int atPositionRotation(state_vector error)
{
	if( (fabs(error[POS_Y]) < TRANSLATION_MARGIN) && (fabs(error[POS_X]) < TRANSLATION_MARGIN) &&
#ifdef ISS_VERSION
		(fabs(error[POS_Z]) < TRANSLATION_MARGIN) &&
#endif
    	(fabs(error[QUAT_1]) < QUAT_AXIS_MARGIN) && (fabs(error[QUAT_2]) < QUAT_AXIS_MARGIN) &&
        (fabs(error[QUAT_3]) < QUAT_AXIS_MARGIN) && (fabs(error[QUAT_4]) > QUAT_ANGLE_MARGIN ))
    {
		return TRUE;
	} else {
		return FALSE;
	}
}

int atZeroVelocity(state_vector error)
{
	if( (fabs(error[VEL_X]) < VELOCITY_MARGIN) && (fabs(error[VEL_Y]) < VELOCITY_MARGIN) &&
		(fabs(error[VEL_Z]) < VELOCITY_MARGIN) &&
    	(fabs(error[RATE_X]) < RATE_MARGIN) && (fabs(error[RATE_Y]) < RATE_MARGIN) &&
        (fabs(error[RATE_Z]) < RATE_MARGIN) )
    {
		return TRUE;
	} else {
		return FALSE;
	}
}

void gspControl(unsigned int test_number, unsigned int test_time, 
unsigned int maneuver_number, unsigned int maneuver_time)
{
    state_vector ctrlState; // current state vector of the sphere
    state_vector ctrlStateError; // difference btwn ctrlState and ctrlStateTarget
    float ctrlControl[6];
    prop_time firing_times;
    const int min_pulse = 10;
    extern const float KPattitudePD, KDattitudePD, KPpositionPD, KDpositionPD;
    float rotation[] = { 0.0f, 0.0f, 0.0f, 0.0f };
    // these are to flip the sphere
//    float upsidedown[] = { 0.0f, 0.0f, 0.0f, 0.0f}; // to flip the sphere's quaternion
    float temp[] = { 0.0f, 0.0f, 0.0f, 0.0f }; // for unflipped quaternions
    float target[] = { 0.0f, 0.0f, 0.0f }; // point we want the Smartphone to look at
    float bias_quat[] = { BIAS_Q1, BIAS_Q2, BIAS_Q3, BIAS_Q4 };
    float flipAboutZ[] = { 0.0f, 0.0f, 1.0f, 0.0f };
    
    float dbgNegX[] = { -1.0f, 0.0f, 0.0f };
    float dbgPosZ[] = { 0.0f, 0.0f, 1.0f };
    float dbgAnswer[] = { 0.0f, 0.0f, 0.0f };
    int i;
    
    //bias_quat corrects for offset of environment coordinate system
    dbg_float_packet dbg_target;
    dbg_short_packet dbg_error;
    extern state_vector initState;
    
    //Clear all uninitialized vectors
    memset(ctrlControl,0,sizeof(float)*6);
    memset(ctrlStateError,0,sizeof(state_vector));
    memset(ctrlStateTarget,0,sizeof(state_vector));
    memset(dbg_target,0,sizeof(dbg_float_packet));
    memset(dbg_error,0,sizeof(dbg_short_packet));
    
    padsStateGet(ctrlState);
    
    target[0] = BEACON_X;
    target[1] = BEACON_Y;
    target[2] = BEACON_Z;
    
    if (testclass==CHECKOUT) {
    	gspControl_Checkout(test_number, test_time, maneuver_number, maneuver_time);
    	return;
    } 
    
    switch(maneuver_number){
    	case 1: //Estimator initialization
        	if (test_time >= ESTIMATOR_TIME){
            	maneuver_num_index++;
            	ctrlManeuverNumSet(maneuver_nums[maneuver_num_index]);
    			memcpy(ctrlStateTarget,ctrlState,sizeof(state_vector));
    			ctrlStateTarget[VEL_X]=0.0f;
    			ctrlStateTarget[VEL_Y]=0.0f;
    			ctrlStateTarget[VEL_Z]=0.0f;
    			ctrlStateTarget[RATE_X]=0.0f;
    			ctrlStateTarget[RATE_Y]=0.0f;
    			ctrlStateTarget[RATE_Z]=0.0f;
    		}
            break;
        case 2: // 1m +x
            ctrlStateTarget[POS_X] = X_EXTENT;
            ctrlStateTarget[POS_Y] = 0.0f;
            ctrlStateTarget[POS_Z] = DEFAULT_Z;
            for(i=0; i<4; i++)
	            rotation[i] = bias_quat[i];
            break;
        case 3: // recenter
            ctrlStateTarget[POS_X] = 0.0f;
            ctrlStateTarget[POS_Y] = 0.0f;
            ctrlStateTarget[POS_Z] = DEFAULT_Z;
            for(i=0; i<4; i++)
	            rotation[i] = bias_quat[i];
            break;
        case 4: // 1m +y
            ctrlStateTarget[POS_X] = 0.0f;
            ctrlStateTarget[POS_Y] = Y_EXTENT;
            ctrlStateTarget[POS_Z] = DEFAULT_Z;
            for(i=0; i<4; i++)
	            rotation[i] = bias_quat[i];
            break;
        case 5: // 1m +z
            ctrlStateTarget[POS_X] = 0.0f;
            ctrlStateTarget[POS_Y] = 0.0f;
            ctrlStateTarget[POS_Z] = Z_EXTENT;
            for(i=0; i<4; i++)
	            rotation[i] = bias_quat[i];
            break;
        case 6: // 90 about X
            memset(ctrlStateTarget,0,sizeof(state_vector));
            angleAxisToQuaternion(1,0,0, PI/2.0, temp);
            quatMult(rotation, temp, bias_quat);
            break;
        case 7: // 180 about X
            memset(ctrlStateTarget,0,sizeof(state_vector));
            angleAxisToQuaternion(1,0,0, PI, temp);
            quatMult(rotation, temp, bias_quat);
            break;
        case 8: // 270 about X
            memset(ctrlStateTarget,0,sizeof(state_vector));
            angleAxisToQuaternion(1,0,0, 3.0*PI/2.0, temp);
            quatMult(rotation, temp, bias_quat);
            break;
        case 9: // 90 about Y
            memset(ctrlStateTarget,0,sizeof(state_vector));
            angleAxisToQuaternion(0,1,0, PI/2.0, temp);
            quatMult(rotation, temp, bias_quat);
            break;
        case 10: // 180 about Y
            memset(ctrlStateTarget,0,sizeof(state_vector));
            angleAxisToQuaternion(0,1,0, PI, temp);
            quatMult(rotation, temp, bias_quat);
            break;
        case 11: // 270 about Y
            memset(ctrlStateTarget,0,sizeof(state_vector));
            angleAxisToQuaternion(0,1,0, 3.0*PI/2.0, temp);
            quatMult(rotation, temp, bias_quat);
            break;
    case 12: // 90 about Z
            memset(ctrlStateTarget,0,sizeof(state_vector));
            angleAxisToQuaternion(0,0,1, PI/2.0, temp);
            quatMult(rotation, temp, bias_quat);
            break;
        case 13: // 180 about Z
            memset(ctrlStateTarget,0,sizeof(state_vector));
            angleAxisToQuaternion(0,0,1, PI, temp);
            quatMult(rotation, temp, bias_quat);
            break;
        case 14: // 270 about Z
            memset(ctrlStateTarget,0,sizeof(state_vector));
            angleAxisToQuaternion(0,0,1, 3.0*PI/2.0, temp);
            quatMult(rotation, temp, bias_quat);
            break;
        case 15: // terminate
    		DEBUG(("Simulator test end time = %d sec\n", test_time/1000));
        	ctrlTestTerminate(TEST_RESULT_NORMAL);
            break;
        case 16: // origin, looking at target set above
    		ctrlStateTarget[POS_X] = 0.0f;
            ctrlStateTarget[POS_Y] = 0.0f;
            ctrlStateTarget[POS_Z] = DEFAULT_Z;
    		lookAt(target, &ctrlState[POS_X], rotation);
    		break;
    	case 17: // out to +X, looking at target
    		ctrlStateTarget[POS_X] = 0.0f;
            ctrlStateTarget[POS_Y] = 0.0f;
            ctrlStateTarget[POS_Z] = DEFAULT_Z;
    		lookAt(target, &ctrlState[POS_X], rotation);
    		break;
    	case 18: // out to +Y, looking at target
    		ctrlStateTarget[POS_X] = 0.0f;
            ctrlStateTarget[POS_Y] = Y_EXTENT;
            ctrlStateTarget[POS_Z] = DEFAULT_Z;
    		lookAt(target, &ctrlState[POS_X], rotation);
    		break;
		case 19: // out to +Z, looking at target
    		ctrlStateTarget[POS_X] = 0.0f;
            ctrlStateTarget[POS_Y] = 0.0f;
            ctrlStateTarget[POS_Z] = Z_EXTENT;
    		lookAt(target, &ctrlState[POS_X], rotation);
    		break;
    case 20: // recenter
            ctrlStateTarget[POS_X] = 0.0f;
            ctrlStateTarget[POS_Y] = 0.0f;
            ctrlStateTarget[POS_Z] = DEFAULT_Z;
            for(i=0; i<4; i++)
	            rotation[i] = bias_quat[i];
            break;
        case 21: // out to -X, +Y
            ctrlStateTarget[POS_X] = -X_EXTENT;
            ctrlStateTarget[POS_Y] = Y_EXTENT;
            ctrlStateTarget[POS_Z] = 0.0f;
            for(i=0; i<4; i++)
	            rotation[i] = bias_quat[i];
            break;
        case 22: // out to -X, -Y
        	ctrlStateTarget[POS_X] = -X_EXTENT;
            ctrlStateTarget[POS_Y] = -Y_EXTENT;
            ctrlStateTarget[POS_Z] = 0.0f;
            for(i=0; i<4; i++)
	            rotation[i] = bias_quat[i];
        	break;
 		case 23: // out to -X, -Z
            ctrlStateTarget[POS_X] = -X_EXTENT;
            ctrlStateTarget[POS_Y] = 0.0f;
            ctrlStateTarget[POS_Z] = -Z_EXTENT;
            for(i=0; i<4; i++)
	            rotation[i] = bias_quat[i];
            break;
        case 24: // out to -X, +Z
        	ctrlStateTarget[POS_X] = -X_EXTENT;
            ctrlStateTarget[POS_Y] = 0.0f;
            ctrlStateTarget[POS_Z] = Z_EXTENT;
            for(i=0; i<4; i++)
	            rotation[i] = bias_quat[i];
        	break;
        case 25: // out to +X, +Y, pointing at +X
	        // need to find the quaternion that points camera to +X
            ctrlStateTarget[POS_X] = X_EXTENT;
            ctrlStateTarget[POS_Y] = Y_EXTENT;
            ctrlStateTarget[POS_Z] = 0.0f;
            quatMult(rotation, flipAboutZ, bias_quat);
            break;
        case 26: // out to +X, -Y, pointing at +X
        	ctrlStateTarget[POS_X] = X_EXTENT;
            ctrlStateTarget[POS_Y] = -Y_EXTENT;
            ctrlStateTarget[POS_Z] = 0.0f;
            quatMult(rotation, flipAboutZ, bias_quat);
        	break;
 		case 27: // out to +X, -Z, pointing at +X
            ctrlStateTarget[POS_X] = X_EXTENT;
            ctrlStateTarget[POS_Y] = 0.0f;
            ctrlStateTarget[POS_Z] = -Z_EXTENT;
            quatMult(rotation, flipAboutZ, bias_quat);
            break;
        case 28: // out to +X, +Z, pointing at +X
        	ctrlStateTarget[POS_X] = X_EXTENT;
            ctrlStateTarget[POS_Y] = 0.0f;
            ctrlStateTarget[POS_Z] = Z_EXTENT;
            quatMult(rotation, flipAboutZ, bias_quat);
        	break;
    }

  	if(maneuver_number>1) {
        ctrlStateTarget[QUAT_1] = rotation[QX];
        ctrlStateTarget[QUAT_2] = rotation[QY];
        ctrlStateTarget[QUAT_3] = rotation[QZ];
        ctrlStateTarget[QUAT_4] = rotation[QW];
    }
    
    if (maneuver_number > 1) {
	
	    //Disable estimator during closed loop firing
    	padsGlobalPeriodSet(SYS_FOREVER);
        //find error
        findStateError(ctrlStateError,ctrlState,ctrlStateTarget);
        
        //call controllers
        ctrlPositionPDgains(KPpositionPD, KDpositionPD, KPpositionPD,
        	KDpositionPD, KPpositionPD, KDpositionPD, ctrlStateError, ctrlControl);
        ctrlAttitudeNLPDwie(KPattitudePD,KDattitudePD,KPattitudePD,KDattitudePD,KPattitudePD,KDattitudePD,ctrlStateError,ctrlControl);

        //mix forces/torques into thruster commands
        ctrlMixWLoc(&firing_times, ctrlControl, ctrlState, min_pulse, 20.0f, FORCE_FRAME_INERTIAL);
        //Set firing times
        propSetThrusterTimes(&firing_times);

        #if (SPHERE_ID == SPHERE1)
           padsGlobalPeriodSetAndWait(200,205);
        #endif
    }

    // termination conditions
    // (maneuver_time > ESTIMATOR_TIME)
    if ((atPositionRotation(ctrlStateError)))
    {
    	// if rotation test, just need x correct
    	// if maneuver 20 or not rotation test, need x and x_dot correct
    	if(((testclass == ROTATION_TEST) && (maneuver_number != 20)) || 
    	   (atZeroVelocity(ctrlStateError)))
    	{
    		DEBUG(("Position = %f, %f, %f\n", ctrlState[POS_X], ctrlState[POS_Y], ctrlState[POS_Z]));
    		DEBUG(("Rotation = %f, %f, %f, %f\n", 
    			ctrlState[0],
    			ctrlState[1],
    			ctrlState[2],
    			ctrlState[3]));
    		
    		// Figure out which way the camera is pointing.
    		// Originally it points down (-X, 0, 0)
    		// so multiply that by the quat target
    		rotateVectorByQuaternion(dbgNegX, rotation, dbgAnswer);
    		DEBUG(("\tCamera points %f, %f, %f\n", dbgAnswer[0], dbgAnswer[1], dbgAnswer[2]));
    		
    		rotateVectorByQuaternion(dbgPosZ, rotation, dbgAnswer);
    		DEBUG(("\tCamera up points %f, %f, %f\n", dbgAnswer[0], dbgAnswer[1], dbgAnswer[2]));
    			
    		DEBUG(("Maneuver end conditions hit for maneuver %d\n\n", maneuver_number));
	    	maneuver_num_index++;
	    	ctrlManeuverNumSet(maneuver_nums[maneuver_num_index]);
    	}
  	}
  
	// send debug information
	dbg_target[0] = maneuver_time;
	dbg_target[1] = ctrlStateTarget[POS_X];
	dbg_target[2] = ctrlStateTarget[POS_Y];
	dbg_target[3] = ctrlStateTarget[POS_Z];
	dbg_target[4] = ctrlStateTarget[QUAT_1];
	dbg_target[5] = ctrlStateTarget[QUAT_2];
	dbg_target[6] = ctrlStateTarget[QUAT_3];
	dbg_target[7] = ctrlStateTarget[QUAT_4];
	
	dbg_error[0] = maneuver_time/1000;
	dbg_error[1] = ctrlStateError[POS_X]*1000;
	dbg_error[2] = ctrlStateError[POS_Y]*1000;
	dbg_error[3] = ctrlStateError[POS_Z]*1000;
	dbg_error[4] = ctrlStateError[VEL_X]*1000;
	dbg_error[5] = ctrlStateError[VEL_Y]*1000;
	dbg_error[6] = ctrlStateError[VEL_Z]*1000;	
	dbg_error[7] = ctrlStateError[QUAT_1]*1000;
	dbg_error[8] = ctrlStateError[QUAT_2]*1000;
	dbg_error[9] = ctrlStateError[QUAT_3]*1000;
	dbg_error[10] = ctrlStateError[QUAT_4]*1000;
	dbg_error[11] = ctrlStateError[RATE_X]*1000;
	dbg_error[12] = ctrlStateError[RATE_Y]*1000;
	dbg_error[13] = ctrlStateError[RATE_Z]*1000;


	commSendPacket(COMM_CHANNEL_STL, GROUND, 0, COMM_CMD_DBG_FLOAT, (unsigned char *) dbg_target, 0);
	commSendPacket(COMM_CHANNEL_STL, GROUND, 0, COMM_CMD_DBG_SHORT, (unsigned char *) dbg_error, 0);
}

void gspProcessRXData()
{
}
