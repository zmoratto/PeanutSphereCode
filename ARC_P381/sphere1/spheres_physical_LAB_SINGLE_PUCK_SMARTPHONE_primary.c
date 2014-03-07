/*
 * spheres_physical_LAB_SINGLE_PUCK_SMARTPHONE.c
 *
 * File written to include the vehicles properties 
 *
 * This file is for the LAB environment
 *	
 * The mass properties involve the following hardware:
 * - Sphere
 * - Single puck air carriage
 * - Smartphone attached on the side of the Sphere
 *
 * Not included:  counter weight of the docking port					
 */
 
#include "spheres_types.h"
#include "smartphone_comm_utils.h"
// positive 90 about +Y


#ifdef LAB_VERSION
state_vector initState = {0.0f,0.0f,-0.79f,  0.0f,0.0f,0.0f,  0.0f,0.0f,0.0f,1.0f,  0.0f,0.0f,0.0f};
// 106 - normal -z face - 96 same but "ISS" (to use w/ flight gui)
const unsigned char PHYS_PROP = 106;  
#endif


const unsigned int DOCK_MECHANISM_PRESENT	= 0;

// Zack Moratto 1/19/14
// SPHERE + MM = 5.32 kg
// Carriage    = 8.42 kg
// TOTAL       = 13.74 kg
const float VEHICLE_MASS = 13.74f; 
const float VEHICLE_ROT_ACCEL = 0.124f;		// [rad/s/s] Vehicle Rotational acceleration
const float VEHICLE_LIN_ACCEL = 0.0069f;	// [m/s/s] Vehicle Linear acceleration  
const float VEHICLE_THRUST_FORCE = 0.08798f; 	// [N] VEHICLE_LIN_ACCEL * VEHICLE_MASS
const float VEHICLE_MOMENT_ARM = 0.0965f;       // [m] Thruster Moment Arm 
const float DEFAULT_THRUSTER_FORCE = 0.11f;    	// [N] baseline thruster force  

//const float VEHICLE_THRUST_FORCE_MULTIPLIER = 0.1646f;	// maps efficient thruster force to real thruster force in ctrl_mixer_1 (Pierre's number)
const float VEHICLE_THRUST_FORCE_MULTIPLIER = 0.39f;	// maps efficient thruster force to real thruster force in ctrl_mixer_1 (Simon's number)


// Suggested gains

//// TS029 PD controller
//const float KPattitudePD = 0.0036f; // Proportional gain for attitude control wn=0.400 rad/s
//const float KDattitudePD = 0.0135f; // Derivative gain for attitude control wn=0.400 rad/s with 25% damping reduction
//const float KPpositionPD = 0.172f; // Proportional gain for position control wn=0.200 rad/s
//const float KDpositionPD = 1.720f; // Derivative gain for position control wn=0.200 rad/s


// PD controller
// original gains (used this in lab)
const float KPpositionPD = 0.497f; // Proportional gain for position control wn=0.200 rad/s
const float KDpositionPD = 4.97f;

const float KPattitudePD = 0.0107f; // Proportional gain for attitude control wn=0.400 rad/s
const float KDattitudePD = 0.0402f; // Derivative gain for attitude control wn=0.400 rad/s with 25% damping reduction

// PID controller (the integral term is set for a time constant of approximately 20 sec)
const float KPattitudePID = 0.0134f; // Proportional gain for attitude control wn=0.400 rad/s
const float KIattitudePID = 0.00054f; // Derivative gain for attitude control wn=0.400 rad/s
const float KDattitudePID = 0.0427f; // Derivative gain for attitude control wn=0.400 rad/s with 25% damping reduction
const float KPpositionPID = 0.746f; // Proportional gain for position control wn=0.200 rad/s
const float KIpositionPID = 0.0249f; // Derivative gain for position control wn=0.200 rad/s
const float KDpositionPID = 5.594f; // Derivative gain for position control wn=0.200 rad/s

