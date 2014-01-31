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
// positive 90 about +Y

// Recommended EKF initialization vector
// Sideways coord.: positive 90 about +Y
//state_vector initState = {-0.56f,0.0f,0.00f,  0.0f,0.0f,0.0f,  0.0f,0.707f,0.0f,0.707f,  0.0f,0.0f,0.0f};
// Sideways coord.: unrotated
//state_vector initState = {0.0f,0.0f,0.56f,  0.0f,0.0f,0.0f,  0.0f,0.0f,0.0f,0.0f,  0.0f,0.0f,0.0f};

// table x, phone fwd (up), tank deck
//state_vector initState = {-0.67f,0.0f,0.0f,  0.0f,0.0f,0.0f,  1.0f,0.0f,0.0f,0.0f,  0.0f,0.0f,0.0f};
// table x, phone ovhd, tank in carrage (aft)
state_vector initState = {-0.67f,0.0f,0.0f,  0.0f,0.0f,0.0f,  0.0f,0.71f,0.0f,0.71f,  0.0f,0.0f,0.0f};

// 108 is sphere on its side in the lab
// 106 is sphere upright in the carriage in the lab
const unsigned char PHYS_PROP = 106;

const unsigned int DOCK_MECHANISM_PRESENT	= 0;

// 12.43 + 0.174g smartphone + 0.142g batteries
const float VEHICLE_MASS = 12.75f;    		// [kg] Mass with air carriage  
const float VEHICLE_ROT_ACCEL = 0.124f;		// [rad/s/s] Vehicle Rotational acceleration
const float VEHICLE_LIN_ACCEL = 0.0069f;	// [m/s/s] Vehicle Linear acceleration  
const float VEHICLE_THRUST_FORCE = 0.08798f; 	// [N] VEHICLE_LIN_ACCEL * VEHICLE_MASS
const float VEHICLE_MOMENT_ARM = 0.0965f;       // [m] Thruster Moment Arm 
const float DEFAULT_THRUSTER_FORCE = 0.11f;    	// [N] baseline thruster force  

//const float VEHICLE_THRUST_FORCE_MULTIPLIER = 0.1646f;	// maps efficient thruster force to real thruster force in ctrl_mixer_1 (Pierre's number)
const float VEHICLE_THRUST_FORCE_MULTIPLIER = 0.39f;	// maps efficient thruster force to real thruster force in ctrl_mixer_1 (Simon's number)


// Suggested gains

// PD controller
const float KPattitudePD = 0.0107f; // Proportional gain for attitude control wn=0.400 rad/s
const float KDattitudePD = 0.0402f; // Derivative gain for attitude control wn=0.400 rad/s with 25% damping reduction
const float KPpositionPD = 0.497f; // Proportional gain for position control wn=0.200 rad/s
const float KDpositionPD = 4.972f; // Derivative gain for position control wn=0.200 rad/s

// PID controller (the integral term is set for a time constant of approximately 20 sec)
const float KPattitudePID = 0.0134f; // Proportional gain for attitude control wn=0.400 rad/s
const float KIattitudePID = 0.00054f; // Derivative gain for attitude control wn=0.400 rad/s
const float KDattitudePID = 0.0427f; // Derivative gain for attitude control wn=0.400 rad/s with 25% damping reduction
const float KPpositionPID = 0.746f; // Proportional gain for position control wn=0.200 rad/s
const float KIpositionPID = 0.0249f; // Derivative gain for position control wn=0.200 rad/s
const float KDpositionPID = 5.594f; // Derivative gain for position control wn=0.200 rad/s

