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
#include "math_matrix.h"
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

  *SMT335CP4 = 0x1104;// Talk to the Expansion Board at 250 kbps

  expv2_init(); // still needed? not in VERTIGO_ExpV2_Testing
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

  switch (test_number)
    {
    case 1: // Quick Checkout
      gspInitTest_Checkout(test_number);
      break;
    default:
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
}

// Record global data. Called at the end of each beacon's transmission
// period.
void gspPadsGlobal(unsigned int beacon,
                   beacon_measurement_matrix measurements) {
}

// Event driven task for estimation, control, and
// communications. Called whenever a masked event occurs.
void gspTaskRun(unsigned int gsp_task_trigger, unsigned int extra_data) {
}

// Apply control laws and set thruster on-times. Called periodically.
void gspControl(unsigned int test_number,
                unsigned int test_time,
                unsigned int maneuver_number,
                unsigned int maneuver_time) {
  static unsigned int step = 20;
  static unsigned int repeat = 0;
  static prop_time firing_times;
  static float ctrl_control[6];
  static state_vector curr_state = {0};
  int i = 0;

  // Make sure the state is something realistic.
  curr_state[QUAT_4] = 1;

  //Disable estimator during closed loop firing
  padsGlobalPeriodSet(SYS_FOREVER);

  for ( i = 0; i < 12; i++ ) {
    firing_times.on_time[i] = 0;
    firing_times.off_time[i] = 0;
  }
  for ( i = 0; i < 6; i++ ) {
    ctrl_control[i] = 0;
  }

  if ( step < 12) {
    firing_times.off_time[step] = 200;
  } else if ( repeat == 0 ) {
    if ( step == 12 ) {
      // Torque about X
      ctrl_control[3] = 10;
    } else if ( step == 13 ) {
      // Torque about -X
      ctrl_control[3] = -10;
    } else if ( step == 14 ) {
      // Torque about Y
      ctrl_control[4] = 10;
    } else if ( step == 15 ) {
      // Torque about -Y
      ctrl_control[4] = -10;
    } else if ( step == 16 ) {
      // Torque about Z
      ctrl_control[5] = 10;
    } else if ( step == 17 ) {
      // Torque about -Z
      ctrl_control[5] = -10;
    } else if ( step == 18 ) {
      // Force on X
      ctrl_control[0] = 10;
    } else if ( step == 19 ) {
      // Force on -X
      ctrl_control[0] = -10;
    } else if ( step == 20 ) {
      // Force on Y
      ctrl_control[1] = 10;
    } else if ( step == 21 ) {
      // Force on -Y
      ctrl_control[1] = -10;
    } else if ( step == 22 ) {
      // Force on Z
      ctrl_control[2] = 10;
    } else if ( step == 23 ) {
      // Force on -Z
      ctrl_control[2] = -10;
    }
    ctrlMixWLoc(&firing_times, ctrl_control, curr_state,
		0, 20.0f, 1000);
  }

  // Set firing times
  propSetThrusterTimes(&firing_times);
  padsGlobalPeriodSetAndWait(200,405);

  repeat++;
  if ( repeat == 2 ) {
    repeat = 0;
    // step = (step + 1) % 24;
  }
}

// necessary to compile
void gspProcessRXData() {}
