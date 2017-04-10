#include "sconfig.h"

// CONTROL SYSTEM PARAMETERS
// IDLE partition, not executing code.
#define IDLE 0

float initial_condition_state[N-1][1]           = {{0.0f}, {0.0f}, {0.0f}, {0.0f}}; // system states initial conditions
float initial_condition_control_input[1][1]     = {{0.0f}}; // control input initial conditions

float r[1][1] = {{1.0f}}; 	// Control objective: reference [radians]
float K[1][N] = {{4.5806870f,-4.6545506f, -0.0020604f, -0.0005108f, 0.4139770f}};
float F[1][1] = {{0.0738633f}};

// CompSOC PLATFORM PARAMETERS AND DEBUGGING INFORMATION
// PARAMETERS
int C = 16000;		// CoMik slot duration: clock cycles
int P = 8000;		// Partition slot duration: clock cycles
int TDM_TABLE_SIZE = 4;	// Slots within a TDM frame
int DELAY = 7000;	// Delay added icluded in the actuation task: SENSOR-TO-ACTUATOR delay
int tdm_tasks_schedule[] = {1,2,3,IDLE}; // Size of the schedule is TDM_TABLE_SIZE. TDM table size and tasks mapping: 1 -> sensing, 2 -> computing, 3 -> actuating, IDLE -> null

// DEBUGGING INFO
int DEBUG_INFO_FROM_S =  0; // Measure data from sensor: states
int DEBUG_INFO_FROM_C =  0; // Measure data from computation: states and control input
int DEBUG_INFO_FROM_A =  1; // Measure data from actuator: control input

int TIMING_INFO_FROM_S =  0; // Measure sensing task execution time
int TIMING_INFO_FROM_C =  0; // Measure computation task execution time
int TIMING_INFO_FROM_A =  0; // Measure actuation tasks execution time. WARNING: make sure where to locate the time stamps at the actutor, it might measure after the delay

//float K[1][N] = {{3.9010646f, -3.9132624f, 0.0037453f, -0.0043307f, 0.6115122f}};
//float F[1][1] = {{0.0121976f}};
//float K[1][N] = {{ -3.0f, -0.5f, -0.005f, -0.003f, -0.5f  }}; // Feedback control gain
//float F[1][1] = {{ 0.5f }}; // Feedforward control gain
