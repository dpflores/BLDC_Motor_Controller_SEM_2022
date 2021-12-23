/*
 * PID.h
 *
 *  Created on: Nov 21, 2021
 *  Author: del
 *  Guia: https://github.com/pms67/PID
 */

#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_


typedef struct {

	//Controller parameters
	float Kp;
	float Ki;
	float Kd;

	//Derivative low-pass filter time constant
	float tau;

	//Output limits

	float limMin;
	float limMax;

	// Sample time (in seconds)

	float T;

	// Controller "memory"

	float integrator;
	float prevError;		//For integral
	float differentiator;
	float prevMeasurement;	//For differential

	//Output

	float out;

} PIDController;

void  PIDController_Init(PIDController *pid);
float PIDController_Update(PIDController *pid, float setpoint, float measurement);
void  PIDController_Reset(PIDController *pid);

#endif /* PID_CONTROLLER_H_ */
