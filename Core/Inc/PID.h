/*
 * PID.h
 *
 */

#ifndef INC_PID_H_
#define INC_PID_H_

typedef struct {

	/* Controller gains */
	float Kp;
	float Ki;
	float Kd;

	/* Derivative low-pass filter time constant */
	float tau;

	/* Output limits */
	float limMin;
	float limMax;

	/* Integrator limits */
	float limMinInt;
	float limMaxInt;

	/* Sample time (in seconds) */
	float T;

	/* Controller "memory" */
	float integrator;
	float prevError;			/* Required for integrator */
	float differentiator;
	float prevMeasurement;		/* Required for differentiator */

	/* Controller output */
	float out;

	/* Error measurement */
	float error;

} PIDController;

void PIDController_Init(PIDController *pid, float Kp, float Ki, float Kd, float tau,
						float limMin, float limMax, float limMinInt, float limMaxInt,
						float Ts);

float PIDController_Update(PIDController *pid, float setpoint, float measurement);

#endif /* INC_PID_H_ */
