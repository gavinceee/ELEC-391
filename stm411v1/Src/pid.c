#include "PID.h"

/* --- PID --- */

void PIDController_Init(PIDController *pid)
{
    pid->integrator      = 0.0f;
    pid->prevError       = 0.0f;
    pid->differentiator  = 0.0f;
    pid->prevMeasurement = 0.0f;
    pid->out             = 0.0f;
}

float PIDController_Update(PIDController *pid, float setpoint, float measurement)
{
    /* 1) Error — linear, no wrapping */
    float error = setpoint - measurement;

    /* 2) Proportional */
    float proportional = pid->Kp * error;

    /* 3) Integral (trapezoidal) */
    pid->integrator += 0.5f * pid->Ki * pid->T * (error + pid->prevError);

    if (pid->integrator > pid->limMaxInt) pid->integrator = pid->limMaxInt;
    else if (pid->integrator < pid->limMinInt) pid->integrator = pid->limMinInt;

    /* 4) Derivative on measurement (no wrapping) */
    float dMeas = measurement - pid->prevMeasurement;

    pid->differentiator =
        -(2.0f * pid->Kd * dMeas
          + (2.0f * pid->tau - pid->T) * pid->differentiator)
        / (2.0f * pid->tau + pid->T);

    /* 5) Output + limits */
    pid->out = proportional + pid->integrator + pid->differentiator;

    if (pid->out > pid->limMax) pid->out = pid->limMax;
    else if (pid->out < pid->limMin) pid->out = pid->limMin;

    /* 6) Store state */
    pid->prevError       = error;
    pid->prevMeasurement = measurement;

    return pid->out;
}
