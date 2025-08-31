#include "pid.h"

void pid_init(pid_controller_t *pid, float kp, float ki, float kd,
              float integrator_min, float integrator_max,
              float output_min, float output_max)
{
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->integrator = 0.0f;
  pid->prev_error = 0.0f;
  pid->integrator_min = integrator_min;
  pid->integrator_max = integrator_max;
  pid->output_min = output_min;
  pid->output_max = output_max;
}

float pid_update(pid_controller_t *pid, float target, float measured_value, float dt)
{
  float error = target - measured_value;
  pid->integrator += error * dt;

  // Clamp the integrator to prevent windup
  if (pid->integrator < pid->integrator_min)
    pid->integrator = pid->integrator_min;
  else if (pid->integrator > pid->integrator_max)
    pid->integrator = pid->integrator_max;

  float derivative = (error - pid->prev_error) / dt;
  pid->prev_error = error;

  // PID output calculation
  float output = (pid->kp * error) + (pid->ki * pid->integrator) + (pid->kd * derivative);

  // Clamp the output to the specified limits
  if (output < pid->output_min)
    output = pid->output_min;
  else if (output > pid->output_max)
    output = pid->output_max;

  return output;
}