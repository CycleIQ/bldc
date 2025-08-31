#ifndef CYCLEIQ_PID_H
#define CYCLEIQ_PID_H

typedef struct PID
{
  float kp; // Proportional gain
  float ki; // Integral gain
  float kd;

  float integrator;
  float prev_error;

  float integrator_min;
  float integrator_max;

  float output_min;
  float output_max;
} pid_controller_t;

void pid_init(pid_controller_t *pid, float kp, float ki, float kd,
              float integrator_min, float integrator_max,
              float output_min, float output_max);
float pid_update(pid_controller_t *pid, float target, float measured_value, float dt);

#endif