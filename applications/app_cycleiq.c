#pragma GCC optimize("Os")

#include "app.h"
#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "mc_interface.h"
#include "timeout.h"
#include "utils_math.h"
#include "comm_can.h"
#include "hw.h"
#include <math.h>
#include "commands.h"

#include "cycleIQ/datatypes.h"
#include "cycleIQ/pas.h"
#include "cycleIQ/comm.h"
#include "cycleIQ/data.h"
#include "cycleIQ/pid.h"

static THD_FUNCTION(cycleiq_thread, arg);
static THD_WORKING_AREA(cycleiq_thread_wa, 1024);

static THD_FUNCTION(cycleiq_speed_sensor_thread, arg);
static THD_WORKING_AREA(cycleiq_speed_sensor_thread_wa, 256);

static const float PHASE_CURRENT_RAMP = 1.5f;

static volatile bool stop_now = true;
static volatile bool is_running = false;

static volatile float motor_rpm;          // Speed from the speed sensor in rpm (used for filtering)
static volatile int8_t motor_temperature; // Motor temperature in Celsius

static volatile bool cycleiq_motor_on = true; // True if the motor is on

static const float gear_currents[] = {
    0.0f,  // 0 gear
    2.5f,  // 1 gear
    5.0f,  // 2 gear
    9.0f,  // 3 gear
    15.0f, // 4 gear
    30.0f, // 5 gear
    50.0f, // 6 gear (experimental)
};

void app_custom_start(void)
{
  stop_now = false;

  cycleiq_comm_init();

  cycleiq_pas_init();
  cycleiq_pas_configure(&(cycleiq_pas_config){
      .pedal_rpm_start = 10.0, // RPM at which PAS starts
      .magnets = 18,           // Number of magnets on the PAS sensor
  });

  chThdCreateStatic(cycleiq_thread_wa, sizeof(cycleiq_thread_wa), NORMALPRIO, cycleiq_thread, NULL);
  chThdCreateStatic(cycleiq_speed_sensor_thread_wa, sizeof(cycleiq_speed_sensor_thread_wa), NORMALPRIO, cycleiq_speed_sensor_thread, NULL);
};
void app_custom_stop(void)
{
  stop_now = true;
  while (is_running)
  {
    chThdSleepMilliseconds(1);
  }

  cycleiq_pas_deinit();
  cycleiq_comm_deinit();
  cycleiq_pas_configure(&(cycleiq_pas_config){0}); // Reset configuration
};

void app_custom_configure(app_configuration *conf) {};

static THD_FUNCTION(cycleiq_thread, arg)
{
  (void)arg;
  chRegSetThreadName("CYCLEIQ");
  is_running = true;
  systime_t time = chVTGetSystemTimeX();
  float phase_current = 0.0f;          // Current in amps
  float phase_current_filtered = 0.0f; // Filtered current

  float current = 0.0f;        // Battery current in amps
  float target_current = 0.0f; // Target current in amps (controlled by PID)

  float duty = 0.0f;

  for (;;)
  {
    if (stop_now)
    {
      is_running = false;
      return;
    }
    chThdSleepMilliseconds(10); // 100Hz loop

    commands_printf("TS: %.2fV", cycleiq_ts_get_voltage());

    cycleiq_pas_loop();
    cycleiq_comm_loop();
    cycleiq_data_loop();

    timeout_reset();

    if (!cycleiq_motor_on)
    {
      mc_interface_set_current(0.0f); // Ensure motor is off
      continue;                       // Skip the rest of the loop if motor is off
    }

    switch (cycleiq_data.support_mode)
    {
    case CYCLEIQ_MODE_PAS:
      if (cycleiq_pas_is_pedaling())
        target_current = gear_currents[cycleiq_data.current_gear];
      else
        target_current = 0.0f;
      break;
    case CYCLEIQ_MODE_TORQUE:
      target_current = cycleiq_ts_get_percentage() * gear_currents[cycleiq_data.current_gear];
      break;
    default:
      target_current = 0.0f;
      break;
    }

    if (target_current <= 0.0f)
    {
      phase_current = 0.0f; // If not pedaling, set phase current to zero
      phase_current_filtered = 0.0f;
    }

    duty = mc_interface_get_duty_cycle_now(); // Get the current duty cycle
    if (duty < 0.02f)
    {
      duty = 0.02f;
    }

    phase_current = target_current / duty;                                                                                                   // Calculate phase current based on target current and duty cycle
    utils_truncate_number(&phase_current, mc_interface_get_configuration()->l_current_min, mc_interface_get_configuration()->l_current_max); // Constrain phase current to max limits

    float new_phase_current = phase_current_filtered;
    UTILS_LP_MOVING_AVG_APPROX(new_phase_current, phase_current, 10); // Apply a low-pass filter to the current

    float delta = new_phase_current - phase_current_filtered;
    if (delta > PHASE_CURRENT_RAMP)
    {
      delta = PHASE_CURRENT_RAMP;
    }

    phase_current_filtered += delta; // Ramp the phase current
    mc_interface_set_current(phase_current_filtered);
  }
}

static THD_FUNCTION(cycleiq_speed_sensor_thread, arg)
{
  (void)arg;
  chRegSetThreadName("CYCLEIQ_SPEED_SENSOR");

  systime_t time = chVTGetSystemTimeX();
  systime_t last_pulse_time = 0;
  systime_t last_temperature_time = 0;
  bool last_state = false;
  uint8_t cnt = 0;

  for (;;)
  {
    if (stop_now)
      return;

    chThdSleepUntilWindowed(time, time + US2ST(200));
    time += US2ST(200);

    bool state = (ADC_Value[ADC_IND_TEMP_MOTOR] < 50); // raw threshold
    if (state && !last_state)
    {
      systime_t pulse_duration = chVTTimeElapsedSinceX(last_pulse_time);
      last_pulse_time = time;

      float pulse_duration_s = (float)ST2US(pulse_duration) * 1e-6f;
      if (pulse_duration_s > 0.0f)
      {
        float current_rpm = 60.0f / pulse_duration_s;
        UTILS_LP_MOVING_AVG_APPROX(motor_rpm, current_rpm, 5);
      }
    }

    last_state = state;

    if (!state) // High voltage
    {
      if (time - last_pulse_time > MS2ST(3) && time - last_temperature_time > MS2ST(3))
      {
        last_temperature_time = time;

        int16_t temperature = (int16_t)NTC_TEMP_MOTOR(3435.0f); // Read motor temperature
        utils_truncate_number_int(&temperature, -128, 127);
        int8_t current_temperature = (int8_t)temperature;                       // Constrain to int8_t range
        UTILS_LP_MOVING_AVG_APPROX(motor_temperature, current_temperature, 10); // Apply a moving average filter
      }
    }

    if (++cnt >= 50)
    {
      cnt = 0;
      cycleiq_data_motor_update(motor_rpm, (int16_t)motor_temperature); // Update motor data
    }

    timeout_reset();
  }
}