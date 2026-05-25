#include "control.h"

#include "data.h"
#include "mc_interface.h"
#include "pas.h"
#include "utils_math.h"

#include <math.h>

#define CYCLEIQ_PHASE_CURRENT_RAMP_A 1.5f
#define CYCLEIQ_MIN_DUTY_FOR_CURRENT_ESTIMATE 0.02f

static const float gear_currents_normal_a[] = {
    0.0f,
    2.0f,
    3.125f,
    5.0f,
};

static const float gear_currents_mountain_a[] = {
    0.0f,
    2.5f,
    5.0f,
    9.0f,
    15.0f,
    30.0f,
    40.0f,
};

static float phase_current_filtered_a;

static uint8_t clamp_gear_index(uint8_t gear, uint8_t max_index) {
  if (gear > max_index) {
    return max_index;
  }

  return gear;
}

static float current_for_gear(uint8_t gear, cycleiq_ride_mode_t ride_mode) {
  if (ride_mode == CYCLEIQ_RIDE_MODE_MOUNTAIN) {
    uint8_t index = clamp_gear_index(
        gear, (uint8_t)(sizeof(gear_currents_mountain_a) /
                            sizeof(gear_currents_mountain_a[0]) -
                        1u));
    return gear_currents_mountain_a[index];
  }

  uint8_t index = clamp_gear_index(
      gear, (uint8_t)(sizeof(gear_currents_normal_a) /
                          sizeof(gear_currents_normal_a[0]) -
                      1u));
  return gear_currents_normal_a[index];
}

static float target_battery_current_a(void) {
  if (!cycleiq_data.motor_enabled) {
    return 0.0f;
  }

  float gear_current =
      current_for_gear(cycleiq_data.current_gear, cycleiq_data.ride_mode);

  switch (cycleiq_data.support_mode) {
  case CYCLEIQ_MODE_PAS:
    return cycleiq_pas_is_pedaling() ? gear_current : 0.0f;

  case CYCLEIQ_MODE_TORQUE:
    return gear_current * cycleiq_ts_get_percentage();

  case CYCLEIQ_MODE_HYBRID:
  default:
    return 0.0f;
  }
}

void cycleiq_control_init(void) {
  phase_current_filtered_a = 0.0f;
  mc_interface_set_current(0.0f);
}

void cycleiq_control_stop(void) {
  phase_current_filtered_a = 0.0f;
  mc_interface_set_current(0.0f);
}

void cycleiq_control_loop(void) {
  float target_current_a = target_battery_current_a();
  float target_phase_current_a = 0.0f;

  if (target_current_a > 0.0f) {
    float duty = fabsf(mc_interface_get_duty_cycle_now());
    if (duty < CYCLEIQ_MIN_DUTY_FOR_CURRENT_ESTIMATE) {
      duty = CYCLEIQ_MIN_DUTY_FOR_CURRENT_ESTIMATE;
    }

    target_phase_current_a = target_current_a / duty;
  }

  const volatile mc_configuration *mcconf = mc_interface_get_configuration();
  float max_current_a = mcconf->l_current_max;
  if (max_current_a < 0.0f) {
    max_current_a = 0.0f;
  }
  utils_truncate_number(&target_phase_current_a, 0.0f, max_current_a);

  float new_phase_current_a = phase_current_filtered_a;
  UTILS_LP_MOVING_AVG_APPROX(new_phase_current_a, target_phase_current_a, 10);

  float delta_a = new_phase_current_a - phase_current_filtered_a;
  utils_truncate_number(&delta_a, -CYCLEIQ_PHASE_CURRENT_RAMP_A,
                        CYCLEIQ_PHASE_CURRENT_RAMP_A);

  phase_current_filtered_a += delta_a;
  mc_interface_set_current(phase_current_filtered_a);
}
