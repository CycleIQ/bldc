#include "control.h"

#include "data.h"
#include "mc_interface.h"
#include "pas.h"
#include "utils_math.h"

#include <math.h>

#define CYCLEIQ_BATTERY_LIMIT_START_DUTY 0.33f
#define CYCLEIQ_MIN_DUTY_FOR_CURRENT_ESTIMATE 0.02f
#define CYCLEIQ_SERVICE_PERIOD_S 0.1f
#define CYCLEIQ_SPEED_TAPER_WINDOW_KPH 2.0f
#define CYCLEIQ_PHASE_RAMP_UP_PAS_A_PER_S 40.0f
#define CYCLEIQ_PHASE_RAMP_UP_TORQUE_A_PER_S 80.0f
#define CYCLEIQ_PHASE_RAMP_DOWN_A_PER_S 40.0f
#define CYCLEIQ_PHASE_RELEASE_A_PER_S 80.0f
#define CYCLEIQ_BATTERY_OVERCURRENT_PHASE_GAIN 2.0f

typedef struct {
  float battery_current_limit_a;
  float phase_current_limit_a;
} cycleiq_gear_limits_t;

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

static float phase_current_output_a;

static uint8_t clamp_gear_index(uint8_t gear, uint8_t max_index) {
  if (gear > max_index) {
    return max_index;
  }

  return gear;
}

static float battery_current_for_gear(uint8_t gear,
                                      cycleiq_ride_mode_t ride_mode) {
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

static cycleiq_gear_limits_t gear_limits_for_current(float battery_current_a) {
  cycleiq_gear_limits_t limits = {
      .battery_current_limit_a = battery_current_a,
      .phase_current_limit_a =
          battery_current_a / CYCLEIQ_BATTERY_LIMIT_START_DUTY,
  };

  return limits;
}

static cycleiq_gear_limits_t gear_limits_for_gear(uint8_t gear,
                                                  cycleiq_ride_mode_t ride_mode) {
  return gear_limits_for_current(battery_current_for_gear(gear, ride_mode));
}

static float speed_taper_factor(void) {
  if (cycleiq_data.ride_mode == CYCLEIQ_RIDE_MODE_MOUNTAIN) {
    return 1.0f;
  }

  float max_speed_kph = cycleiq_config.max_speed_kph;
  if (max_speed_kph <= 0.0f) {
    return 0.0f;
  }

  float speed_kph = cycleiq_data.speed_mps * 3.6f;
  if (speed_kph >= max_speed_kph) {
    return 0.0f;
  }

  float taper_start_kph = max_speed_kph - CYCLEIQ_SPEED_TAPER_WINDOW_KPH;
  if (speed_kph <= taper_start_kph) {
    return 1.0f;
  }

  float factor = (max_speed_kph - speed_kph) / CYCLEIQ_SPEED_TAPER_WINDOW_KPH;
  utils_truncate_number(&factor, 0.0f, 1.0f);
  return factor;
}

static float support_factor(void) {
  if (!cycleiq_data.motor_enabled) {
    return 0.0f;
  }

  switch (cycleiq_data.support_mode) {
  case CYCLEIQ_MODE_PAS:
    return cycleiq_pas_is_pedaling() ? 1.0f : 0.0f;

  case CYCLEIQ_MODE_TORQUE:
    if (!cycleiq_ts_is_active()) {
      return 0.0f;
    }
    return cycleiq_ts_get_percentage();

  case CYCLEIQ_MODE_HYBRID:
  default:
    return 0.0f;
  }
}

static float positive_phase_current_limit(void) {
  const volatile mc_configuration *mcconf = mc_interface_get_configuration();
  float max_current_a = mcconf->lo_current_max;

  if (max_current_a <= 0.0f) {
    max_current_a = mcconf->l_current_max;
  }
  if (max_current_a < 0.0f) {
    max_current_a = 0.0f;
  }

  return max_current_a;
}

static float ramp_up_rate_for_mode(void) {
  if (cycleiq_data.support_mode == CYCLEIQ_MODE_TORQUE) {
    return CYCLEIQ_PHASE_RAMP_UP_TORQUE_A_PER_S;
  }

  return CYCLEIQ_PHASE_RAMP_UP_PAS_A_PER_S;
}

static float ramped_phase_current(float target_phase_current_a,
                                  bool release_fast) {
  float delta_a = target_phase_current_a - phase_current_output_a;
  float ramp_limit_a = ramp_up_rate_for_mode() * CYCLEIQ_SERVICE_PERIOD_S;

  if (delta_a < 0.0f) {
    float ramp_down_a = release_fast ? CYCLEIQ_PHASE_RELEASE_A_PER_S
                                     : CYCLEIQ_PHASE_RAMP_DOWN_A_PER_S;
    ramp_limit_a = ramp_down_a * CYCLEIQ_SERVICE_PERIOD_S;
  }

  utils_truncate_number(&delta_a, -ramp_limit_a, ramp_limit_a);
  return phase_current_output_a + delta_a;
}

void cycleiq_control_init(void) {
  phase_current_output_a = 0.0f;
  mc_interface_set_current(0.0f);
}

void cycleiq_control_stop(void) {
  phase_current_output_a = 0.0f;
  mc_interface_set_current(0.0f);
}

void cycleiq_control_loop(void) {
  cycleiq_gear_limits_t gear_limits =
      gear_limits_for_gear(cycleiq_data.current_gear, cycleiq_data.ride_mode);
  float demand_factor = support_factor() * speed_taper_factor();
  utils_truncate_number(&demand_factor, 0.0f, 1.0f);

  float target_battery_current_a =
      gear_limits.battery_current_limit_a * demand_factor;
  float target_phase_current_a = gear_limits.phase_current_limit_a * demand_factor;
  bool release_fast = target_battery_current_a <= 0.0f;

  if (target_battery_current_a > 0.0f) {
    float duty = fabsf(mc_interface_get_duty_cycle_now());
    if (duty < CYCLEIQ_MIN_DUTY_FOR_CURRENT_ESTIMATE) {
      duty = CYCLEIQ_MIN_DUTY_FOR_CURRENT_ESTIMATE;
    }

    float battery_limited_phase_current_a = target_battery_current_a / duty;
    if (battery_limited_phase_current_a < target_phase_current_a) {
      target_phase_current_a = battery_limited_phase_current_a;
    }

    float measured_battery_current_a = cycleiq_data.battery_current_a;
    if (!isfinite(measured_battery_current_a) ||
        measured_battery_current_a < 0.0f) {
      measured_battery_current_a = 0.0f;
    }

    float battery_overcurrent_a =
        measured_battery_current_a - target_battery_current_a;
    if (battery_overcurrent_a > 0.0f) {
      float corrected_phase_current_a =
          phase_current_output_a -
          battery_overcurrent_a * CYCLEIQ_BATTERY_OVERCURRENT_PHASE_GAIN;
      if (corrected_phase_current_a < target_phase_current_a) {
        target_phase_current_a = corrected_phase_current_a;
      }
    }
  }

  float max_current_a = positive_phase_current_limit();
  utils_truncate_number(&target_phase_current_a, 0.0f, max_current_a);

  phase_current_output_a =
      ramped_phase_current(target_phase_current_a, release_fast);
  utils_truncate_number(&phase_current_output_a, 0.0f, max_current_a);
  mc_interface_set_current(phase_current_output_a);
}
