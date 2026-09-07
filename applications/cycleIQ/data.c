#include "data.h"

#include "conf_general.h"
#include "commands.h"
#include "datatypes.h"
#include "mc_interface.h"

#include "ch.h"

#include <math.h>

#include "utils_math.h"

#define MAX_GEAR 3
#ifdef CYCLEIQ_HIGH_POWER
#define MAX_GEAR_MOUNTAIN 6 // Maximum gear for high power mode
#else
#define MAX_GEAR_MOUNTAIN 5 // Maximum gear for normal mode
#endif
#define CYCLEIQ_MAX_TRIP_INTEGRATION_S 1.0f
#define CYCLEIQ_CONFIG_EEPROM_MAGIC_ADDR 120
#define CYCLEIQ_CONFIG_EEPROM_VERSION_ADDR 121
#define CYCLEIQ_CONFIG_MAGIC 0x43495131u
#define CYCLEIQ_CONFIG_VERSION 1u

static float session_watt_hours_used_start;
static float session_watt_hours_charged_start;
static float session_amp_hours_used_start;
static float session_amp_hours_charged_start;
static systime_t session_start_time;
static systime_t trip_last_update_time;
static bool trip_timing_initialized;

cycleiq_config_t cycleiq_config = {
    .max_speed_kph = 25.0f,
    .battery_internal_resistance_ohm = 0.05f,
    .wheel_diameter_m = 0.66f,
};

cycleiq_data_t cycleiq_data = {
    .battery_level_pct = 100,
    .battery_voltage_v = 0.0f,
    .battery_current_a = 0.0f,
    .watt_hours = 0.0f,
    .amp_hours = 0.0f,
    .motor_temperature_c = 0,
    .motor_rpm = 0.0f, // Initialize motor RPM to 0
    .controller_temperature_c = 0,
    .motor_current_a = 0.0f,
    .motor_power_w = 0,
    .current_gear = 3, // Default gear
    .max_gear = MAX_GEAR,
    .support_mode = CYCLEIQ_SUPPORT_MODE_PAS,
    .ride_mode = CYCLEIQ_RIDE_MODE_NORMAL, // Default ride mode
    .motor_enabled = true,
    .speed_mps = 0.0f,
    .trip_distance_km = 0.0f,
    .trip_time_s = 0u,
    .average_speed_mps = 0.0f,
    .range_km = 0.0f,
};

static float finite_or_zero(float value) {
  return isfinite(value) ? value : 0.0f;
}

static float non_negative_delta(float current, float baseline) {
  float delta = finite_or_zero(current) - finite_or_zero(baseline);
  if (delta < 0.0f || !isfinite(delta)) {
    return 0.0f;
  }

  return delta;
}

static bool config_validate_all(const cycleiq_config_t *config) {
  return config != NULL && isfinite(config->max_speed_kph) &&
         config->max_speed_kph >= 5.0f && config->max_speed_kph <= 45.0f &&
         isfinite(config->battery_internal_resistance_ohm) &&
         config->battery_internal_resistance_ohm >= 0.0f &&
         config->battery_internal_resistance_ohm <= 0.5f &&
         isfinite(config->wheel_diameter_m) && config->wheel_diameter_m >= 0.3f &&
         config->wheel_diameter_m <= 1.0f;
}

static void capture_session_energy_baselines(void) {
  session_watt_hours_used_start =
      finite_or_zero(mc_interface_get_watt_hours(false));
  session_watt_hours_charged_start =
      finite_or_zero(mc_interface_get_watt_hours_charged(false));
  session_amp_hours_used_start = finite_or_zero(mc_interface_get_amp_hours(false));
  session_amp_hours_charged_start =
      finite_or_zero(mc_interface_get_amp_hours_charged(false));
}

void cycleiq_data_apply_ride_mode_limits(void) {
  switch (cycleiq_data.ride_mode)
  {
  case CYCLEIQ_RIDE_MODE_MOUNTAIN:
    cycleiq_data.max_gear = MAX_GEAR_MOUNTAIN;
    break;

  default:
    cycleiq_data.max_gear = MAX_GEAR;
    break;
  }

  if (cycleiq_data.current_gear > cycleiq_data.max_gear)
  {
    cycleiq_data.current_gear = cycleiq_data.max_gear;
  }
}

void cycleiq_data_init(void)
{
  cycleiq_data_reset();
  capture_session_energy_baselines();
  session_start_time = chVTGetSystemTimeX();
  trip_last_update_time = session_start_time;
  trip_timing_initialized = false;
  cycleiq_data.current_gear = MAX_GEAR;
  cycleiq_data_apply_ride_mode_limits();
}

void cycleiq_data_reset(void)
{
  cycleiq_data.battery_level_pct = 0;
  cycleiq_data.battery_voltage_v = 0.0f;
  cycleiq_data.battery_current_a = 0.0f;
  cycleiq_data.watt_hours = 0.0f;
  cycleiq_data.amp_hours = 0.0f;
  cycleiq_data.motor_temperature_c = 0;
  cycleiq_data.controller_temperature_c = 0;
  cycleiq_data.motor_current_a = 0.0f;
  cycleiq_data.motor_power_w = 0;
  cycleiq_data.motor_rpm = 0.0f;
  cycleiq_data.current_gear = 0;                     // Reset to default gear
  cycleiq_data.support_mode = CYCLEIQ_SUPPORT_MODE_PAS;
  cycleiq_data.ride_mode = CYCLEIQ_RIDE_MODE_NORMAL; // Reset to default ride mode
  cycleiq_data.motor_enabled = true;
  cycleiq_data.speed_mps = 0.0f;
  cycleiq_data.trip_distance_km = 0.0f;
  cycleiq_data.trip_time_s = 0u;
  cycleiq_data.average_speed_mps = 0.0f;
  cycleiq_data.range_km = 0.0f;
  trip_timing_initialized = false;
}

void cycleiq_config_load(void)
{
  eeprom_var magic;
  eeprom_var version;
  cycleiq_config_t loaded = cycleiq_config;

  if (!conf_general_read_eeprom_var_custom(&magic,
                                           CYCLEIQ_CONFIG_EEPROM_MAGIC_ADDR) ||
      !conf_general_read_eeprom_var_custom(
          &version, CYCLEIQ_CONFIG_EEPROM_VERSION_ADDR) ||
      magic.as_u32 != CYCLEIQ_CONFIG_MAGIC ||
      version.as_u32 != CYCLEIQ_CONFIG_VERSION) {
    return;
  }

  eeprom_var value;
  if (!conf_general_read_eeprom_var_custom(&value, 122)) {
    return;
  }
  loaded.max_speed_kph = (float)value.as_u32 / 100.0f;
  if (!conf_general_read_eeprom_var_custom(&value, 123)) {
    return;
  }
  loaded.battery_internal_resistance_ohm = (float)value.as_u32 / 1000.0f;
  if (!conf_general_read_eeprom_var_custom(&value, 124)) {
    return;
  }
  loaded.wheel_diameter_m = (float)value.as_u32 / 1000.0f;

  if (!config_validate_all(&loaded)) {
    return;
  }

  cycleiq_config = loaded;
}

bool cycleiq_config_save(void)
{
  eeprom_var value;

  if (!config_validate_all(&cycleiq_config)) {
    return false;
  }

  value.as_u32 = (uint32_t)(cycleiq_config.max_speed_kph * 100.0f + 0.5f);
  if (!conf_general_store_eeprom_var_custom(&value, 122)) {
    return false;
  }
  value.as_u32 =
      (uint32_t)(cycleiq_config.battery_internal_resistance_ohm * 1000.0f +
                 0.5f);
  if (!conf_general_store_eeprom_var_custom(&value, 123)) {
    return false;
  }
  value.as_u32 = (uint32_t)(cycleiq_config.wheel_diameter_m * 1000.0f + 0.5f);
  if (!conf_general_store_eeprom_var_custom(&value, 124)) {
    return false;
  }

  value.as_u32 = CYCLEIQ_CONFIG_VERSION;
  if (!conf_general_store_eeprom_var_custom(
          &value, CYCLEIQ_CONFIG_EEPROM_VERSION_ADDR)) {
    return false;
  }

  value.as_u32 = CYCLEIQ_CONFIG_MAGIC;
  if (!conf_general_store_eeprom_var_custom(&value,
                                            CYCLEIQ_CONFIG_EEPROM_MAGIC_ADDR)) {
    return false;
  }

  return true;
}

void cycleiq_data_loop(void)
{
  systime_t now = chVTGetSystemTimeX();

  // Update battery data
  cycleiq_data.battery_voltage_v = mc_interface_get_input_voltage_filtered();
  float battery_level = utils_map(cycleiq_data.battery_voltage_v,
                                  44.0f,
                                  54.6f,
                                  0.0f,
                                  100.0f);
  utils_truncate_number(&battery_level, 0.0f, 100.0f);
  cycleiq_data.battery_level_pct = (uint8_t)battery_level;

  cycleiq_data.battery_current_a =
      mc_interface_get_tot_current_in_filtered();
  cycleiq_data.motor_current_a =
      mc_interface_get_tot_current_directional_filtered();
  float controller_temperature = mc_interface_temp_fet_filtered();
  utils_truncate_number(&controller_temperature, -128.0f, 127.0f);
  cycleiq_data.controller_temperature_c = (int8_t)controller_temperature;

  float motor_power = cycleiq_data.battery_current_a *
                      cycleiq_data.battery_voltage_v;
  utils_truncate_number(&motor_power, -32768.0f, 32767.0f);
  cycleiq_data.motor_power_w = (int16_t)motor_power;

  float watt_hours_used = non_negative_delta(mc_interface_get_watt_hours(false),
                                             session_watt_hours_used_start);
  float watt_hours_charged = non_negative_delta(
      mc_interface_get_watt_hours_charged(false),
      session_watt_hours_charged_start);
  cycleiq_data.watt_hours = watt_hours_used - watt_hours_charged;
  if (cycleiq_data.watt_hours < 0.0f || !isfinite(cycleiq_data.watt_hours)) {
    cycleiq_data.watt_hours = 0.0f;
  }

  float amp_hours_used = non_negative_delta(mc_interface_get_amp_hours(false),
                                            session_amp_hours_used_start);
  float amp_hours_charged = non_negative_delta(
      mc_interface_get_amp_hours_charged(false),
      session_amp_hours_charged_start);
  cycleiq_data.amp_hours = amp_hours_used - amp_hours_charged;
  if (cycleiq_data.amp_hours < 0.0f || !isfinite(cycleiq_data.amp_hours)) {
    cycleiq_data.amp_hours = 0.0f;
  }

  systime_t session_elapsed_ticks = now - session_start_time;
  cycleiq_data.trip_time_s =
      (uint32_t)(session_elapsed_ticks / CH_CFG_ST_FREQUENCY);

  if (!trip_timing_initialized) {
    trip_last_update_time = now;
    trip_timing_initialized = true;
  } else {
    systime_t elapsed_ticks = now - trip_last_update_time;
    trip_last_update_time = now;

    float dt_s = (float)elapsed_ticks / (float)CH_CFG_ST_FREQUENCY;
    utils_truncate_number(&dt_s, 0.0f, CYCLEIQ_MAX_TRIP_INTEGRATION_S);

    float speed_mps = cycleiq_data.speed_mps;
    if (isfinite(speed_mps) && speed_mps > 0.0f) {
      cycleiq_data.trip_distance_km += (speed_mps * dt_s) / 1000.0f;
    }
  }

  if (cycleiq_data.trip_time_s > 0u) {
    cycleiq_data.average_speed_mps =
        (cycleiq_data.trip_distance_km * 1000.0f) /
        (float)cycleiq_data.trip_time_s;
  } else {
    cycleiq_data.average_speed_mps = 0.0f;
  }
}

void cycleiq_data_motor_sensor_update(float rpm, int8_t temperature_c)
{
  if (!isfinite(rpm) || rpm < 0.0f) {
    rpm = 0.0f;
  }

  cycleiq_data.motor_rpm = rpm;
  cycleiq_data.motor_temperature_c = temperature_c;

  // Update the speed based on RPM
  cycleiq_data.speed_mps = (rpm * cycleiq_config.wheel_diameter_m * M_PI) / 60.0f;
  if (!isfinite(cycleiq_data.speed_mps) || cycleiq_data.speed_mps < 0.0f) {
    cycleiq_data.speed_mps = 0.0f;
  }
}

bool cycleiq_data_set_gear(uint8_t gear) {
  if (gear > cycleiq_data.max_gear) {
    return false;
  }

  cycleiq_data.current_gear = gear;
  return true;
}

bool cycleiq_data_set_support_mode(cycleiq_support_mode_t mode) {
  if (mode != CYCLEIQ_SUPPORT_MODE_PAS &&
      mode != CYCLEIQ_SUPPORT_MODE_TORQUE) {
    return false;
  }

  cycleiq_data.support_mode = mode;
  return true;
}

bool cycleiq_data_set_ride_mode(cycleiq_ride_mode_t mode) {
  if (mode > CYCLEIQ_RIDE_MODE_MOUNTAIN) {
    return false;
  }

  cycleiq_data.ride_mode = mode;
  cycleiq_data_apply_ride_mode_limits();
  return true;
}

void cycleiq_data_set_motor_enabled(bool enabled) {
  cycleiq_data.motor_enabled = enabled;
}
