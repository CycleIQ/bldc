#include "data.h"

#include "conf_general.h"
#include "commands.h"
#include "datatypes.h"
#include "mc_interface.h"

#include <math.h>

#include "utils_math.h"

#define MAX_GEAR 3
#ifdef CYCLEIQ_HIGH_POWER
#define MAX_GEAR_MOUNTAIN 6 // Maximum gear for high power mode
#else
#define MAX_GEAR_MOUNTAIN 5 // Maximum gear for normal mode
#endif

cycleiq_config_t cycleiq_config = {
    .max_speed_kph = 25.0f,
    .battery_internal_resistance_ohm = 0.05f,
    .wheel_diameter_m = 0.66f,
};

cycleiq_data_t cycleiq_data = {
    .screen = CYCLEIQ_SCREEN_MAIN,
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
    .support_mode = CYCLEIQ_MODE_PAS,      // Default support mode
    .ride_mode = CYCLEIQ_RIDE_MODE_NORMAL, // Default ride mode
    .motor_enabled = true,
    .speed_mps = 0.0f,
    .trip_distance_km = 0.0f,
    .range_km = 0.0f,
};

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
  cycleiq_data.current_gear = MAX_GEAR;
  cycleiq_data_apply_ride_mode_limits();
}

void cycleiq_data_reset(void)
{
  cycleiq_data.screen = CYCLEIQ_SCREEN_MAIN; // Reset to default screen
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
  cycleiq_data.support_mode = CYCLEIQ_MODE_PAS;      // Reset to default support mode
  cycleiq_data.ride_mode = CYCLEIQ_RIDE_MODE_NORMAL; // Reset to default ride mode
  cycleiq_data.motor_enabled = true;
  cycleiq_data.speed_mps = 0.0f;
  cycleiq_data.trip_distance_km = 0.0f;
  cycleiq_data.range_km = 0.0f;
}

void cycleiq_config_load(void)
{
}

void cycleiq_config_save(void)
{
}

void cycleiq_data_loop(void)
{
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
  utils_truncate_number(&motor_power, 0.0f, 65535.0f);
  cycleiq_data.motor_power_w = (uint16_t)motor_power;
}

void cycleiq_data_motor_sensor_update(float rpm, int8_t temperature_c)
{
  cycleiq_data.motor_rpm = rpm;
  cycleiq_data.motor_temperature_c = temperature_c;

  // Update the speed based on RPM
  cycleiq_data.speed_mps = (rpm * cycleiq_config.wheel_diameter_m * M_PI) / 60.0f;
}

bool cycleiq_data_set_gear(uint8_t gear) {
  if (gear < 1 || gear > cycleiq_data.max_gear) {
    return false;
  }

  cycleiq_data.current_gear = gear;
  return true;
}

bool cycleiq_data_set_support_mode(cycleiq_support_mode_t mode) {
  if (mode > CYCLEIQ_MODE_HYBRID) {
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

bool cycleiq_data_set_screen(cycleiq_screen_t screen) {
  if (screen > CYCLEIQ_SCREEN_GRAPH) {
    return false;
  }

  cycleiq_data.screen = screen;
  return true;
}

void cycleiq_data_set_motor_enabled(bool enabled) {
  cycleiq_data.motor_enabled = enabled;
}
