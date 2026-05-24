#include "data.h"
#include "mc_interface.h"
#include "datatypes.h"
#include "conf_general.h"
#include <math.h>
#include "commands.h"

#include "utils_math.h"

#define MAX_GEAR 3
#ifdef CYCLEIQ_HIGH_POWER
#define MAX_GEAR_MOUNTAIN 6 // Maximum gear for high power mode
#else
#define MAX_GEAR_MOUNTAIN 5 // Maximum gear for normal mode
#endif

cycleiq_config_t cycleiq_config = {
    .max_speed = 25.0f,                   // Default max speed in km/h
    .battery_internal_resistance = 0.05f, // Default internal resistance in ohms
    .wheel_diameter = 0.66f,              // (26 inches) Default wheel diameter in meters
};

cycleiq_data_t cycleiq_data = {
    .screen = CYCLEIQ_SCREEN_MAIN,
    .battery_level = 100,
    .battery_voltage = 0.0f,
    .battery_current = 0.0f,
    .watt_hours = 0.0f,
    .amp_hours = 0.0f,
    .motor_temperature = 0,
    .motor_rpm = 0.0f, // Initialize motor RPM to 0
    .controller_temperature = 0,
    .motor_current = 0.0f,
    .motor_power = 0,
    .current_gear = 3, // Default gear
    .max_gear = MAX_GEAR,
    .support_mode = CYCLEIQ_MODE_PAS,      // Default support mode
    .ride_mode = CYCLEIQ_RIDE_MODE_NORMAL, // Default ride mode
    .speed = 0.0f,
    .trip_distance = 0.0f, // Initialize trip distance to 0
    .range = 0.0f,         // Initialize range to 0
};

void cycleiq_change_ride_mode(void)
{
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

  vesc_set_erpm_limit();
}

void vesc_set_erpm_limit(void)
{
  // const volatile mc_configuration *mcconf = mc_interface_get_configuration();
  // float max_speed = cycleiq_data.ride_mode == CYCLEIQ_RIDE_MODE_NORMAL ? 25.0f : cycleiq_config.max_speed;
  //
  // float pole_pairs = mcconf->si_motor_poles / 2.0f;
  // float gear_ratio = mcconf->si_gear_ratio > 0.0f ? mcconf->si_gear_ratio : 1.0f;
  // float wheel_circumference = mcconf->si_wheel_diameter * M_PI; // Wheel circumference in meters
  //
  // float max_speed_rpm = (max_speed * 1000.0f) / 60.0f * (1.0f / wheel_circumference) * gear_ratio; // Max speed in RPM
  // mcconf->l_max_erpm = max_speed_rpm * pole_pairs;
  // mcconf->l_min_erpm = -mcconf->l_max_erpm;
}

void cycleiq_data_init(void)
{
  cycleiq_data_reset();

  cycleiq_change_ride_mode();
}

void cycleiq_data_reset(void)
{
  cycleiq_data.screen = CYCLEIQ_SCREEN_MAIN; // Reset to default screen
  cycleiq_data.battery_level = 0;
  cycleiq_data.battery_voltage = 0.0f;
  cycleiq_data.battery_current = 0.0f;
  cycleiq_data.watt_hours = 0.0f;
  cycleiq_data.amp_hours = 0.0f;
  cycleiq_data.motor_temperature = 0;
  cycleiq_data.controller_temperature = 0;
  cycleiq_data.motor_current = 0.0f;
  cycleiq_data.motor_power = 0;
  cycleiq_data.motor_rpm = 0.0f;
  cycleiq_data.current_gear = 0;                     // Reset to default gear
  cycleiq_data.support_mode = CYCLEIQ_MODE_PAS;      // Reset to default support mode
  cycleiq_data.ride_mode = CYCLEIQ_RIDE_MODE_NORMAL; // Reset to default ride mode
  cycleiq_data.speed = 0.0f;
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
  cycleiq_data.battery_voltage = mc_interface_get_input_voltage_filtered();
  cycleiq_data.battery_level = (uint8_t)utils_map(cycleiq_data.battery_voltage,
                                                  44.0f,
                                                  54.6f,
                                                  0.0f,
                                                  100.0f);
  if (cycleiq_data.battery_level > 100)
    cycleiq_data.battery_level = 100;
  if (cycleiq_data.battery_level < 0)
    cycleiq_data.battery_level = 0;

  cycleiq_data.motor_power = (uint16_t)(mc_interface_get_tot_current_filtered() *
                                        mc_interface_get_input_voltage_filtered());
}

void cycleiq_data_motor_update(float rpm, int8_t temperature)
{
  cycleiq_data.motor_rpm = rpm;
  cycleiq_data.motor_temperature = temperature;

  // Update the speed based on RPM
  cycleiq_data.speed = (rpm * cycleiq_config.wheel_diameter * M_PI) / 60.0f; // Speed in m/s
}
