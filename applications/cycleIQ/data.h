#ifndef CYCLEIQ_DATA_H
#define CYCLEIQ_DATA_H

#include <stdbool.h>
#include <stdint.h>

#include "datatypes.h"

typedef struct CYCLEIQ_CONFIG
{
  float max_speed_kph;
  float battery_internal_resistance_ohm;
  float wheel_diameter_m;
} cycleiq_config_t;

typedef struct CYCLEIQ_DATA
{
  // CycleIQ internal data
  cycleiq_screen_t screen;

  // Battery data
  uint8_t battery_level_pct;
  float battery_voltage_v;
  float battery_current_a;
  float watt_hours;
  float amp_hours;

  // Motor data
  int8_t motor_temperature_c;
  int8_t controller_temperature_c;
  float motor_current_a;
  uint16_t motor_power_w;
  float motor_rpm;

  // Ride configuration
  uint8_t current_gear;
  uint8_t max_gear;
  cycleiq_support_mode_t support_mode;
  cycleiq_ride_mode_t ride_mode;
  bool motor_enabled;

  // Ride data
  float speed_mps;
  float trip_distance_km;
  float range_km;
} cycleiq_data_t;

void cycleiq_data_init(void);
void cycleiq_data_reset(void);
void cycleiq_config_load(void);
void cycleiq_config_save(void);

void cycleiq_data_loop(void);
void cycleiq_data_motor_sensor_update(float rpm, int8_t temperature_c);

bool cycleiq_data_set_gear(uint8_t gear);
bool cycleiq_data_set_support_mode(cycleiq_support_mode_t mode);
bool cycleiq_data_set_ride_mode(cycleiq_ride_mode_t mode);
bool cycleiq_data_set_screen(cycleiq_screen_t screen);
void cycleiq_data_set_motor_enabled(bool enabled);
void cycleiq_data_apply_ride_mode_limits(void);

extern cycleiq_config_t cycleiq_config;
extern cycleiq_data_t cycleiq_data;

#endif
