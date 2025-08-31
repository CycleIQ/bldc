#ifndef CYCLEIQ_DATA_H
#define CYCLEIQ_DATA_H

#include "app.h"
#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"

#include "datatypes.h"

typedef struct CYCLEIQ_CONFIG
{
  float max_speed;
  float battery_internal_resistance;

  float wheel_diameter; // Diameter in meters
} cycleiq_config_t;

typedef struct CYCLEIQ_DATA
{
  // CycleIQ internal data
  cycleiq_screen_t screen;

  // Battery data
  uint8_t battery_level;
  float battery_voltage;
  float battery_current; // No regen so this is always positive
  float watt_hours;
  float amp_hours;

  // Motor data
  int8_t motor_temperature;
  int8_t controller_temperature;
  float motor_current;
  uint16_t motor_power;
  float motor_rpm;

  // Ride configuration
  uint8_t current_gear;
  cycleiq_support_mode_t support_mode;
  cycleiq_ride_mode_t ride_mode;

  // Ride data
  float speed;
  float trip_distance; // Distance in km
  float range;         // Range in km
} cycleiq_data_t;

void cycleiq_data_reset(void);
void cycleiq_config_load(void);
void cycleiq_config_save(void);

void cycleiq_data_loop(void);
void cycleiq_data_motor_update(float rpm, int8_t temperature);

extern cycleiq_config_t cycleiq_config;
extern cycleiq_data_t cycleiq_data;

#endif