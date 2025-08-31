#ifndef CYCLEIQ_DEFINES_H
#define CYCLEIQ_DEFINES_H

#include "mc_interface.h"

typedef enum CYCLEIQ_COMMAND
{
  CYCLEIQ_POWER_OFF = 0x00,
  CYCLEIQ_POWER_ON = 0x01,

  // Ride commands
  CYCLEIQ_COMM_GEAR_SET = 0x02,
  CYCLEIQ_COMM_MODE_SET = 0x03,
  CYCLEIQ_COMM_RIDE_MODE_SET = 0x04,

  // Data commands
  CYCLEIQ_COMM_SCREEN_SET = 0x05,

  // Configuration commands
  CYCLEIQ_COMM_CONFIG_GET = 0x06,
  CYCLEIQ_COMM_CONFIG_SET = 0x07,
} cycleiq_command_t;

typedef enum CYCLEIQ_MODE
{
  CYCLEIQ_MODE_PAS = 0,
  CYCLEIQ_MODE_TORQUE,
  CYCLEIQ_MODE_HYBRID,
} cycleiq_support_mode_t;

typedef enum CYCLEIQ_RIDE_MODE
{
  CYCLEIQ_RIDE_MODE_NORMAL = 0,
  CYCLEIQ_RIDE_MODE_MOUNTAIN,
} cycleiq_ride_mode_t;

typedef enum CYCLEIQ_SCREEN
{
  CYCLEIQ_SCREEN_MAIN = 0x00, // [battery percentage, speed << 8, speed & 0xFF, motor temperature (int16_t), gear << 2 | support_mode, trip << 8, trip & 0xFF, range << 8, range & 0xFF]
  CYCLEIQ_SCREEN_TRIP = 0x01,
  CYCLEIQ_SCREEN_SYSTEM_INFO = 0x02,
  CYCLEIQ_SCREEN_GRAPH = 0x03,
} cycleiq_screen_t;

#endif