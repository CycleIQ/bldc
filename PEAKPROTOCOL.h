#ifndef PEAK_PROTOCOL_H
#define PEAK_PROTOCOL_H

#include <stdint.h>

/*
 * PEAK / CycleIQ CAN protocol reference.
 *
 * Transport:
 * - Classic CAN, max payload 8 bytes.
 * - Extended CAN frames.
 * - Multi-byte integers are big-endian.
 * - CAN ID layout:
 *
 *     can_id = ((uint32_t)CYCLEIQ_CAN_ID << 8) | packet_type_or_command;
 *
 * The display firmware expects the ESC to use the same ID layout for telemetry
 * and to decode display commands from the low byte of the CAN ID.
 */

#ifndef CYCLEIQ_CAN_ID
#define CYCLEIQ_CAN_ID 0x45
#endif

#define CYCLEIQ_CAN_FRAME_ID(type_or_command)                                  \
  (((uint32_t)CYCLEIQ_CAN_ID << 8) | (uint8_t)(type_or_command))

#define CYCLEIQ_CAN_NODE_ID(can_id) (((can_id) >> 8) & 0xFF)
#define CYCLEIQ_CAN_PACKET_TYPE(can_id) ((can_id) & 0xFF)

typedef enum {
  CYCLEIQ_POWER_OFF = 0x00,
  CYCLEIQ_POWER_ON = 0x01,

  CYCLEIQ_COMM_GEAR_SET = 0x02,
  CYCLEIQ_COMM_MODE_SET = 0x03,
  CYCLEIQ_COMM_RIDE_MODE_SET = 0x04,

  CYCLEIQ_COMM_SCREEN_SET = 0x05,
  CYCLEIQ_COMM_CONFIG_GET = 0x06,
  CYCLEIQ_COMM_CONFIG_SET = 0x07,
} cycleiq_command_t;

typedef enum {
  CYCLEIQ_MODE_PAS = 0,
  CYCLEIQ_MODE_TORQUE = 1,
  CYCLEIQ_MODE_HYBRID = 2,
} cycleiq_support_mode_t;

typedef enum {
  CYCLEIQ_RIDE_MODE_NORMAL = 0,
  CYCLEIQ_RIDE_MODE_MOUNTAIN = 1,
} cycleiq_ride_mode_t;

typedef enum {
  PEAK_PACKET_TYPE_BATTERY_STATUS = 0x10,
  PEAK_PACKET_TYPE_BATTERY_ENERGY = 0x11,
  PEAK_PACKET_TYPE_MOTOR_STATUS = 0x12,
  PEAK_PACKET_TYPE_CONTROLLER_STATE = 0x13,
  PEAK_PACKET_TYPE_LIVE_STATUS = 0x14,
  PEAK_PACKET_TYPE_TRIP_PRIMARY = 0x15,
  PEAK_PACKET_TYPE_TRIP_SECONDARY = 0x16,
} peak_packet_type_t;

/*
 * Display -> ESC commands
 *
 * CYCLEIQ_POWER_OFF, len 0
 *   No payload.
 *
 * CYCLEIQ_POWER_ON, len 0
 *   No payload.
 *
 * CYCLEIQ_COMM_GEAR_SET, len 1
 *   byte 0: gear, uint8_t
 *   Current PEAK gear range is 1..6. The ESC should validate bounds.
 *
 * CYCLEIQ_COMM_MODE_SET, len 1
 *   byte 0: cycleiq_support_mode_t
 *
 * CYCLEIQ_COMM_RIDE_MODE_SET, len 1
 *   byte 0: cycleiq_ride_mode_t
 */

#define PEAK_PACKET_BATTERY_STATUS_LEN 5
typedef struct {
  uint8_t battery_percentage;      // percent, 0..100
  uint8_t battery_voltage_cV[2];   // uint16_t centivolts
  uint8_t battery_current_cA[2];   // int16_t centiamps
} peak_packet_battery_status_t;

#define PEAK_PACKET_BATTERY_ENERGY_LEN 4
typedef struct {
  uint8_t watt_hours_dWh[2];       // uint16_t deci-watt-hours
  uint8_t amp_hours_cAh[2];        // uint16_t centi-amp-hours
} peak_packet_battery_energy_t;

#define PEAK_PACKET_MOTOR_STATUS_LEN 6
typedef struct {
  int8_t motor_temperature_C;      // degrees C
  int8_t controller_temperature_C; // degrees C
  uint8_t motor_current_cA[2];     // int16_t centiamps
  uint8_t motor_rpm[2];            // uint16_t rpm
} peak_packet_motor_status_t;

#define PEAK_PACKET_CONTROLLER_STATE_LEN 3
typedef struct {
  uint8_t assist_level;
  uint8_t support_mode;            // cycleiq_support_mode_t
  uint8_t ride_mode;               // cycleiq_ride_mode_t
} peak_packet_controller_state_t;

#define PEAK_PACKET_LIVE_STATUS_LEN 4
typedef struct {
  uint8_t speed_ckmh[2];           // uint16_t centi-km/h
  uint8_t power_W[2];              // uint16_t watts
} peak_packet_live_status_t;

#define PEAK_PACKET_TRIP_PRIMARY_LEN 8
typedef struct {
  uint8_t trip_distance_m[4];      // uint32_t meters
  uint8_t trip_time_s[4];          // uint32_t seconds
} peak_packet_trip_primary_t;

#define PEAK_PACKET_TRIP_SECONDARY_LEN 3
typedef struct {
  uint8_t trip_average_speed_ckmh[2]; // uint16_t centi-km/h
  uint8_t trip_estimated_range_km;    // uint8_t kilometers
} peak_packet_trip_secondary_t;

static inline uint16_t cycleiq_u16_to_be(uint16_t value) {
  return (uint16_t)((value >> 8) | (value << 8));
}

static inline void cycleiq_write_be_u16(uint8_t *out, uint16_t value) {
  out[0] = (uint8_t)(value >> 8);
  out[1] = (uint8_t)value;
}

static inline void cycleiq_write_be_i16(uint8_t *out, int16_t value) {
  cycleiq_write_be_u16(out, (uint16_t)value);
}

static inline void cycleiq_write_be_u32(uint8_t *out, uint32_t value) {
  out[0] = (uint8_t)(value >> 24);
  out[1] = (uint8_t)(value >> 16);
  out[2] = (uint8_t)(value >> 8);
  out[3] = (uint8_t)value;
}

#endif
