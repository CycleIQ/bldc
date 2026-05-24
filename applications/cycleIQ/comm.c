#include "comm.h"
#include "comm_can.h"
#include "commands.h"

#include "data.h"
#include "datatypes.h"

#include "cycleiq_utils.h"

#define PEAK_CAN_ID 0x6Au
#define CYCLEIQ_CAN_ID 0x6Bu

#define PEAK_CAN_FRAME_ID(type_or_command)                                     \
  (((uint32_t)PEAK_CAN_ID << 8) | (uint8_t)(type_or_command))
#define CYCLEIQ_CAN_NODE_ID(can_id) (((can_id) >> 8) & 0xFF)
#define CYCLEIQ_CAN_PACKET_TYPE(can_id) ((can_id) & 0xFF)

#define PEAK_PACKET_TYPE_BATTERY_STATUS 0x10u
#define PEAK_PACKET_TYPE_BATTERY_ENERGY 0x11u
#define PEAK_PACKET_TYPE_MOTOR_STATUS 0x12u
#define PEAK_PACKET_TYPE_CONTROLLER_STATE 0x13u
#define PEAK_PACKET_TYPE_LIVE_STATUS 0x14u
#define PEAK_PACKET_TYPE_TRIP_PRIMARY 0x15u
#define PEAK_PACKET_TYPE_TRIP_SECONDARY 0x16u

#define PEAK_LIVE_STATUS_PERIOD_MS 100u
#define PEAK_CONTROLLER_STATE_PERIOD_MS 1000u
#define PEAK_BATTERY_STATUS_PERIOD_MS 500u
#define PEAK_MOTOR_STATUS_PERIOD_MS 250u
#define PEAK_SLOW_PACKET_PERIOD_MS 1000u

static volatile ring_buffer_t send_buffer;

static bool telemetry_timing_initialized;
static systime_t next_battery_status_time;
static systime_t next_battery_energy_time;
static systime_t next_motor_status_time;
static systime_t next_controller_state_time;
static systime_t next_live_status_time;
static systime_t next_trip_primary_time;
static systime_t next_trip_secondary_time;

static bool controller_state_sent;
static uint8_t last_controller_gear;
static cycleiq_support_mode_t last_controller_support_mode;
static cycleiq_ride_mode_t last_controller_ride_mode;

static uint16_t cycleiq_float_to_u16(float value, float scale) {
  if (value <= 0.0f) {
    return 0;
  }

  value *= scale;
  if (value >= 65535.0f) {
    return 65535;
  }

  return (uint16_t)value;
}

static int16_t cycleiq_float_to_i16(float value, float scale) {
  value *= scale;
  if (value >= 32767.0f) {
    return 32767;
  }
  if (value <= -32768.0f) {
    return -32768;
  }

  return (int16_t)value;
}

static uint8_t cycleiq_float_to_u8(float value, float scale) {
  if (value <= 0.0f) {
    return 0;
  }

  value *= scale;
  if (value >= 255.0f) {
    return 255;
  }

  return (uint8_t)value;
}

static uint32_t cycleiq_float_to_u32(float value, float scale) {
  if (value <= 0.0f) {
    return 0;
  }

  value *= scale;
  if (value >= 4294967295.0f) {
    return 4294967295u;
  }

  return (uint32_t)value;
}

static void cycleiq_write_be_u16(uint8_t *out, uint16_t value) {
  out[0] = (uint8_t)(value >> 8);
  out[1] = (uint8_t)value;
}

static void cycleiq_write_be_i16(uint8_t *out, int16_t value) {
  cycleiq_write_be_u16(out, (uint16_t)value);
}

static void cycleiq_write_be_u32(uint8_t *out, uint32_t value) {
  out[0] = (uint8_t)(value >> 24);
  out[1] = (uint8_t)(value >> 16);
  out[2] = (uint8_t)(value >> 8);
  out[3] = (uint8_t)value;
}

static void cycleiq_transmit_packet(uint8_t packet_type, const uint8_t *data,
                                    uint8_t len) {
  comm_can_transmit_eid(PEAK_CAN_FRAME_ID(packet_type), data, len);
}

static bool cycleiq_time_due(systime_t now, systime_t due_time) {
  return (systime_t)(now - due_time) < (systime_t)(1u << 31);
}

static bool cycleiq_packet_due(systime_t now, systime_t *next_time,
                               systime_t period) {
  if (cycleiq_time_due(now, *next_time)) {
    *next_time = now + period;
    return true;
  }

  return false;
}

static bool cycleiq_controller_state_changed(void) {
  return !controller_state_sent ||
         last_controller_gear != cycleiq_data.current_gear ||
         last_controller_support_mode != cycleiq_data.support_mode ||
         last_controller_ride_mode != cycleiq_data.ride_mode;
}

static void cycleiq_send_controller_state(uint8_t *data) {
  data[0] = cycleiq_data.current_gear;
  data[1] = (uint8_t)cycleiq_data.support_mode;
  data[2] = (uint8_t)cycleiq_data.ride_mode;
  cycleiq_transmit_packet(PEAK_PACKET_TYPE_CONTROLLER_STATE, data, 3);

  controller_state_sent = true;
  last_controller_gear = cycleiq_data.current_gear;
  last_controller_support_mode = cycleiq_data.support_mode;
  last_controller_ride_mode = cycleiq_data.ride_mode;
}

static bool cycleIQ_CAN_rx_callback(uint32_t id, uint8_t *data, uint8_t len) {
  if (CYCLEIQ_CAN_NODE_ID(id) != CYCLEIQ_CAN_ID) {
    return false;
  }

  cycleiq_command_t cmd = (cycleiq_command_t)CYCLEIQ_CAN_PACKET_TYPE(id);

  switch (cmd) {
  case CYCLEIQ_POWER_OFF:
    // cycleiq_motor_on = false;
    mc_interface_set_current(0.0f); // Turn off motor
    break;
  case CYCLEIQ_POWER_ON:
    // cycleiq_motor_on = true;
    break;
  case CYCLEIQ_COMM_GEAR_SET:
    if (len < 1)
      break;
    if (data[0] < 1 || data[0] > cycleiq_data.max_gear)
      break; // Invalid gear
    cycleiq_data.current_gear = data[0];
    break;
  case CYCLEIQ_COMM_MODE_SET:
    if (len < 1)
      break;
    if (data[0] > CYCLEIQ_MODE_HYBRID)
      break; // Invalid mode
    cycleiq_data.support_mode = data[0];
    break;
  case CYCLEIQ_COMM_RIDE_MODE_SET:
    if (len < 1)
      break;
    if (data[0] > CYCLEIQ_RIDE_MODE_MOUNTAIN)
      break; // Invalid mode

    cycleiq_data.ride_mode = data[0];
    cycleiq_change_ride_mode();
    break;
  case CYCLEIQ_COMM_SCREEN_SET:
    if (len < 1)
      break;
    if (data[0] > CYCLEIQ_SCREEN_GRAPH)
      break; // Invalid screen
    cycleiq_data.screen = (cycleiq_screen_t)data[0];
    break;
  default:
    break;
  }

  return true;
}

void cycleiq_comm_init(void) {
  comm_can_set_eid_rx_callback(&cycleIQ_CAN_rx_callback);
  ring_buffer_init(&send_buffer, 16); // Initialize send buffer with size 16

  telemetry_timing_initialized = false;
  controller_state_sent = false;
}

void cycleiq_comm_deinit(void) {
  comm_can_set_eid_rx_callback(NULL);
  ring_buffer_clear(&send_buffer); // Clear the send buffer
  ring_buffer_free(&send_buffer);  // Free the send buffer resources
}

void cycleiq_comm_loop(void) {
  uint8_t data[8] = {0};
  systime_t now = chVTGetSystemTimeX();

  if (!telemetry_timing_initialized) {
    next_live_status_time = now;
    next_battery_status_time = now + MS2ST(50);
    next_motor_status_time = now + MS2ST(150);
    next_controller_state_time = now + MS2ST(250);
    next_battery_energy_time = now + MS2ST(350);
    next_trip_primary_time = now + MS2ST(550);
    next_trip_secondary_time = now + MS2ST(750);
    telemetry_timing_initialized = true;
  }

  if (cycleiq_packet_due(now, &next_live_status_time,
                         MS2ST(PEAK_LIVE_STATUS_PERIOD_MS))) {
    cycleiq_write_be_u16(&data[0],
                         cycleiq_float_to_u16(cycleiq_data.speed, 360.0f));
    cycleiq_write_be_u16(&data[2], cycleiq_data.motor_power);
    cycleiq_transmit_packet(PEAK_PACKET_TYPE_LIVE_STATUS, data, 4);
  }

  if (cycleiq_packet_due(now, &next_battery_status_time,
                         MS2ST(PEAK_BATTERY_STATUS_PERIOD_MS))) {
    data[0] = cycleiq_data.battery_level;
    cycleiq_write_be_u16(
        &data[1], cycleiq_float_to_u16(cycleiq_data.battery_voltage, 100.0f));
    cycleiq_write_be_i16(
        &data[3], cycleiq_float_to_i16(cycleiq_data.battery_current, 100.0f));
    cycleiq_transmit_packet(PEAK_PACKET_TYPE_BATTERY_STATUS, data, 5);
  }

  if (cycleiq_packet_due(now, &next_motor_status_time,
                         MS2ST(PEAK_MOTOR_STATUS_PERIOD_MS))) {
    data[0] = (uint8_t)cycleiq_data.motor_temperature;
    data[1] = (uint8_t)cycleiq_data.controller_temperature;
    cycleiq_write_be_i16(
        &data[2], cycleiq_float_to_i16(cycleiq_data.motor_current, 100.0f));
    cycleiq_write_be_u16(&data[4],
                         cycleiq_float_to_u16(cycleiq_data.motor_rpm, 1.0f));
    cycleiq_transmit_packet(PEAK_PACKET_TYPE_MOTOR_STATUS, data, 6);
  }

  if (cycleiq_controller_state_changed()) {
    cycleiq_send_controller_state(data);
    next_controller_state_time = now + MS2ST(PEAK_CONTROLLER_STATE_PERIOD_MS);
  } else if (cycleiq_packet_due(now, &next_controller_state_time,
                                MS2ST(PEAK_CONTROLLER_STATE_PERIOD_MS))) {
    cycleiq_send_controller_state(data);
  }

  if (cycleiq_packet_due(now, &next_battery_energy_time,
                         MS2ST(PEAK_SLOW_PACKET_PERIOD_MS))) {
    cycleiq_write_be_u16(&data[0],
                         cycleiq_float_to_u16(cycleiq_data.watt_hours, 10.0f));
    cycleiq_write_be_u16(&data[2],
                         cycleiq_float_to_u16(cycleiq_data.amp_hours, 100.0f));
    cycleiq_transmit_packet(PEAK_PACKET_TYPE_BATTERY_ENERGY, data, 4);
  }

  if (cycleiq_packet_due(now, &next_trip_primary_time,
                         MS2ST(PEAK_SLOW_PACKET_PERIOD_MS))) {
    cycleiq_write_be_u32(
        &data[0], cycleiq_float_to_u32(cycleiq_data.trip_distance, 1000.0f));
    cycleiq_write_be_u32(&data[4], 0); // No trip time data is tracked yet.
    cycleiq_transmit_packet(PEAK_PACKET_TYPE_TRIP_PRIMARY, data, 8);
  }

  if (cycleiq_packet_due(now, &next_trip_secondary_time,
                         MS2ST(PEAK_SLOW_PACKET_PERIOD_MS))) {
    cycleiq_write_be_u16(&data[0],
                         0); // No trip average speed data is tracked yet.
    data[2] = cycleiq_float_to_u8(cycleiq_data.range, 1.0f);
    cycleiq_transmit_packet(PEAK_PACKET_TYPE_TRIP_SECONDARY, data, 3);
  }
}
