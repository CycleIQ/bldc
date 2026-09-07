#include "comm.h"

#include "ch.h"
#include "comm_can.h"
#include "data.h"
#include "datatypes.h"
#include "walk.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#define CYCLEIQ_LIVE_PERIOD_MS 100u
#define CYCLEIQ_STATE_PERIOD_MS 1000u
#define CYCLEIQ_BATTERY_PERIOD_MS 500u
#define CYCLEIQ_THERMALS_PERIOD_MS 250u

static bool telemetry_timing_initialized;
static systime_t next_live_time;
static systime_t next_battery_time;
static systime_t next_thermals_time;
static systime_t next_state_time;

static bool state_sent;
static uint8_t last_gear;
static cycleiq_support_mode_t last_support_mode;
static cycleiq_ride_mode_t last_ride_mode;
static bool last_walk_active;

static void cycleiq_transmit_frame(const cycleiq_frame_t *frame) {
  comm_can_transmit_eid(frame->id, frame->data, frame->len);
}

static bool cycleiq_time_due(systime_t now, systime_t due_time) {
  return (systime_t)(now - due_time) < (systime_t)(1u << 31);
}

static bool cycleiq_packet_due(systime_t now, systime_t *next_time,
                               systime_t period) {
  if (!cycleiq_time_due(now, *next_time)) {
    return false;
  }

  *next_time = now + period;
  return true;
}

static uint16_t cycleiq_to_u16(float value, float scale) {
  if (!isfinite(value) || value <= 0.0f) {
    return 0u;
  }

  value *= scale;
  if (value >= 65535.0f) {
    return 65535u;
  }

  return (uint16_t)value;
}

static int16_t cycleiq_to_i16(float value) {
  if (!isfinite(value)) {
    return 0;
  }
  if (value >= 32767.0f) {
    return 32767;
  }
  if (value <= -32768.0f) {
    return -32768;
  }

  return (int16_t)value;
}

static bool cycleiq_state_changed(bool walk_active) {
  return !state_sent || last_gear != cycleiq_data.current_gear ||
         last_support_mode != cycleiq_data.support_mode ||
         last_ride_mode != cycleiq_data.ride_mode ||
         last_walk_active != walk_active;
}

static void cycleiq_send_state(cycleiq_frame_t *frame, bool walk_active) {
  if (!cycleiq_telemetry_state(frame, cycleiq_data.current_gear,
                               cycleiq_data.support_mode,
                               cycleiq_data.ride_mode, walk_active)) {
    return;
  }

  cycleiq_transmit_frame(frame);
  state_sent = true;
  last_gear = cycleiq_data.current_gear;
  last_support_mode = cycleiq_data.support_mode;
  last_ride_mode = cycleiq_data.ride_mode;
  last_walk_active = walk_active;
}

static void cycleiq_send_live(cycleiq_frame_t *frame) {
  if (cycleiq_telemetry_live(frame, cycleiq_to_u16(cycleiq_data.speed_mps, 360.0f),
                             cycleiq_to_i16(cycleiq_data.motor_power_w))) {
    cycleiq_transmit_frame(frame);
  }
}

static void cycleiq_send_thermals(cycleiq_frame_t *frame) {
  if (cycleiq_telemetry_thermals(frame, cycleiq_data.motor_temperature_c,
                                 cycleiq_data.controller_temperature_c)) {
    cycleiq_transmit_frame(frame);
  }
}

static void cycleiq_send_battery(cycleiq_frame_t *frame) {
  if (cycleiq_telemetry_battery(
          frame, cycleiq_data.battery_level_pct,
          cycleiq_to_u16(cycleiq_data.battery_voltage_v, 100.0f))) {
    cycleiq_transmit_frame(frame);
  }
}

static void cycleiq_send_sync(void) {
  cycleiq_frame_t frame;
  cycleiq_send_state(&frame, cycleiq_walk_is_active());
  cycleiq_send_live(&frame);
  cycleiq_send_thermals(&frame);
  cycleiq_send_battery(&frame);
}

static bool cycleIQ_CAN_rx_callback(uint32_t id, uint8_t *data, uint8_t len) {
  cycleiq_frame_t frame;
  if (!cycleiq_frame_from_can(&frame, id, data, len) ||
      !cycleiq_frame_is_for_node(&frame, CYCLEIQ_ESC_CAN_ID)) {
    return false;
  }

  switch ((cycleiq_command_t)cycleiq_frame_type(&frame)) {
  case CYCLEIQ_COMMAND_GEAR_UP:
    if (frame.len == CYCLEIQ_COMMAND_EMPTY_LEN &&
        cycleiq_data.current_gear < cycleiq_data.max_gear) {
      (void)cycleiq_data_set_gear(cycleiq_data.current_gear + 1u);
    }
    break;

  case CYCLEIQ_COMMAND_GEAR_DOWN:
    if (frame.len == CYCLEIQ_COMMAND_EMPTY_LEN && cycleiq_data.current_gear > 0u) {
      (void)cycleiq_data_set_gear(cycleiq_data.current_gear - 1u);
    }
    break;

  case CYCLEIQ_COMMAND_SET_SUPPORT_MODE: {
    cycleiq_support_mode_t mode;
    if (cycleiq_read_command_support_mode(&frame, &mode)) {
      (void)cycleiq_data_set_support_mode(mode);
    }
    break;
  }

  case CYCLEIQ_COMMAND_SET_RIDE_MODE: {
    cycleiq_ride_mode_t mode;
    if (cycleiq_read_command_ride_mode(&frame, &mode)) {
      (void)cycleiq_data_set_ride_mode(mode);
    }
    break;
  }

  case CYCLEIQ_COMMAND_SET_WALK: {
    bool enabled;
    if (cycleiq_read_command_walk(&frame, &enabled) &&
        (!enabled || cycleiq_data.motor_enabled)) {
      cycleiq_walk_set_enabled(enabled);
    }
    break;
  }

  case CYCLEIQ_COMMAND_SYNC_REQUEST:
    if (frame.len == CYCLEIQ_COMMAND_EMPTY_LEN) {
      cycleiq_send_sync();
    }
    break;

  default:
    break;
  }

  return true;
}

void cycleiq_comm_init(void) {
  comm_can_set_eid_rx_callback(&cycleIQ_CAN_rx_callback);
  telemetry_timing_initialized = false;
  state_sent = false;
}

void cycleiq_comm_deinit(void) {
  comm_can_set_eid_rx_callback(NULL);
}

void cycleiq_comm_loop(void) {
  cycleiq_frame_t frame;
  systime_t now = chVTGetSystemTimeX();

  if (!telemetry_timing_initialized) {
    next_live_time = now;
    next_thermals_time = now + MS2ST(50);
    next_battery_time = now + MS2ST(100);
    next_state_time = now + MS2ST(150);
    telemetry_timing_initialized = true;
  }

  if (cycleiq_packet_due(now, &next_live_time, MS2ST(CYCLEIQ_LIVE_PERIOD_MS))) {
    cycleiq_send_live(&frame);
  }
  if (cycleiq_packet_due(now, &next_thermals_time,
                         MS2ST(CYCLEIQ_THERMALS_PERIOD_MS))) {
    cycleiq_send_thermals(&frame);
  }
  if (cycleiq_packet_due(now, &next_battery_time,
                         MS2ST(CYCLEIQ_BATTERY_PERIOD_MS))) {
    cycleiq_send_battery(&frame);
  }

  bool walk_active = cycleiq_walk_is_active();
  if (cycleiq_state_changed(walk_active) ||
      cycleiq_packet_due(now, &next_state_time, MS2ST(CYCLEIQ_STATE_PERIOD_MS))) {
    cycleiq_send_state(&frame, walk_active);
    next_state_time = now + MS2ST(CYCLEIQ_STATE_PERIOD_MS);
  }
}
