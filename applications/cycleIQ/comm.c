#include "comm.h"

#include "ch.h"
#include "comm_can.h"
#include "control.h"
#include "data.h"
#include "datatypes.h"

#include <stdbool.h>
#include <stdint.h>

#define PEAK_LIVE_STATUS_PERIOD_MS 100u
#define PEAK_CONTROLLER_STATE_PERIOD_MS 1000u
#define PEAK_BATTERY_STATUS_PERIOD_MS 500u
#define PEAK_MOTOR_STATUS_PERIOD_MS 250u
#define PEAK_SLOW_PACKET_PERIOD_MS 1000u

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

static void cycleiq_transmit_frame(const cycleiq_frame_t *frame) {
  comm_can_transmit_eid(frame->id, frame->data, frame->len);
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

static void cycleiq_send_controller_state(cycleiq_frame_t *frame) {
  if (!cycleiq_telemetry_controller_state(
          frame, cycleiq_data.current_gear, cycleiq_data.support_mode,
          cycleiq_data.ride_mode)) {
    return;
  }

  cycleiq_transmit_frame(frame);

  controller_state_sent = true;
  last_controller_gear = cycleiq_data.current_gear;
  last_controller_support_mode = cycleiq_data.support_mode;
  last_controller_ride_mode = cycleiq_data.ride_mode;
}

static void cycleiq_send_protocol_version(cycleiq_frame_t *frame) {
  if (cycleiq_telemetry_protocol_version(frame)) {
    cycleiq_transmit_frame(frame);
  }
}

static void cycleiq_send_config_ack(cycleiq_frame_t *frame, uint8_t command,
                                    cycleiq_config_status_t status,
                                    cycleiq_config_field_t detail) {
  if (cycleiq_telemetry_config_ack(frame, command, status, detail)) {
    cycleiq_transmit_frame(frame);
  }
}

static uint16_t cycleiq_read_be_u16(const uint8_t *data) {
  return ((uint16_t)data[0] << 8) | (uint16_t)data[1];
}

static void cycleiq_handle_config_get(cycleiq_frame_t *frame) {
  cycleiq_config_field_t field = CYCLEIQ_CONFIG_FIELD_ALL;
  if (!cycleiq_command_read_config_get(frame, &field)) {
    cycleiq_config_status_t status =
        frame->len == CYCLEIQ_COMMAND_CONFIG_GET_LEN
            ? CYCLEIQ_CONFIG_STATUS_UNKNOWN_FIELD
            : CYCLEIQ_CONFIG_STATUS_MALFORMED;
    cycleiq_send_config_ack(frame, CYCLEIQ_COMM_CONFIG_GET, status,
                            (cycleiq_config_field_t)frame->data[0]);
    return;
  }

  if (field == CYCLEIQ_CONFIG_FIELD_ALL) {
    cycleiq_config_snapshot_t snapshot;
    cycleiq_config_get_snapshot(&snapshot);
    if (cycleiq_telemetry_config_snapshot(frame, &snapshot)) {
      cycleiq_transmit_frame(frame);
    }
    return;
  }

  uint16_t value = 0u;
  cycleiq_config_status_t status = cycleiq_config_get_field(field, &value);
  if (status == CYCLEIQ_CONFIG_STATUS_OK) {
    if (cycleiq_telemetry_config_field(frame, field, value)) {
      cycleiq_transmit_frame(frame);
    }
  } else {
    cycleiq_send_config_ack(frame, CYCLEIQ_COMM_CONFIG_GET, status, field);
  }
}

static void cycleiq_handle_config_set(cycleiq_frame_t *frame) {
  cycleiq_config_op_t op = CYCLEIQ_CONFIG_OP_SET_FIELD;
  if (!cycleiq_command_read_config_op(frame, &op)) {
    cycleiq_send_config_ack(frame, CYCLEIQ_COMM_CONFIG_SET,
                            CYCLEIQ_CONFIG_STATUS_MALFORMED,
                            CYCLEIQ_CONFIG_FIELD_ALL);
    return;
  }

  cycleiq_config_status_t status = CYCLEIQ_CONFIG_STATUS_OK;
  cycleiq_config_field_t detail = CYCLEIQ_CONFIG_FIELD_ALL;

  switch (op) {
  case CYCLEIQ_CONFIG_OP_SET_FIELD:
    if (frame->len != CYCLEIQ_COMMAND_CONFIG_SET_FIELD_LEN) {
      status = CYCLEIQ_CONFIG_STATUS_MALFORMED;
      break;
    }
    detail = (cycleiq_config_field_t)frame->data[1];
    status = cycleiq_config_stage_field(detail,
                                        cycleiq_read_be_u16(&frame->data[2]));
    break;

  case CYCLEIQ_CONFIG_OP_SET_SNAPSHOT: {
    cycleiq_config_snapshot_t snapshot;
    if (!cycleiq_command_read_config_snapshot(frame, &snapshot)) {
      status = CYCLEIQ_CONFIG_STATUS_MALFORMED;
      break;
    }
    status = cycleiq_config_stage_snapshot(&snapshot);
    break;
  }

  case CYCLEIQ_CONFIG_OP_COMMIT:
    if (frame->len != CYCLEIQ_COMMAND_CONFIG_SET_OP_LEN) {
      status = CYCLEIQ_CONFIG_STATUS_MALFORMED;
      break;
    }
    status = cycleiq_config_commit();
    break;

  case CYCLEIQ_CONFIG_OP_DISCARD:
    if (frame->len != CYCLEIQ_COMMAND_CONFIG_SET_OP_LEN) {
      status = CYCLEIQ_CONFIG_STATUS_MALFORMED;
      break;
    }
    cycleiq_config_discard_staged();
    break;

  default:
    status = CYCLEIQ_CONFIG_STATUS_MALFORMED;
    break;
  }

  cycleiq_send_config_ack(frame, CYCLEIQ_COMM_CONFIG_SET, status, detail);
}

static bool cycleIQ_CAN_rx_callback(uint32_t id, uint8_t *data, uint8_t len) {
  cycleiq_frame_t frame;
  if (!cycleiq_frame_from_can(&frame, id, data, len)) {
    return false;
  }

  if (!cycleiq_frame_is_for_node(&frame, CYCLEIQ_CAN_ID)) {
    return false;
  }

  cycleiq_command_t cmd = (cycleiq_command_t)cycleiq_frame_type(&frame);
  uint8_t value = 0;

  switch (cmd) {
  case CYCLEIQ_POWER_OFF:
    cycleiq_data_set_motor_enabled(false);
    cycleiq_control_stop();
    break;

  case CYCLEIQ_POWER_ON:
    cycleiq_data_set_motor_enabled(true);
    break;

  case CYCLEIQ_COMM_GEAR_SET:
    if (cycleiq_command_read_u8(&frame, &value)) {
      (void)cycleiq_data_set_gear(value);
    }
    break;

  case CYCLEIQ_COMM_MODE_SET:
    if (cycleiq_command_read_u8(&frame, &value)) {
      (void)cycleiq_data_set_support_mode((cycleiq_support_mode_t)value);
    }
    break;

  case CYCLEIQ_COMM_RIDE_MODE_SET:
    if (cycleiq_command_read_u8(&frame, &value)) {
      (void)cycleiq_data_set_ride_mode((cycleiq_ride_mode_t)value);
    }
    break;

  case CYCLEIQ_COMM_SCREEN_SET:
    if (cycleiq_command_read_u8(&frame, &value)) {
      (void)cycleiq_data_set_screen((cycleiq_screen_t)value);
    }
    break;

  case CYCLEIQ_COMM_CONFIG_GET:
    cycleiq_handle_config_get(&frame);
    break;

  case CYCLEIQ_COMM_CONFIG_SET:
    cycleiq_handle_config_set(&frame);
    break;

  case CYCLEIQ_COMM_PROTOCOL_VERSION_GET:
    cycleiq_send_protocol_version(&frame);
    break;

  default:
    break;
  }

  return true;
}

void cycleiq_comm_init(void) {
  comm_can_set_eid_rx_callback(&cycleIQ_CAN_rx_callback);

  telemetry_timing_initialized = false;
  controller_state_sent = false;
}

void cycleiq_comm_deinit(void) {
  comm_can_set_eid_rx_callback(NULL);
}

void cycleiq_comm_loop(void) {
  cycleiq_frame_t frame;
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
    if (cycleiq_telemetry_live_status(&frame, cycleiq_data.speed_mps,
                                      cycleiq_data.motor_power_w)) {
      cycleiq_transmit_frame(&frame);
    }
  }

  if (cycleiq_packet_due(now, &next_battery_status_time,
                         MS2ST(PEAK_BATTERY_STATUS_PERIOD_MS))) {
    if (cycleiq_telemetry_battery_status(
            &frame, cycleiq_data.battery_level_pct,
            cycleiq_data.battery_voltage_v, cycleiq_data.battery_current_a)) {
      cycleiq_transmit_frame(&frame);
    }
  }

  if (cycleiq_packet_due(now, &next_motor_status_time,
                         MS2ST(PEAK_MOTOR_STATUS_PERIOD_MS))) {
    if (cycleiq_telemetry_motor_status(
            &frame, cycleiq_data.motor_temperature_c,
            cycleiq_data.controller_temperature_c, cycleiq_data.motor_current_a,
            cycleiq_data.motor_rpm)) {
      cycleiq_transmit_frame(&frame);
    }
  }

  if (cycleiq_controller_state_changed()) {
    cycleiq_send_controller_state(&frame);
    next_controller_state_time = now + MS2ST(PEAK_CONTROLLER_STATE_PERIOD_MS);
  } else if (cycleiq_packet_due(now, &next_controller_state_time,
                                MS2ST(PEAK_CONTROLLER_STATE_PERIOD_MS))) {
    cycleiq_send_controller_state(&frame);
  }

  if (cycleiq_packet_due(now, &next_battery_energy_time,
                         MS2ST(PEAK_SLOW_PACKET_PERIOD_MS))) {
    if (cycleiq_telemetry_battery_energy(&frame, cycleiq_data.watt_hours,
                                         cycleiq_data.amp_hours)) {
      cycleiq_transmit_frame(&frame);
    }
  }

  if (cycleiq_packet_due(now, &next_trip_primary_time,
                         MS2ST(PEAK_SLOW_PACKET_PERIOD_MS))) {
    if (cycleiq_telemetry_trip_primary(&frame, cycleiq_data.trip_distance_km,
                                       cycleiq_data.trip_time_s)) {
      cycleiq_transmit_frame(&frame);
    }
  }

  if (cycleiq_packet_due(now, &next_trip_secondary_time,
                         MS2ST(PEAK_SLOW_PACKET_PERIOD_MS))) {
    if (cycleiq_telemetry_trip_secondary(&frame,
                                         cycleiq_data.average_speed_mps,
                                         cycleiq_data.range_km)) {
      cycleiq_transmit_frame(&frame);
    }
  }
}
