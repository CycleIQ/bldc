#include "comm.h"
#include "commands.h"
#include "comm_can.h"

#include "datatypes.h"
#include "data.h"

#include "cycleiq_utils.h"

#define PEAK_CAN_ID 0x6Au
#define CYCLEIQ_CAN_ID 0x6Bu

static volatile ring_buffer_t send_buffer;

#ifdef CYCLEIQ_HIGH_POWER
static const int max_gear = 6; // Maximum gear for high power mode
#else
static const int max_gear = 5; // Maximum gear for normal mode
#endif

static bool cycleIQ_CAN_rx_callback(uint32_t id, uint8_t *data, uint8_t len)
{
  if ((id & 0xFF) != CYCLEIQ_CAN_ID)
  {
    return false;
  }

  cycleiq_command_t cmd = (cycleiq_command_t)(id >> 8) & 0xFF;

  switch (cmd)
  {
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
    if (data[0] > max_gear)
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
  default:
    break;
  }

  return true;
}

void cycleiq_comm_init(void)
{
  comm_can_set_eid_rx_callback(&cycleIQ_CAN_rx_callback);
  ring_buffer_init(&send_buffer, 16); // Initialize send buffer with size 16
}

void cycleiq_comm_deinit(void)
{
  comm_can_set_eid_rx_callback(NULL);
  ring_buffer_clear(&send_buffer); // Clear the send buffer
  ring_buffer_free(&send_buffer);  // Free the send buffer resources
}

void cycleiq_comm_loop(void)
{
  if (!ring_buffer_is_empty(&send_buffer))
  {
    // Prepare data to send
  }

  switch (cycleiq_data.screen)
  {
  case CYCLEIQ_SCREEN_MAIN:
  {
    uint8_t data[8] = {0};
    // Send part 1
    data[0] = 0; // First part of the data packet
    data[1] = cycleiq_data.battery_level;
    data[2] = (uint8_t)(cycleiq_data.speed * 10.0f) >> 8;                                               // Speed in km/h, scaled by 10
    data[3] = (uint8_t)(cycleiq_data.speed * 10.0f) & 0xFF;                                             // Speed in km/h, scaled by 10
    data[4] = cycleiq_data.current_gear << 3 | cycleiq_data.support_mode << 1 | cycleiq_data.ride_mode; // Current gear and support mode
    data[5] = (uint8_t)(cycleiq_data.motor_temperature);
    data[6] = (uint8_t)(cycleiq_data.motor_power >> 8);   // Motor power in watts, high byte
    data[7] = (uint8_t)(cycleiq_data.motor_power & 0xFF); // Motor power in watts, low byte

    comm_can_transmit_eid(
        PEAK_CAN_ID | (CYCLEIQ_SCREEN_MAIN << 8),
        data, 8);

    data[0] = 1; // Second part of the data packet
    data[1] = (uint8_t)(cycleiq_data.trip_distance * 10.0f) >> 8;
    data[2] = (uint8_t)(cycleiq_data.trip_distance * 10.0f) & 0xFF; // Trip distance in km, scaled by 10
    data[3] = (uint8_t)(cycleiq_data.range) >> 8;
    data[4] = (uint8_t)(cycleiq_data.range) & 0xFF; // Range in km,
    comm_can_transmit_eid(
        PEAK_CAN_ID | (CYCLEIQ_SCREEN_MAIN << 8),
        data, 5);
  }
  break;
  default:
    break;
  }
}