#pragma GCC optimize("Os")

#include "app.h"
#include "ch.h"
#include "hal.h"

#include "comm_can.h"
#include "mc_interface.h"

#define CYCLEIQ_CAN_ID 0xCCA         // Example CAN ID for CycleIQ
#define CYCLEIQ_DISPLAY_CAN_ID 0xCCB // Example CAN ID for CycleIQ display

static THD_FUNCTION(pas_thread, arg);
static THD_WORKING_AREA(pas_thread_wa, 1024);

static volatile bool stop_now = true;
static volatile bool is_running = false;

static volatile float pas_rpm = 0.0;
static volatile float torque = 0.0;

static bool cycleIQ_CAN_rx_callback(uint32_t id, uint8_t *data, uint8_t len)
{
  if (id == CYCLEIQ_DISPLAY_CAN_ID)
  {
    return true;
  }

  return false;
}

void app_custom_start(void)
{
  stop_now = false;
  comm_can_set_sid_rx_callback(&cycleIQ_CAN_rx_callback);
  chThdCreateStatic(pas_thread_wa, sizeof(pas_thread_wa), NORMALPRIO, pas_thread, NULL);
};
void app_custom_stop(void) {};
void app_custom_configure(app_configuration *conf) {};

static THD_FUNCTION(pas_thread, arg)
{
  (void)arg;
  chRegSetThreadName("CYCLEIQ");
  is_running = true;

  palSetPadMode(GPIOB, 10, PAL_MODE_INPUT_PULLUP);

  for (;;)
  {
    if (stop_now)
    {
      is_running = false;
      return;
    }

    uint8_t data[8];
    data[0] = (uint8_t)(pas_rpm * 10); // Example conversion to a byte
    data[1] = (uint8_t)(torque * 10);  // Example conversion to a byte
    data[2] = 0;                       // Reserved for future use
    data[3] = 0;                       // Reserved for future use
    data[4] = 0;                       // Reserved for future use
    data[5] = 0;                       // Reserved for future use
    data[6] = 0;                       // Reserved for future use
    data[7] = 0;                       // Reserved for future use

    comm_can_transmit_sid(CYCLEIQ_CAN_ID, data, 8);    

    mc_interface_set_current(0.0); // Set current to zero for safety

    chThdSleepMilliseconds(1); // 1 kHz update rate
  }
}
