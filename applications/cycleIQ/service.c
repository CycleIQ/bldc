#include "service.h"

#include "ch.h"
#include "comm.h"
#include "control.h"
#include "data.h"
#include "pas.h"
#include "timeout.h"

#include <stdbool.h>

#define CYCLEIQ_SERVICE_PERIOD_MS 100u

static THD_FUNCTION(cycleiq_service_thread, arg);
static THD_WORKING_AREA(cycleiq_service_thread_wa, 1024);

static volatile bool stop_now = true;
static volatile bool is_running;

void cycleiq_service_start(void) {
  stop_now = false;
  chThdCreateStatic(cycleiq_service_thread_wa, sizeof(cycleiq_service_thread_wa),
                    NORMALPRIO, cycleiq_service_thread, NULL);
}

void cycleiq_service_stop(void) {
  stop_now = true;
  while (is_running) {
    chThdSleepMilliseconds(1);
  }
}

static THD_FUNCTION(cycleiq_service_thread, arg) {
  (void)arg;
  chRegSetThreadName("CYCLEIQ");
  is_running = true;

  for (;;) {
    if (stop_now) {
      cycleiq_control_stop();
      is_running = false;
      return;
    }

    chThdSleepMilliseconds(CYCLEIQ_SERVICE_PERIOD_MS);

    cycleiq_pas_loop();
    cycleiq_data_loop();
    cycleiq_comm_loop();
    cycleiq_control_loop();

    timeout_reset();
  }
}
