#include "walk.h"

#include "ch.h"

#define CYCLEIQ_WALK_COMMAND_TIMEOUT_MS 1000u

static bool walk_active;
static systime_t last_walk_refresh_time;

void cycleiq_walk_init(void) {
  walk_active = false;
  last_walk_refresh_time = 0;
}

void cycleiq_walk_set_enabled(bool enabled) {
  systime_t now = chVTGetSystemTimeX();

  chSysLock();
  if (enabled) {
    last_walk_refresh_time = now;
  }
  walk_active = enabled;
  chSysUnlock();
}

void cycleiq_walk_loop(void) {
  systime_t now = chVTGetSystemTimeX();

  chSysLock();
  if (walk_active &&
      (systime_t)(now - last_walk_refresh_time) >=
          MS2ST(CYCLEIQ_WALK_COMMAND_TIMEOUT_MS)) {
    walk_active = false;
  }
  chSysUnlock();
}

bool cycleiq_walk_is_active(void) {
  bool active;

  chSysLock();
  active = walk_active;
  chSysUnlock();

  return active;
}
