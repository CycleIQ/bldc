#pragma GCC optimize("Os")

#include "app.h"

#include "cycleIQ/comm.h"
#include "cycleIQ/control.h"
#include "cycleIQ/data.h"
#include "cycleIQ/pas.h"
#include "cycleIQ/sensors.h"
#include "cycleIQ/service.h"

void app_custom_start(void) {
  cycleiq_data_init();
  cycleiq_comm_init();
  cycleiq_control_init();

  cycleiq_pas_init();
  cycleiq_pas_configure(&(cycleiq_pas_config){
      .pedal_rpm_start = 15.0f,
      .pedal_rpm_max = 240.0f,
#ifdef CYCLEIQ_HAS_2_WIRE_PAS
      .magnets = 18,
#else
      .magnets = 36,
#endif
  });

  cycleiq_sensors_start();
  cycleiq_service_start();
}

void app_custom_stop(void) {
  cycleiq_service_stop();
  cycleiq_sensors_stop();

  cycleiq_control_stop();
  cycleiq_pas_deinit();
  cycleiq_comm_deinit();
  cycleiq_pas_configure(&(cycleiq_pas_config){0});
}

void app_custom_configure(app_configuration *conf) {
  (void)conf;
}
