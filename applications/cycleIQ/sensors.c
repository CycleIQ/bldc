#include "sensors.h"

#include "ch.h"
#include "data.h"
#include "hal.h"
#include "hw.h"
#include "mc_interface.h"
#include "stm32f4xx_conf.h"
#include "timeout.h"
#include "utils_math.h"

#include <stdbool.h>
#include <stdint.h>

#define CYCLEIQ_SPEED_SENSOR_PERIOD_US 200u
#define CYCLEIQ_MOTOR_SENSOR_UPDATE_DIVIDER 50u
#define CYCLEIQ_MOTOR_TEMP_SAMPLE_GAP_MS 3u
#define CYCLEIQ_MOTOR_SPEED_INPUT_THRESHOLD 50

static THD_FUNCTION(cycleiq_speed_sensor_thread, arg);
static THD_WORKING_AREA(cycleiq_speed_sensor_thread_wa, 256);

static volatile bool stop_now = true;
static volatile bool is_running;

static volatile float motor_rpm;
static volatile int8_t motor_temperature_c;

void cycleiq_sensors_start(void) {
  stop_now = false;
  motor_rpm = 0.0f;
  motor_temperature_c = 0;
  chThdCreateStatic(cycleiq_speed_sensor_thread_wa,
                    sizeof(cycleiq_speed_sensor_thread_wa), NORMALPRIO,
                    cycleiq_speed_sensor_thread, NULL);
}

void cycleiq_sensors_stop(void) {
  stop_now = true;
  while (is_running) {
    chThdSleepMilliseconds(1);
  }
}

static THD_FUNCTION(cycleiq_speed_sensor_thread, arg) {
  (void)arg;
  chRegSetThreadName("CYCLEIQ_SENSORS");
  is_running = true;

  systime_t time = chVTGetSystemTimeX();
  systime_t last_pulse_time = 0;
  systime_t last_temperature_time = 0;
  bool last_state = false;
  uint8_t update_counter = 0;

  for (;;) {
    if (stop_now) {
      is_running = false;
      return;
    }

    chThdSleepUntilWindowed(time, time + US2ST(CYCLEIQ_SPEED_SENSOR_PERIOD_US));
    time += US2ST(CYCLEIQ_SPEED_SENSOR_PERIOD_US);

    bool state = (ADC_Value[ADC_IND_TEMP_MOTOR] <
                  CYCLEIQ_MOTOR_SPEED_INPUT_THRESHOLD);
    if (state && !last_state) {
      systime_t pulse_duration = chVTTimeElapsedSinceX(last_pulse_time);
      last_pulse_time = time;

      float pulse_duration_s = (float)ST2US(pulse_duration) * 1e-6f;
      if (pulse_duration_s > 0.0f) {
        float current_rpm = 60.0f / pulse_duration_s;
        UTILS_LP_MOVING_AVG_APPROX(motor_rpm, current_rpm, 5);
      }
    }

    last_state = state;

    if (!state &&
        time - last_pulse_time > MS2ST(CYCLEIQ_MOTOR_TEMP_SAMPLE_GAP_MS) &&
        time - last_temperature_time >
            MS2ST(CYCLEIQ_MOTOR_TEMP_SAMPLE_GAP_MS)) {
      last_temperature_time = time;

      int temperature_c = (int)NTC_TEMP_MOTOR(3435.0f);
      utils_truncate_number_int(&temperature_c, -128, 127);
      int8_t current_temperature_c = (int8_t)temperature_c;
      UTILS_LP_MOVING_AVG_APPROX(motor_temperature_c, current_temperature_c,
                                 10);
    }

    if (++update_counter >= CYCLEIQ_MOTOR_SENSOR_UPDATE_DIVIDER) {
      update_counter = 0;
      cycleiq_data_motor_sensor_update(motor_rpm, motor_temperature_c);
    }

    timeout_reset();
  }
}
