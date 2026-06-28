#include "sensors.h"

#include "ch.h"
#include "commands.h"
#include "data.h"
#include "hal.h"
#include "hw.h"
#include "isr_vector_table.h"
#include "mc_interface.h"
#include "stm32f4xx_conf.h"
#include "terminal.h"
#include "timeout.h"
#include "utils_math.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#define CYCLEIQ_SPEED_SENSOR_SAMPLE_RATE_HZ 5000u
#define CYCLEIQ_SPEED_SENSOR_TIMER_FREQ_HZ 1000000u
#define CYCLEIQ_SPEED_SENSOR_TIMER_CLK_HZ (SYSTEM_CORE_CLOCK / 2u)
#define CYCLEIQ_SPEED_SENSOR_TIMER_PRESCALER                                 \
  ((CYCLEIQ_SPEED_SENSOR_TIMER_CLK_HZ / CYCLEIQ_SPEED_SENSOR_TIMER_FREQ_HZ) - \
   1u)
#define CYCLEIQ_SPEED_SENSOR_TIMER_PERIOD                                     \
  ((CYCLEIQ_SPEED_SENSOR_TIMER_FREQ_HZ /                                    \
    CYCLEIQ_SPEED_SENSOR_SAMPLE_RATE_HZ) -                                  \
   1u)
#define CYCLEIQ_SPEED_SENSOR_PUBLISH_PERIOD_MS 10u
#define CYCLEIQ_MOTOR_TEMP_SAMPLE_GAP_MS 3u
#define CYCLEIQ_MOTOR_TEMP_SAMPLE_GAP_TICKS                                  \
  ((CYCLEIQ_SPEED_SENSOR_SAMPLE_RATE_HZ * CYCLEIQ_MOTOR_TEMP_SAMPLE_GAP_MS + \
    999u) /                                                                  \
   1000u)
#define CYCLEIQ_MOTOR_SPEED_STALE_MS 3000u
#define CYCLEIQ_MOTOR_SPEED_STALE_TICKS                                      \
  ((CYCLEIQ_SPEED_SENSOR_SAMPLE_RATE_HZ * CYCLEIQ_MOTOR_SPEED_STALE_MS) /    \
   1000u)
#define CYCLEIQ_MOTOR_SPEED_INPUT_THRESHOLD 50
#define CYCLEIQ_SPEED_SENSOR_LOW_DEBOUNCE_SAMPLES 2u
#define CYCLEIQ_MAX_PLAUSIBLE_SPEED_KPH 80.0f
#define CYCLEIQ_KPH_TO_MPS (1.0f / 3.6f)

static THD_FUNCTION(cycleiq_sensor_publish_thread, arg);
static THD_WORKING_AREA(cycleiq_sensor_publish_thread_wa, 256);

static volatile bool stop_now = true;
static volatile bool is_running;
static volatile bool sample_timer_running;

static volatile float motor_rpm;
static volatile int8_t motor_temperature_c;
static volatile bool speed_sensor_low;
static volatile bool speed_sensor_low_debounced;
static volatile uint8_t speed_sensor_low_samples;
static volatile uint32_t sample_ticks;
static volatile uint32_t last_raw_low_ticks;
static volatile uint32_t last_raw_pulse_ticks;
static volatile uint32_t last_accepted_pulse_ticks;
static volatile uint32_t last_temperature_ticks;
static volatile uint32_t pulse_interval_ticks;
static volatile bool pulse_interval_pending;
static volatile uint32_t raw_falling_edges;
static volatile uint32_t debounced_falling_edges;
static volatile uint32_t accepted_pulses;
static volatile uint32_t rejected_too_fast_pulses;
static volatile uint32_t last_raw_pulse_interval_ticks;
static volatile uint32_t last_accepted_pulse_interval_ticks;
static volatile uint32_t min_raw_pulse_interval_ticks;
static volatile uint16_t last_speed_adc_value;

static void speed_sensor_sample_timer_start(void);
static void speed_sensor_sample_timer_stop(void);
static void speed_sensor_sample_timer_isr(void);
static void cycleiq_sensor_publish_update_speed(void);
static void cycleiq_sensor_publish_update_temperature(void);
static void terminal_cycleiq_speed_diag(int argc, const char **argv);

static uint32_t elapsed_ticks(uint32_t now, uint32_t then) {
  return now - then;
}

static uint32_t min_pulse_ticks_for_max_speed(void) {
  float wheel_diameter_m = cycleiq_config.wheel_diameter_m;
  if (!isfinite(wheel_diameter_m) || wheel_diameter_m <= 0.0f) {
    wheel_diameter_m = 0.3f;
  }

  float max_speed_mps = CYCLEIQ_MAX_PLAUSIBLE_SPEED_KPH * CYCLEIQ_KPH_TO_MPS;
  float min_ticks = (wheel_diameter_m * (float)M_PI *
                     (float)CYCLEIQ_SPEED_SENSOR_SAMPLE_RATE_HZ) /
                    max_speed_mps;
  if (!isfinite(min_ticks) ||
      min_ticks < (float)CYCLEIQ_SPEED_SENSOR_LOW_DEBOUNCE_SAMPLES) {
    return CYCLEIQ_SPEED_SENSOR_LOW_DEBOUNCE_SAMPLES;
  }

  return (uint32_t)(min_ticks + 0.999f);
}

void cycleiq_sensors_start(void) {
  stop_now = false;
  is_running = false;
  motor_rpm = 0.0f;
  motor_temperature_c = 0;
  speed_sensor_low =
      ADC_Value[ADC_IND_TEMP_MOTOR] < CYCLEIQ_MOTOR_SPEED_INPUT_THRESHOLD;
  speed_sensor_low_debounced = speed_sensor_low;
  speed_sensor_low_samples =
      speed_sensor_low ? CYCLEIQ_SPEED_SENSOR_LOW_DEBOUNCE_SAMPLES : 0;
  sample_ticks = 0;
  last_raw_low_ticks = 0;
  last_raw_pulse_ticks = 0;
  last_accepted_pulse_ticks = 0;
  last_temperature_ticks = 0;
  pulse_interval_ticks = 0;
  pulse_interval_pending = false;
  raw_falling_edges = 0;
  debounced_falling_edges = 0;
  accepted_pulses = 0;
  rejected_too_fast_pulses = 0;
  last_raw_pulse_interval_ticks = 0;
  last_accepted_pulse_interval_ticks = 0;
  min_raw_pulse_interval_ticks = 0;
  last_speed_adc_value = ADC_Value[ADC_IND_TEMP_MOTOR];

  terminal_register_command_callback(
      "cycleiq_speed_diag",
      "Print cycleIQ wheel speed sensor diagnostics",
      0,
      terminal_cycleiq_speed_diag);

  speed_sensor_sample_timer_start();
  chThdCreateStatic(cycleiq_sensor_publish_thread_wa,
                    sizeof(cycleiq_sensor_publish_thread_wa), NORMALPRIO,
                    cycleiq_sensor_publish_thread, NULL);
}

void cycleiq_sensors_stop(void) {
  stop_now = true;
  speed_sensor_sample_timer_stop();
  while (is_running) {
    chThdSleepMilliseconds(1);
  }
  terminal_unregister_callback(terminal_cycleiq_speed_diag);
}

static void speed_sensor_sample_timer_start(void) {
  if (sample_timer_running) {
    return;
  }

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
  TIM_DeInit(TIM6);

  TIM_TimeBaseInitTypeDef timer_config;
  timer_config.TIM_Period = CYCLEIQ_SPEED_SENSOR_TIMER_PERIOD;
  timer_config.TIM_Prescaler = CYCLEIQ_SPEED_SENSOR_TIMER_PRESCALER;
  timer_config.TIM_ClockDivision = 0;
  timer_config.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM6, &timer_config);

  TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
  TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
  nvicEnableVector(TIM6_DAC_IRQn, 7);
  TIM_Cmd(TIM6, ENABLE);
  sample_timer_running = true;
}

static void speed_sensor_sample_timer_stop(void) {
  if (!sample_timer_running) {
    return;
  }

  TIM_ITConfig(TIM6, TIM_IT_Update, DISABLE);
  TIM_Cmd(TIM6, DISABLE);
  nvicDisableVector(TIM6_DAC_IRQn);
  TIM_DeInit(TIM6);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, DISABLE);
  sample_timer_running = false;
}

static void speed_sensor_sample_timer_isr(void) {
  uint32_t now = ++sample_ticks;

  uint16_t adc_value = ADC_Value[ADC_IND_TEMP_MOTOR];
  bool state = adc_value < CYCLEIQ_MOTOR_SPEED_INPUT_THRESHOLD;
  last_speed_adc_value = adc_value;
  if (state) {
    last_raw_low_ticks = now;
  }

  if (state && !speed_sensor_low) {
    raw_falling_edges++;

    uint32_t previous_raw_pulse_ticks = last_raw_pulse_ticks;
    last_raw_pulse_ticks = now;

    if (previous_raw_pulse_ticks != 0) {
      uint32_t pulse_ticks = elapsed_ticks(now, previous_raw_pulse_ticks);
      if (pulse_ticks > 0) {
        last_raw_pulse_interval_ticks = pulse_ticks;
        if (min_raw_pulse_interval_ticks == 0 ||
            pulse_ticks < min_raw_pulse_interval_ticks) {
          min_raw_pulse_interval_ticks = pulse_ticks;
        }
      }
    }
  }

  speed_sensor_low = state;

  if (state) {
    if (speed_sensor_low_samples < UINT8_MAX) {
      speed_sensor_low_samples++;
    }
  } else {
    speed_sensor_low_samples = 0;
  }

  bool debounced_low =
      speed_sensor_low_samples >= CYCLEIQ_SPEED_SENSOR_LOW_DEBOUNCE_SAMPLES;
  if (debounced_low && !speed_sensor_low_debounced) {
    debounced_falling_edges++;

    uint32_t previous_accepted_pulse_ticks = last_accepted_pulse_ticks;
    bool accept_pulse = true;
    uint32_t accepted_interval_ticks = 0;

    if (previous_accepted_pulse_ticks != 0) {
      accepted_interval_ticks =
          elapsed_ticks(now, previous_accepted_pulse_ticks);
      if (accepted_interval_ticks < min_pulse_ticks_for_max_speed()) {
        accept_pulse = false;
        rejected_too_fast_pulses++;
      }
    }

    if (accept_pulse) {
      last_accepted_pulse_ticks = now;
      accepted_pulses++;

      if (accepted_interval_ticks > 0) {
        last_accepted_pulse_interval_ticks = accepted_interval_ticks;
        pulse_interval_ticks = accepted_interval_ticks;
        pulse_interval_pending = true;
      }
    }
  }

  speed_sensor_low_debounced = debounced_low;
}

static void cycleiq_sensor_publish_update_speed(void) {
  uint32_t pulse_ticks = 0;
  bool have_pulse = false;

  chSysLock();
  if (pulse_interval_pending) {
    pulse_ticks = pulse_interval_ticks;
    pulse_interval_pending = false;
    have_pulse = true;
  }
  chSysUnlock();

  if (have_pulse) {
    // One magnet pulse is one wheel rotation.
    float current_rpm =
        (60.0f * (float)CYCLEIQ_SPEED_SENSOR_SAMPLE_RATE_HZ) /
        (float)pulse_ticks;
    UTILS_LP_MOVING_AVG_APPROX(motor_rpm, current_rpm, 5);
  }
}

static void cycleiq_sensor_publish_update_temperature(void) {
  uint32_t now = sample_ticks;

  /*
   * The wheel hall switch and motor NTC share one ADC input. A low ADC value
   * means the magnet is pulling the line to 0 V; only sample the NTC divider
   * after the line has been high for a short gap.
   */
  if (speed_sensor_low ||
      (last_raw_low_ticks != 0 &&
       elapsed_ticks(now, last_raw_low_ticks) <
           CYCLEIQ_MOTOR_TEMP_SAMPLE_GAP_TICKS) ||
      elapsed_ticks(now, last_temperature_ticks) <
          CYCLEIQ_MOTOR_TEMP_SAMPLE_GAP_TICKS) {
    return;
  }

  last_temperature_ticks = now;

  int temperature_c = (int)NTC_TEMP_MOTOR(3435.0f);
  utils_truncate_number_int(&temperature_c, -128, 127);
  int8_t current_temperature_c = (int8_t)temperature_c;
  UTILS_LP_MOVING_AVG_APPROX(motor_temperature_c, current_temperature_c, 10);
}

static THD_FUNCTION(cycleiq_sensor_publish_thread, arg) {
  (void)arg;
  chRegSetThreadName("CYCLEIQ_SENSORS");
  is_running = true;

  for (;;) {
    if (stop_now) {
      is_running = false;
      return;
    }

    chThdSleepMilliseconds(CYCLEIQ_SPEED_SENSOR_PUBLISH_PERIOD_MS);

    cycleiq_sensor_publish_update_speed();
    cycleiq_sensor_publish_update_temperature();

    uint32_t now = sample_ticks;
    if (last_accepted_pulse_ticks == 0 ||
        elapsed_ticks(now, last_accepted_pulse_ticks) >
            CYCLEIQ_MOTOR_SPEED_STALE_TICKS) {
      motor_rpm = 0.0f;
    }

    cycleiq_data_motor_sensor_update(motor_rpm, motor_temperature_c);

    timeout_reset();
  }
}

CH_IRQ_HANDLER(TIM6_IRQHandler) {
  CH_IRQ_PROLOGUE();
  if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET) {
    TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
    speed_sensor_sample_timer_isr();
  }
  CH_IRQ_EPILOGUE();
}

static void terminal_cycleiq_speed_diag(int argc, const char **argv) {
  (void)argc;
  (void)argv;

  uint32_t raw_edges_snapshot;
  uint32_t debounced_edges_snapshot;
  uint32_t accepted_pulses_snapshot;
  uint32_t rejected_too_fast_snapshot;
  uint32_t last_raw_interval_snapshot;
  uint32_t last_accepted_interval_snapshot;
  uint32_t min_raw_interval_snapshot;
  uint16_t adc_value_snapshot;
  float motor_rpm_snapshot;

  chSysLock();
  raw_edges_snapshot = raw_falling_edges;
  debounced_edges_snapshot = debounced_falling_edges;
  accepted_pulses_snapshot = accepted_pulses;
  rejected_too_fast_snapshot = rejected_too_fast_pulses;
  last_raw_interval_snapshot = last_raw_pulse_interval_ticks;
  last_accepted_interval_snapshot = last_accepted_pulse_interval_ticks;
  min_raw_interval_snapshot = min_raw_pulse_interval_ticks;
  adc_value_snapshot = last_speed_adc_value;
  motor_rpm_snapshot = motor_rpm;
  chSysUnlock();

  float speed_kph = (motor_rpm_snapshot * cycleiq_config.wheel_diameter_m *
                     (float)M_PI * 3.6f) /
                    60.0f;
  if (!isfinite(speed_kph) || speed_kph < 0.0f) {
    speed_kph = 0.0f;
  }

  commands_printf("cycleIQ speed diagnostics:");
  commands_printf("  raw_edges: %u", raw_edges_snapshot);
  commands_printf("  debounced_edges: %u", debounced_edges_snapshot);
  commands_printf("  accepted_pulses: %u", accepted_pulses_snapshot);
  commands_printf("  rejected_too_fast: %u", rejected_too_fast_snapshot);
  commands_printf("  last_raw_interval_ticks: %u", last_raw_interval_snapshot);
  commands_printf("  last_accepted_interval_ticks: %u",
                  last_accepted_interval_snapshot);
  commands_printf("  min_raw_interval_ticks: %u", min_raw_interval_snapshot);
  commands_printf("  adc_value: %u", adc_value_snapshot);
  commands_printf("  motor_rpm: %.2f", (double)motor_rpm_snapshot);
  commands_printf("  speed_kph: %.2f", (double)speed_kph);
  commands_printf("  min_accepted_interval_ticks: %u",
                  min_pulse_ticks_for_max_speed());
}
