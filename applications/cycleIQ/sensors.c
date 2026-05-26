#include "sensors.h"

#include "ch.h"
#include "data.h"
#include "hal.h"
#include "hw.h"
#include "isr_vector_table.h"
#include "mc_interface.h"
#include "stm32f4xx_conf.h"
#include "timeout.h"
#include "utils_math.h"

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

static THD_FUNCTION(cycleiq_sensor_publish_thread, arg);
static THD_WORKING_AREA(cycleiq_sensor_publish_thread_wa, 256);

static volatile bool stop_now = true;
static volatile bool is_running;
static volatile bool sample_timer_running;

static volatile float motor_rpm;
static volatile int8_t motor_temperature_c;
static volatile bool speed_sensor_low;
static volatile uint32_t sample_ticks;
static volatile uint32_t last_pulse_ticks;
static volatile uint32_t last_temperature_ticks;
static volatile uint32_t pulse_interval_ticks;
static volatile bool pulse_interval_pending;

static void speed_sensor_sample_timer_start(void);
static void speed_sensor_sample_timer_stop(void);
static void speed_sensor_sample_timer_isr(void);
static void cycleiq_sensor_publish_update_speed(void);
static void cycleiq_sensor_publish_update_temperature(void);

static uint32_t elapsed_ticks(uint32_t now, uint32_t then) {
  return now - then;
}

void cycleiq_sensors_start(void) {
  stop_now = false;
  is_running = false;
  motor_rpm = 0.0f;
  motor_temperature_c = 0;
  speed_sensor_low =
      ADC_Value[ADC_IND_TEMP_MOTOR] < CYCLEIQ_MOTOR_SPEED_INPUT_THRESHOLD;
  sample_ticks = 0;
  last_pulse_ticks = 0;
  last_temperature_ticks = 0;
  pulse_interval_ticks = 0;
  pulse_interval_pending = false;

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

  bool state =
      ADC_Value[ADC_IND_TEMP_MOTOR] < CYCLEIQ_MOTOR_SPEED_INPUT_THRESHOLD;
  if (state && !speed_sensor_low) {
    uint32_t previous_pulse_ticks = last_pulse_ticks;
    last_pulse_ticks = now;

    if (previous_pulse_ticks != 0) {
      uint32_t pulse_ticks = elapsed_ticks(now, previous_pulse_ticks);
      if (pulse_ticks > 0) {
        pulse_interval_ticks = pulse_ticks;
        pulse_interval_pending = true;
      }
    }
  }

  speed_sensor_low = state;
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
      (last_pulse_ticks != 0 &&
       elapsed_ticks(now, last_pulse_ticks) < CYCLEIQ_MOTOR_TEMP_SAMPLE_GAP_TICKS) ||
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
    if (last_pulse_ticks == 0 ||
        elapsed_ticks(now, last_pulse_ticks) > CYCLEIQ_MOTOR_SPEED_STALE_TICKS) {
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
