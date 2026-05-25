#include "pas.h"

#include "app.h"

#include "ch.h"
#include "comm_can.h"
#include "hal.h"
#include "hw.h"
#include "isr_vector_table.h"
#include "mc_interface.h"
#include "stm32f4xx_conf.h"
#include "timeout.h"
#include "utils_math.h"
#include <commands.h>
#include <math.h>

#define TORQUE_SENSOR_SAMPLES 10
#define PAS_SAMPLE_RATE_HZ 10000U
#define PAS_FILTER_TIME_US 1500U
#define PAS_FILTER_SAMPLES                                                     \
  ((PAS_SAMPLE_RATE_HZ * PAS_FILTER_TIME_US + 999999U) / 1000000U)
#define PAS_MIN_TRANSITION_TIME_US PAS_FILTER_TIME_US
#define PAS_TIMER_FREQ_HZ 1000000U
#define PAS_TIMER_CLK_HZ (SYSTEM_CORE_CLOCK / 2U)
#define PAS_TIMER_PRESCALER ((PAS_TIMER_CLK_HZ / PAS_TIMER_FREQ_HZ) - 1U)
#define PAS_TIMER_PERIOD ((PAS_TIMER_FREQ_HZ / PAS_SAMPLE_RATE_HZ) - 1U)

// Configuration structure for PAS
static volatile cycleiq_pas_config config;

static volatile float max_pulse_period_ms =
    0.0; // Maximum pulse period in milliseconds after which PAS stops
static volatile float min_pulse_period_ms =
    0.0; // Minimum pulse period in milliseconds to filter out noise
static volatile uint8_t min_correct_direction =
    0; // Minimum correct direction events to consider pedaling

// Variables to track PAS state
static volatile int last_state = 0;
static volatile systime_t last_state_change = 0;
static volatile systime_t last_pulse_time = 0;
static volatile uint8_t correct_direction_counter =
    0; // Counter for correct direction events
static volatile systime_t max_pulse_period_ticks = 0;
static volatile systime_t min_pulse_period_ticks = 0;
static volatile systime_t min_transition_period_ticks = 0;

// Software-filtered PAS input state. Updated from the TIM7 ISR.
static volatile uint8_t filter_counter_a = 0;
static volatile uint8_t filter_counter_b = 0;
static volatile int filtered_state = 0;
static volatile bool sample_timer_running = false;

// Human readable PAS state
static volatile bool is_pedaling = false;
static volatile float pedal_rpm = 0.0;
static volatile float last_pedal_rpm = 0.0; // Last pedal RPM for filtering

// Constants for torque sensor voltage thresholds
static float TORQUE_VOLTAGE_MIN = 1.5f; // Starting voltage for torque sensor
const float TORQUE_VOLTAGE_MAX = 2.4f;  // Maximum voltage for torque sensor
static volatile float torque_sensor_voltage =
    0.0f; // Current voltage from the torque sensor (used for filtering)

#ifdef CYCLEIQ_HAS_2_WIRE_PAS
static const int pas_lookup[] = {0, 3, 1, 2};
#endif

static int pas_update_state(void) {
#ifdef CYCLEIQ_HAS_2_WIRE_PAS
  int a = palReadPad(PAS_WIRE_A_BANK, PAS_WIRE_A_PIN);
  int b = palReadPad(PAS_WIRE_B_BANK, PAS_WIRE_B_PIN);
#else
  int a = 0; // No second wire for single-wire PAS
  int b = palReadPad(PAS_BANK, PAS_PIN);
#endif

  int state = (a << 1) | b; // Combine the two states into a single value

  return state;
}

static uint8_t pas_update_filter_counter(uint8_t counter, bool raw_high) {
  if (raw_high) {
    if (counter < PAS_FILTER_SAMPLES) {
      counter++;
    }
  } else if (counter > 0) {
    counter--;
  }

  return counter;
}

static void pas_handle_filtered_state_change(int state, systime_t current_time) {
  if (config.magnets == 0 || min_transition_period_ticks == 0) {
    return;
  }

  int prev_state = last_state;

  if (state == prev_state)
    return;

  if (last_pulse_time != 0 &&
      current_time - last_pulse_time < min_transition_period_ticks)
    return;

#ifdef CYCLEIQ_HAS_2_WIRE_PAS
  int diff = (pas_lookup[state] - pas_lookup[prev_state] + 4) %
             4;    // Calculate the difference in state (0-3)
  if (diff == 2) // Invalid state (may happen due to noise)
    return;      // Ignore the state change, but don't reset the counter

  last_state = state;
  last_pulse_time = current_time;

  if (diff == 3) { // Backwards
    correct_direction_counter =
        0; // Reset counter if the direction is not correct
    return;
  }

  if (correct_direction_counter < min_correct_direction)
    correct_direction_counter++;
#else
  last_state = state;
  last_pulse_time = current_time;

  if (correct_direction_counter < min_correct_direction)
    correct_direction_counter++;
#endif

  if (state == 0) { // Only update once per pulse
    systime_t previous_state_change = last_state_change;

    if (previous_state_change != 0) {
      systime_t pulse_period = current_time - previous_state_change;
      if (pulse_period < min_pulse_period_ticks) {
        return;
      }

      if (pulse_period > 0) {
        last_state_change = current_time;
        float current_rpm =
            (60.0f * (float)CH_CFG_ST_FREQUENCY) /
            ((float)config.magnets * (float)pulse_period);

        if (last_pedal_rpm <= 0.0f) {
          last_pedal_rpm = current_rpm;
        } else {
          UTILS_LP_MOVING_AVG_APPROX(last_pedal_rpm, current_rpm, 5);
        }

        pedal_rpm = last_pedal_rpm;
      }
    } else {
      last_state_change = current_time;
    }
  }
}

static void pas_sample_timer_start(void) {
  if (sample_timer_running) {
    return;
  }

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
  TIM_DeInit(TIM7);

  TIM_TimeBaseInitTypeDef timer_config;
  timer_config.TIM_Period = PAS_TIMER_PERIOD;
  timer_config.TIM_Prescaler = PAS_TIMER_PRESCALER;
  timer_config.TIM_ClockDivision = 0;
  timer_config.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM7, &timer_config);

  TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
  TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
  nvicEnableVector(TIM7_IRQn, 7);
  TIM_Cmd(TIM7, ENABLE);
  sample_timer_running = true;
}

static void pas_sample_timer_stop(void) {
  if (!sample_timer_running) {
    return;
  }

  TIM_ITConfig(TIM7, TIM_IT_Update, DISABLE);
  TIM_Cmd(TIM7, DISABLE);
  nvicDisableVector(TIM7_IRQn);
  TIM_DeInit(TIM7);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, DISABLE);
  sample_timer_running = false;
}

static void pas_sample_timer_isr(void) {
  int raw_state = pas_update_state();

#ifdef CYCLEIQ_HAS_2_WIRE_PAS
  filter_counter_a =
      pas_update_filter_counter(filter_counter_a, (raw_state & 2) != 0);
#else
  filter_counter_a = 0;
#endif
  filter_counter_b =
      pas_update_filter_counter(filter_counter_b, (raw_state & 1) != 0);

  int new_state = filtered_state;
  if (filter_counter_a >= PAS_FILTER_SAMPLES) {
    new_state |= 2;
  } else if (filter_counter_a == 0) {
    new_state &= ~2;
  }

  if (filter_counter_b >= PAS_FILTER_SAMPLES) {
    new_state |= 1;
  } else if (filter_counter_b == 0) {
    new_state &= ~1;
  }

  if (new_state != filtered_state) {
    filtered_state = new_state;
    pas_handle_filtered_state_change(new_state, chVTGetSystemTimeX());
  }
}

void cycleiq_pas_init(void) {
#ifdef CYCLEIQ_HAS_2_WIRE_PAS
  palSetPadMode(PAS_WIRE_A_BANK, PAS_WIRE_A_PIN, PAL_MODE_INPUT);
  palSetPadMode(PAS_WIRE_B_BANK, PAS_WIRE_B_PIN, PAL_MODE_INPUT);
#else
  palSetPadMode(PAS_BANK, PAS_PIN, PAL_MODE_INPUT);
#endif

  filtered_state = pas_update_state();
  last_state = filtered_state;
  filter_counter_a = (filtered_state & 2) != 0 ? PAS_FILTER_SAMPLES : 0;
  filter_counter_b = (filtered_state & 1) != 0 ? PAS_FILTER_SAMPLES : 0;
  last_state_change = 0;
  last_pulse_time = 0;
  correct_direction_counter = 0;
  pedal_rpm = 0.0f;
  last_pedal_rpm = 0.0f;

  // Zero out the torque sensor voltage across 10 samples
  torque_sensor_voltage = 0.0f;
  for (int i = 0; i < TORQUE_SENSOR_SAMPLES; i++) {
    torque_sensor_voltage += ADC_VOLTS(TS_INDEX);
    chThdSleepMilliseconds(10);
  }
  torque_sensor_voltage /= TORQUE_SENSOR_SAMPLES; // Average the readings
  torque_sensor_voltage *=
      1.03f; // Apply slight offset to account for ADC inaccuracies (3%)

  TORQUE_VOLTAGE_MIN = torque_sensor_voltage;
  torque_sensor_voltage = 0.0f; // Reset the filtered voltage
}

void cycleiq_pas_deinit(void) {
  pas_sample_timer_stop();

#ifdef CYCLEIQ_HAS_2_WIRE_PAS
  palSetPadMode(PAS_WIRE_A_BANK, PAS_WIRE_A_PIN, PAL_MODE_INPUT);
  palSetPadMode(PAS_WIRE_B_BANK, PAS_WIRE_B_PIN, PAL_MODE_INPUT);
#else
  palSetPadMode(PAS_BANK, PAS_PIN, PAL_MODE_INPUT);
#endif
}

void cycleiq_pas_configure(cycleiq_pas_config *conf) {
  pas_sample_timer_stop();
  config = *conf;

  if (config.magnets == 0 || config.pedal_rpm_start <= 0.0f ||
      config.pedal_rpm_max <= 0.0f) {
    max_pulse_period_ticks = 0;
    min_pulse_period_ticks = 0;
    min_transition_period_ticks = 0;
    return;
  }

#ifdef CYCLEIQ_HAS_2_WIRE_PAS
  min_correct_direction = config.magnets * 4 / 2;
  max_pulse_period_ms =
      1000.0 / ((config.pedal_rpm_start / 60.0) *
                config.magnets); // Calculate the maximum pulse period based on
                                 // pedal RPM and magnets
  min_pulse_period_ms =
      1000.0 / ((config.pedal_rpm_max / 60.0) *
                config.magnets); // Calculate the minimum pulse period based on
                                 // pedal RPM and magnets
#else
  min_correct_direction =
      config.magnets / 2; // For single-wire PAS, set the minimum correct
                          // direction events to half the magnets
  max_pulse_period_ms =
      1000.0 / ((config.pedal_rpm_start / 60.0) *
                config.magnets); // Calculate the maximum pulse period based on
                                 // pedal RPM and magnets
  min_pulse_period_ms =
      1000.0 / ((config.pedal_rpm_max / 60.0) *
                config.magnets); // Calculate the minimum pulse period based on
                                 // pedal RPM and magnets
#endif

  uint32_t max_pulse_period_us = (uint32_t)(max_pulse_period_ms * 1000.0f);
  uint32_t min_pulse_period_us = (uint32_t)(min_pulse_period_ms * 1000.0f);

  max_pulse_period_ticks = US2ST(max_pulse_period_us);
  min_pulse_period_ticks = US2ST(min_pulse_period_us);
  min_transition_period_ticks = US2ST(PAS_MIN_TRANSITION_TIME_US);

  filtered_state = pas_update_state();
  last_state = filtered_state;
  filter_counter_a = (filtered_state & 2) != 0 ? PAS_FILTER_SAMPLES : 0;
  filter_counter_b = (filtered_state & 1) != 0 ? PAS_FILTER_SAMPLES : 0;
  last_state_change = 0;
  last_pulse_time = 0;
  correct_direction_counter = 0;
  pedal_rpm = 0.0f;
  last_pedal_rpm = 0.0f;

  pas_sample_timer_start();
}

void cycleiq_pas_isr_handler(void) {
  /*
   * PAS decoding is sampled and debounced from TIM7. The board-level EXTI
   * handler still calls this hook on some builds, so keep it as a cheap
   * compatibility hook rather than doing duplicate raw-edge processing.
   */
}

void cycleiq_pas_loop(void) {
  systime_t current_time = chVTGetSystemTimeX();

  float ts_voltage = ADC_VOLTS(TS_INDEX); // Read the torque sensor voltage
  UTILS_LP_MOVING_AVG_APPROX(torque_sensor_voltage, ts_voltage,
                             50); // Apply a moving average filter

  if (last_state_change == 0 ||
      current_time - last_state_change > max_pulse_period_ticks) {
    // If the time since the last state change exceeds the maximum pulse period,
    // stop pedaling
    is_pedaling = false;
    pedal_rpm = 0.0f;
    last_pedal_rpm = 0.0f;
    correct_direction_counter = 0; // Reset the counter

    return;
  }

  if (correct_direction_counter < min_correct_direction) {
    is_pedaling = false;
    pedal_rpm = 0.0f;
  } else {
    is_pedaling = true;
  }
}

CH_IRQ_HANDLER(TIM7_IRQHandler) {
  CH_IRQ_PROLOGUE();
  if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET) {
    TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
    pas_sample_timer_isr();
  }
  CH_IRQ_EPILOGUE();
}

bool cycleiq_pas_is_pedaling(void) {
  bool res;
  chSysLock();
  res = is_pedaling;
  chSysUnlock();
  return res;
}

float cycleiq_pas_get_pedal_rpm(void) {
  float res;
  chSysLock();
  res = pedal_rpm;
  chSysUnlock();
  return res;
}
float cycleiq_ts_get_voltage(void) {
  float res;
  chSysLock();
  res = torque_sensor_voltage;
  chSysUnlock();
  return res;
}
bool cycleiq_ts_is_active(void) {
  bool res;
  chSysLock();
  res = (torque_sensor_voltage >= TORQUE_VOLTAGE_MIN &&
         torque_sensor_voltage <= TORQUE_VOLTAGE_MAX);
  chSysUnlock();
  return res;
}
float cycleiq_ts_get_percentage(void) {
  float res;
  chSysLock();
  res = utils_map(torque_sensor_voltage, TORQUE_VOLTAGE_MIN, TORQUE_VOLTAGE_MAX,
                  0.0f, 1.0f);
  utils_truncate_number(&res, 0.0f, 1.5f); // Max 150%
  chSysUnlock();
  return res;
}
