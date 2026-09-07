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
#define TORQUE_SENSOR_FAST_FILTER_SAMPLES 6
#define TORQUE_SENSOR_FAST_BLEND 0.20f
#define TORQUE_SENSOR_ACTIVE_THRESHOLD_PERCENT 0.03f
#define TORQUE_SENSOR_RELEASE_THRESHOLD_PERCENT 0.015f
#define TORQUE_SENSOR_LOW_TORQUE_ANGLE_NUMERATOR 1U
#define TORQUE_SENSOR_LOW_TORQUE_ANGLE_DENOMINATOR 3U
#define TORQUE_SENSOR_MAX_HALF_REVOLUTION_SAMPLES 18U
#define TORQUE_PULSE_QUEUE_SIZE 16U
#define TORQUE_PULSE_QUEUE_MASK (TORQUE_PULSE_QUEUE_SIZE - 1U)
#define PAS_STOP_TIMEOUT_MIN_MS 60U
#define PAS_STOP_TIMEOUT_MAX_MS 250U
#define PAS_STOP_TIMEOUT_PERIODS 4U
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
static volatile systime_t last_forward_transition_time = 0;
static volatile systime_t last_forward_transition_period_ticks = 0;

// Constants for torque sensor voltage thresholds
static float TORQUE_VOLTAGE_MIN = 1.5f; // Starting voltage for torque sensor
const float TORQUE_VOLTAGE_MAX = 2.4f;  // Maximum voltage for torque sensor
static volatile float torque_sensor_voltage =
    0.0f; // Fast-filtered torque sensor voltage.
static volatile float torque_demand_percentage = 0.0f;
static volatile bool torque_sensor_valid = false;
static volatile bool torque_release_fast = false;

/*
 * PAS events define equal crank-angle bins. The ISR only queues a raw ADC
 * snapshot; all filtering and floating-point work stays in thread context.
 */
static volatile uint16_t torque_pulse_adc_queue[TORQUE_PULSE_QUEUE_SIZE];
static volatile uint8_t torque_pulse_queue_head = 0;
static volatile uint8_t torque_pulse_queue_tail = 0;
static volatile bool torque_pulse_queue_overflow = false;

static float torque_angle_bins[TORQUE_SENSOR_MAX_HALF_REVOLUTION_SAMPLES];
static uint8_t torque_angle_window_samples = 0;
static uint8_t torque_angle_write_index = 0;
static uint8_t torque_angle_valid_samples = 0;
static uint8_t torque_low_pulse_count = 0;
static float torque_angle_sum = 0.0f;

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

static void torque_pulse_queue_clear(void) {
  chSysLock();
  torque_pulse_queue_head = 0;
  torque_pulse_queue_tail = 0;
  torque_pulse_queue_overflow = false;
  chSysUnlock();
}

static void torque_pulse_queue_push(uint16_t adc_value) {
  uint8_t head = torque_pulse_queue_head;
  uint8_t next_head = (head + 1U) & TORQUE_PULSE_QUEUE_MASK;

  if (next_head == torque_pulse_queue_tail) {
    torque_pulse_queue_overflow = true;
    return;
  }

  torque_pulse_adc_queue[head] = adc_value;
  torque_pulse_queue_head = next_head;
}

static bool torque_pulse_queue_pop(uint16_t *adc_value) {
  bool has_sample = false;

  chSysLock();
  uint8_t tail = torque_pulse_queue_tail;
  if (tail != torque_pulse_queue_head) {
    *adc_value = torque_pulse_adc_queue[tail];
    torque_pulse_queue_tail = (tail + 1U) & TORQUE_PULSE_QUEUE_MASK;
    has_sample = true;
  }
  chSysUnlock();

  return has_sample;
}

static bool torque_pulse_queue_take_overflow(void) {
  bool overflow;

  chSysLock();
  overflow = torque_pulse_queue_overflow;
  torque_pulse_queue_overflow = false;
  chSysUnlock();

  return overflow;
}

static void torque_angle_history_clear(void) {
  torque_angle_write_index = 0;
  torque_angle_valid_samples = 0;
  torque_low_pulse_count = 0;
  torque_angle_sum = 0.0f;
}

static float torque_percentage_from_voltage(float voltage) {
  float percentage =
      utils_map(voltage, TORQUE_VOLTAGE_MIN, TORQUE_VOLTAGE_MAX, 0.0f, 1.0f);
  utils_truncate_number(&percentage, 0.0f, 1.5f);
  return percentage;
}

static uint8_t torque_low_pulse_limit(void) {
  uint8_t limit = (uint8_t)((config.magnets *
                             TORQUE_SENSOR_LOW_TORQUE_ANGLE_NUMERATOR +
                             TORQUE_SENSOR_LOW_TORQUE_ANGLE_DENOMINATOR - 1U) /
                            TORQUE_SENSOR_LOW_TORQUE_ANGLE_DENOMINATOR);
  return limit > 0 ? limit : 1;
}

static void torque_record_angle_sample(uint16_t adc_value) {
  if (torque_angle_window_samples == 0) {
    return;
  }

  float voltage = ((float)adc_value / 4096.0f) * V_REG;
  float sample_percentage = torque_percentage_from_voltage(voltage);

  if (sample_percentage <= TORQUE_SENSOR_RELEASE_THRESHOLD_PERCENT) {
    if (torque_low_pulse_count < torque_low_pulse_limit()) {
      torque_low_pulse_count++;
    }
  } else {
    torque_low_pulse_count = 0;
  }

  if (torque_angle_valid_samples == torque_angle_window_samples) {
    torque_angle_sum -= torque_angle_bins[torque_angle_write_index];
  } else {
    torque_angle_valid_samples++;
  }

  torque_angle_bins[torque_angle_write_index] = sample_percentage;
  torque_angle_sum += sample_percentage;
  torque_angle_write_index++;
  if (torque_angle_write_index >= torque_angle_window_samples) {
    torque_angle_write_index = 0;
  }
}

static bool pas_forward_motion_timed_out(systime_t current_time) {
  systime_t last_transition;
  systime_t transition_period;

  chSysLock();
  last_transition = last_forward_transition_time;
  transition_period = last_forward_transition_period_ticks;
  chSysUnlock();

  if (last_transition == 0) {
    return false;
  }

  systime_t timeout_ticks = transition_period * PAS_STOP_TIMEOUT_PERIODS;
  systime_t min_timeout_ticks = MS2ST(PAS_STOP_TIMEOUT_MIN_MS);
  systime_t max_timeout_ticks = MS2ST(PAS_STOP_TIMEOUT_MAX_MS);
  if (timeout_ticks < min_timeout_ticks) {
    timeout_ticks = min_timeout_ticks;
  } else if (timeout_ticks > max_timeout_ticks) {
    timeout_ticks = max_timeout_ticks;
  }

  return current_time - last_transition > timeout_ticks;
}

static void pas_handle_filtered_state_change(int state,
                                             systime_t current_time) {
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
             4;  // Calculate the difference in state (0-3)
  if (diff == 2) // Invalid state (may happen due to noise)
    return;      // Ignore the state change, but don't reset the counter

  last_state = state;
  last_pulse_time = current_time;

  if (diff == 3) { // Backwards
    correct_direction_counter =
        0; // Reset counter if the direction is not correct
    return;
  }

  systime_t previous_forward_transition = last_forward_transition_time;
  if (previous_forward_transition != 0) {
    systime_t transition_period = current_time - previous_forward_transition;
    last_forward_transition_period_ticks =
        transition_period <= max_pulse_period_ticks ? transition_period : 0;
  }
  last_forward_transition_time = current_time;

  if (correct_direction_counter < min_correct_direction)
    correct_direction_counter++;
#else
  last_state = state;
  last_pulse_time = current_time;

  systime_t previous_forward_transition = last_forward_transition_time;
  if (previous_forward_transition != 0) {
    systime_t transition_period = current_time - previous_forward_transition;
    last_forward_transition_period_ticks =
        transition_period <= max_pulse_period_ticks ? transition_period : 0;
  }
  last_forward_transition_time = current_time;

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
        torque_pulse_queue_push(ADC_Value[TS_INDEX]);
        float current_rpm = (60.0f * (float)CH_CFG_ST_FREQUENCY) /
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
      torque_pulse_queue_push(ADC_Value[TS_INDEX]);
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
  last_forward_transition_time = 0;
  last_forward_transition_period_ticks = 0;
  torque_pulse_queue_clear();
  torque_angle_history_clear();

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
  torque_sensor_voltage = TORQUE_VOLTAGE_MIN;
  torque_demand_percentage = 0.0f;
  torque_sensor_valid = true;
  torque_release_fast = false;
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
  min_correct_direction = config.magnets * 4 / 6;
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
      config.magnets / 6; // For single-wire PAS, set the minimum correct
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
  last_forward_transition_time = 0;
  last_forward_transition_period_ticks = 0;
  torque_pulse_queue_clear();
  torque_angle_history_clear();
  torque_angle_window_samples = config.magnets / 2U;
  if (torque_angle_window_samples > TORQUE_SENSOR_MAX_HALF_REVOLUTION_SAMPLES) {
    torque_angle_window_samples = TORQUE_SENSOR_MAX_HALF_REVOLUTION_SAMPLES;
  }
  torque_demand_percentage = 0.0f;
  torque_release_fast = false;

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
  if (isfinite(ts_voltage)) {
    UTILS_LP_MOVING_AVG_APPROX(torque_sensor_voltage, ts_voltage,
                               TORQUE_SENSOR_FAST_FILTER_SAMPLES);
    torque_sensor_valid = true;
  } else {
    torque_sensor_voltage = 0.0f;
    torque_sensor_valid = false;
  }

  if (torque_pulse_queue_take_overflow()) {
    torque_angle_history_clear();
  }

  uint16_t pulse_adc_value;
  while (torque_pulse_queue_pop(&pulse_adc_value)) {
    torque_record_angle_sample(pulse_adc_value);
  }

  float fast_percentage = torque_percentage_from_voltage(torque_sensor_voltage);
  float angle_percentage = fast_percentage;
  if (torque_angle_valid_samples > 0) {
    angle_percentage = torque_angle_sum / torque_angle_valid_samples;
  }

  float demand_percentage =
      angle_percentage + TORQUE_SENSOR_FAST_BLEND *
                             (fast_percentage - angle_percentage);
  utils_truncate_number(&demand_percentage, 0.0f, 1.5f);
  torque_demand_percentage = demand_percentage;

  torque_release_fast = !torque_sensor_valid ||
                        pas_forward_motion_timed_out(current_time) ||
                        torque_low_pulse_count >= torque_low_pulse_limit();
  if (torque_release_fast) {
    torque_demand_percentage = 0.0f;
    torque_angle_history_clear();
  }

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
  res = !torque_release_fast &&
        torque_demand_percentage >= TORQUE_SENSOR_ACTIVE_THRESHOLD_PERCENT;
  chSysUnlock();
  return res;
}
float cycleiq_ts_get_percentage(void) {
  float res;
  chSysLock();
  res = torque_demand_percentage;
  chSysUnlock();
  return res;
}

bool cycleiq_ts_should_release_fast(void) {
  bool res;
  chSysLock();
  res = torque_release_fast;
  chSysUnlock();
  return res;
}
