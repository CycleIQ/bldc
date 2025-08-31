#include "pas.h"

#include "app.h"

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "mc_interface.h"
#include "timeout.h"
#include "utils_math.h"
#include "comm_can.h"
#include "hw.h"
#include <math.h>
#include <commands.h>

// Configuration structure for PAS
static volatile cycleiq_pas_config config;
static volatile float max_pulse_period_ms = 0.0; // Maximum pulse period in milliseconds after which PAS stops

// Variables to track PAS state
static volatile systime_t last_pulse_time = 0;
static volatile float pedal_rpm = 0.0;
static volatile bool is_pedaling = false;
static volatile uint8_t correct_direction_counter = 0; // Counter for correct direction events
static volatile float last_pedal_rpm = 0.0;            // Last pedal RPM for filtering
static volatile uint32_t interrupt_counter = 0;        // Counter for interrupts to handle debouncing

static volatile int last_state = 0;
static volatile systime_t last_state_change = 0;

// Constants for torque sensor voltage thresholds
const float TORQUE_VOLTAGE_MIN = 1.55f;             // Starting voltage for torque sensor
const float TORQUE_VOLTAGE_MAX = 3.0f;              // Maximum voltage for torque sensor
static volatile float torque_sensor_voltage = 0.0f; // Current voltage from the torque sensor (used for filtering)

#ifdef CYCLEIQ_HAS_2_WIRE_PAS
static const int pas_lookup[] = {0, 3, 1, 2};
#endif

void cycleiq_pas_init(void)
{
#ifdef CYCLEIQ_HAS_2_WIRE_PAS
  palSetPadMode(PAS_WIRE_A_BANK, PAS_WIRE_A_PIN, PAL_MODE_INPUT);
  palSetPadMode(PAS_WIRE_B_BANK, PAS_WIRE_B_PIN, PAL_MODE_INPUT);

  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource10); // Configure EXTI for PAS wire A
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource11); // Configure EXTI for PAS wire B

  // Configure EXTI Line for PAS wire A
  EXTI_InitTypeDef EXTI_InitStructureA;
  EXTI_InitStructureA.EXTI_Line = EXTI_Line10;
  EXTI_InitStructureA.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructureA.EXTI_Trigger = EXTI_Trigger_Rising_Falling; // Trigger on both rising and falling edges
  EXTI_InitStructureA.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructureA);

  // Configure EXTI Line for PAS wire B
  EXTI_InitTypeDef EXTI_InitStructureB;
  EXTI_InitStructureB.EXTI_Line = EXTI_Line11;
  EXTI_InitStructureB.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructureB.EXTI_Trigger = EXTI_Trigger_Rising_Falling; // Trigger on both rising and falling edges
  EXTI_InitStructureB.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructureB);

  // Enable the NVIC for the EXTI lines
  nvicEnableVector(EXTI15_10_IRQn, 1); // Enable EXTI line 10 and 11 interrupts
#else
  palSetPadMode(PAS_BANK, PAS_PIN, PAL_MODE_INPUT_PULLUP); // Set the PAS pin as input with pull-up

  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource11); // Configure EXTI for PAS pin

  // Configure EXTI Line for PAS pin
  EXTI_InitTypeDef EXTI_InitStructure;
  EXTI_InitStructure.EXTI_Line = EXTI_Line11; // Use line
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // Trigger on falling edge
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;               // Enable the line
  EXTI_Init(&EXTI_InitStructure);
  // Enable the NVIC for the EXTI line
  nvicEnableVector(EXTI15_10_IRQn, 1); // Enable EXTI line 10 interrupt
#endif
}

void cycleiq_pas_deinit(void)
{
#ifdef CYCLEIQ_HAS_2_WIRE_PAS
  nvicDisableVector(EXTI15_10_IRQn); // Disable EXTI line 10 and 11 interrupts
  palSetPadMode(PAS_WIRE_A_BANK, PAS_WIRE_A_PIN, PAL_MODE_INPUT);
  palSetPadMode(PAS_WIRE_B_BANK, PAS_WIRE_B_PIN, PAL_MODE_INPUT);
#else
  nvicDisableVector(EXTI15_10_IRQn); // Disable EXTI line 10 interrupt
  palSetPadMode(PAS_BANK, PAS_PIN, PAL_MODE_INPUT);
#endif
}

void cycleiq_pas_configure(cycleiq_pas_config *conf)
{
  config = *conf;

#ifdef CYCLEIQ_HAS_2_WIRE_PAS
  config.min_correct_direction = config.magnets * 4 / 2;
  max_pulse_period_ms = 1000.0 / ((config.pedal_rpm_start / 60.0) * config.magnets); // Calculate the maximum pulse period based on pedal RPM and magnets
#else
  config.min_correct_direction = config.magnets / 2;                                 // For single-wire PAS, set the minimum correct direction events to half the magnets
  max_pulse_period_ms = 1000.0 / ((config.pedal_rpm_start / 60.0) * config.magnets); // Calculate the maximum pulse period based on pedal RPM and magnets
#endif
}

static int pas_update_state(void)
{
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

void cycleiq_pas_isr_handler(void)
{
  int state = pas_update_state(); // Get the current state of the PAS sensor
  int prev_state = last_state;    // Store the previous state for comparison

  last_state = state; // Update the last state

  if (state == prev_state)
    return;

#ifdef CYCLEIQ_HAS_2_WIRE_PAS
  int diff = (pas_lookup[state] - pas_lookup[prev_state] + 4) % 4; // Calculate the difference in state (0-3)
  if (diff != 1)
  {
    correct_direction_counter = 0; // Reset counter if the direction is not correct
    return;
  }

  if (correct_direction_counter < config.min_correct_direction)
    correct_direction_counter++;
#else
  if (correct_direction_counter < config.min_correct_direction)
    correct_direction_counter++;
#endif

  if (state == 0) // Only update once per pulse
    last_state_change = chVTGetSystemTimeX();
}

void cycleiq_pas_loop(void)
{
  systime_t current_time = chVTGetSystemTimeX();

  float ts_voltage = ADC_VOLTS(TS_INDEX);                            // Read the torque sensor voltage
  UTILS_LP_MOVING_AVG_APPROX(torque_sensor_voltage, ts_voltage, 50); // Apply a moving average filter

  if (current_time - last_state_change > MS2ST(max_pulse_period_ms))
  {
    // If the time since the last state change exceeds the maximum pulse period, stop pedaling
    is_pedaling = false;
    pedal_rpm = 0.0f;
    correct_direction_counter = 0; // Reset the counter

    return;
  }

  if (correct_direction_counter < config.min_correct_direction)
  {
    is_pedaling = false;
    pedal_rpm = 0.0f;
  }
  else
  {
    is_pedaling = true;
  }
}

bool cycleiq_pas_is_pedaling(void)
{
#ifdef CYCLEIQ_HAS_2_WIRE_PAS
  bool res;
  chSysLock();
  res = is_pedaling;
  chSysUnlock();
  return res;
#else
  return cycleiq_ts_is_active();
#endif
}
float cycleiq_pas_get_pedal_rpm(void)
{
  float res;
  chSysLock();
  res = pedal_rpm;
  chSysUnlock();
  return res;
}
uint32_t cycleiq_pas_get_interrupt_counter(void)
{
  uint32_t res;
  chSysLock();
  res = interrupt_counter;
  chSysUnlock();
  return res;
}
float cycleiq_ts_get_voltage(void)
{
  float res;
  chSysLock();
  res = torque_sensor_voltage;
  chSysUnlock();
  return res;
}
bool cycleiq_ts_is_active(void)
{
  bool res;
  chSysLock();
  res = (torque_sensor_voltage >= TORQUE_VOLTAGE_MIN && torque_sensor_voltage <= TORQUE_VOLTAGE_MAX);
  chSysUnlock();
  return res;
}
float cycleiq_ts_get_percentage(void)
{
  float res;
  chSysLock();
  res = utils_map(torque_sensor_voltage, TORQUE_VOLTAGE_MIN, TORQUE_VOLTAGE_MAX, 0.0f, 1.0f);
  utils_truncate_number(&res, 0.0f, 1.0f);
  chSysUnlock();
  return res;
}
