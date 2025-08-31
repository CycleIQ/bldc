#ifndef CYCLEIQ_PAS_H_
#define CYCLEIQ_PAS_H_

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "mc_interface.h"
#include "timeout.h"
#include "utils_math.h"
#include "comm_can.h"
#include "hw.h"
#include <math.h>

typedef struct
{
  float pedal_rpm_start;         // RPM at which PAS starts
  uint8_t min_correct_direction; // Minimum correct direction events to consider pedaling
  uint8_t magnets;               // Number of magnets on the PAS sensor
} cycleiq_pas_config;

void cycleiq_pas_init(void);
void cycleiq_pas_deinit(void);
void cycleiq_pas_configure(cycleiq_pas_config *conf);
void cycleiq_pas_loop(void);
void cycleiq_pas_isr_handler(void);

bool cycleiq_pas_is_pedaling(void);
float cycleiq_pas_get_pedal_rpm(void);
uint32_t cycleiq_pas_get_interrupt_counter(void);

float cycleiq_ts_get_voltage(void);
float cycleiq_ts_get_percentage(void);
bool cycleiq_ts_is_active(void);

#endif