#ifndef CYCLEIQ_UTILS_H
#define CYCLEIQ_UTILS_H

#include "app.h"
#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"

typedef struct
{
  uint8_t *buffer; // Pointer to the buffer
  size_t size;     // Size of the buffer
  size_t head;     // Index of the head of the buffer
  size_t tail;     // Index of the tail of the buffer
} ring_buffer_t;

void ring_buffer_init(ring_buffer_t *rb, size_t size);
void ring_buffer_push(ring_buffer_t *rb, uint8_t data);
uint8_t ring_buffer_pop(ring_buffer_t *rb);
bool ring_buffer_is_empty(ring_buffer_t *rb);
bool ring_buffer_is_full(ring_buffer_t *rb);
size_t ring_buffer_length(ring_buffer_t *rb);
void ring_buffer_clear(ring_buffer_t *rb);
void ring_buffer_free(ring_buffer_t *rb);

#endif