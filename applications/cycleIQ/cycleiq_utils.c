#include "cycleiq_utils.h"
#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include <stdlib.h>

void ring_buffer_init(ring_buffer_t *rb, size_t size)
{
  rb->buffer = (uint8_t *)malloc(size);
  if (rb->buffer == NULL)
  {
    // Handle memory allocation failure
    rb->size = 0;
    rb->head = 0;
    rb->tail = 0;
  }
  else
  {
    rb->size = size;
    rb->head = 0;
    rb->tail = 0;
  }
}

void ring_buffer_push(ring_buffer_t *rb, uint8_t data)
{
  if (!ring_buffer_is_full(rb))
  {
    rb->buffer[rb->head] = data;
    rb->head = (rb->head + 1) % rb->size;
  }
}

uint8_t ring_buffer_pop(ring_buffer_t *rb)
{
  if (ring_buffer_is_empty(rb))
  {
    return 0; // or handle underflow
  }

  uint8_t data = rb->buffer[rb->tail];
  rb->tail = (rb->tail + 1) % rb->size;
  return data;
}

bool ring_buffer_is_empty(ring_buffer_t *rb)
{
  return rb->head == rb->tail;
}

bool ring_buffer_is_full(ring_buffer_t *rb)
{
  return (rb->head + 1) % rb->size == rb->tail;
}

size_t ring_buffer_length(ring_buffer_t *rb)
{
  if (rb->head >= rb->tail)
  {
    return rb->head - rb->tail;
  }
  else
  {
    return rb->size - (rb->tail - rb->head);
  }
}

void ring_buffer_clear(ring_buffer_t *rb)
{
  rb->head = 0;
  rb->tail = 0;
}

void ring_buffer_free(ring_buffer_t *rb)
{
  if (rb->buffer != NULL)
  {
    free(rb->buffer);
    rb->buffer = NULL;
  }
  rb->size = 0;
  rb->head = 0;
  rb->tail = 0;
}