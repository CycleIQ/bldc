#ifndef CYCLEIQ_WALK_H
#define CYCLEIQ_WALK_H

#include <stdbool.h>

void cycleiq_walk_init(void);
void cycleiq_walk_set_enabled(bool enabled);
void cycleiq_walk_loop(void);
bool cycleiq_walk_is_active(void);

#endif
