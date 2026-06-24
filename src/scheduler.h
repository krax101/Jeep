#pragma once
#include "types.h"

// Maximum number of simultaneously pending engine events.
// Sequential mode uses 24 slots (6 INJ_OPEN + 6 INJ_CLOSE + 6 IGN_DWELL + 6 IGN_FIRE).
// 32 gives headroom for mode transitions without silent event drops.
#define SCHED_MAX_EVENTS  32

// Initialise the scheduler and hardware timer infrastructure.
void sched_init();

// Add an angle-triggered event.  If an event with the same type+channel
// already exists it is replaced.
// angle_720_x10: 0–7199 (for sequential) or 0–3599 (for batch)
// delay_us: additional delay fired via one-shot timer after angle trigger
void sched_add(EventType type, uint8_t channel,
               uint16_t angle_720_x10, uint32_t delay_us);

// Remove all events of a given type+channel.
void sched_remove(EventType type, uint8_t channel);

// Called from the CPS ISR on every tooth (or on each main-loop tick if
// no tooth ISR is available).  Fires any events whose angle has passed.
void sched_tick(uint16_t current_angle_720_x10, uint32_t tooth_period_us);

// Direct-fire helpers used by the one-shot timer callbacks.
void sched_fire_inj_open(uint8_t ch);
void sched_fire_inj_close(uint8_t ch);
void sched_fire_ign_dwell(uint8_t ch);
void sched_fire_ign_spark(uint8_t ch);
