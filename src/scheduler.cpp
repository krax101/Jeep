#include "scheduler.h"
#include "config.h"
#include <Arduino.h>
#include <TeensyTimerTool.h>

using namespace TeensyTimerTool;

// ---- Static Event Table -------------------------------------
static EngineEvent s_events[SCHED_MAX_EVENTS];
static uint8_t     s_event_count = 0;

// ---- One-Shot Timers (one per injector + 1 ignition) --------
// TeensyTimerTool GPT timers give 1-μs resolution on Teensy 4.1
static OneShotTimer s_inj_timers[ENGINE_CYLINDERS];
static OneShotTimer s_ign_timer;

// ---- Timer Callbacks ----------------------------------------

static void inj_close_cb_0() { sched_fire_inj_close(0); }
static void inj_close_cb_1() { sched_fire_inj_close(1); }
static void inj_close_cb_2() { sched_fire_inj_close(2); }
static void inj_close_cb_3() { sched_fire_inj_close(3); }
static void inj_close_cb_4() { sched_fire_inj_close(4); }
static void inj_close_cb_5() { sched_fire_inj_close(5); }
static void ign_spark_cb()   { sched_fire_ign_spark(0); }

static void (*s_inj_cbs[ENGINE_CYLINDERS])() = {
    inj_close_cb_0, inj_close_cb_1, inj_close_cb_2,
    inj_close_cb_3, inj_close_cb_4, inj_close_cb_5
};

// Pin map: injector 0..5
static const uint8_t INJ_PINS[ENGINE_CYLINDERS] = {
    PIN_INJ_1, PIN_INJ_2, PIN_INJ_3,
    PIN_INJ_4, PIN_INJ_5, PIN_INJ_6
};

// ---- Public Init --------------------------------------------

void sched_init() {
    memset(s_events, 0, sizeof(s_events));
    s_event_count = 0;

    for (uint8_t i = 0; i < ENGINE_CYLINDERS; i++) {
        s_inj_timers[i].begin(s_inj_cbs[i]);
        pinMode(INJ_PINS[i], OUTPUT);
        digitalWriteFast(INJ_PINS[i], LOW);  // Injectors OFF by default
    }
    s_ign_timer.begin(ign_spark_cb);
    pinMode(PIN_IGN_COIL, OUTPUT);
    digitalWriteFast(PIN_IGN_COIL, LOW);
}

// ---- Event Management ---------------------------------------

void sched_add(EventType type, uint8_t channel,
               uint16_t angle_720_x10, uint32_t delay_us) {
    // Replace existing event of same type+channel if present
    for (uint8_t i = 0; i < s_event_count; i++) {
        if (s_events[i].type == type && s_events[i].channel == channel) {
            s_events[i].angle_720_x10 = angle_720_x10;
            s_events[i].delay_us      = delay_us;
            s_events[i].active        = true;
            return;
        }
    }
    if (s_event_count < SCHED_MAX_EVENTS) {
        s_events[s_event_count++] = { type, channel, angle_720_x10, delay_us, true };
    }
}

void sched_remove(EventType type, uint8_t channel) {
    for (uint8_t i = 0; i < s_event_count; i++) {
        if (s_events[i].type == type && s_events[i].channel == channel) {
            s_events[i].active = false;
        }
    }
}

// ---- Tick: Called Each Tooth --------------------------------

// prev_angle and current_angle in 720° × 10 space
static uint16_t s_prev_angle = 0;

static inline bool angle_passed(uint16_t prev, uint16_t cur, uint16_t target) {
    // Handle wrap-around of the 720° window
    if (cur >= prev) {
        return (target >= prev && target < cur);
    } else {
        // Wrapped around
        return (target >= prev || target < cur);
    }
}

void sched_tick(uint16_t current_angle_720_x10, uint32_t tooth_period_us) {
    for (uint8_t i = 0; i < s_event_count; i++) {
        if (!s_events[i].active) continue;

        if (angle_passed(s_prev_angle, current_angle_720_x10,
                         s_events[i].angle_720_x10)) {
            // Compute how many μs since we crossed the trigger angle.
            // Wrap-aware: if the event was at e.g. 7150 and we just passed 0°,
            // a naive subtraction underflows uint16 giving a massive past_us.
            uint16_t ev_angle  = s_events[i].angle_720_x10;
            uint16_t angle_diff;
            if (current_angle_720_x10 >= ev_angle)
                angle_diff = current_angle_720_x10 - ev_angle;
            else
                angle_diff = (uint16_t)(7200u - ev_angle + current_angle_720_x10);

            uint32_t past_us = 0;
            if (tooth_period_us > 0)
                past_us = ((uint32_t)angle_diff * tooth_period_us) /
                          (TRIGGER_DEGREES_PER_TOOTH * 10);

            uint32_t fire_delay = (s_events[i].delay_us > past_us)
                                  ? (s_events[i].delay_us - past_us) : 0;

            switch (s_events[i].type) {
                case EventType::INJ_OPEN:
                    if (fire_delay > 0)
                        s_inj_timers[s_events[i].channel].trigger(fire_delay);
                    else
                        sched_fire_inj_open(s_events[i].channel);
                    break;
                case EventType::INJ_CLOSE:
                    s_inj_timers[s_events[i].channel].trigger(
                        s_events[i].delay_us > 0 ? s_events[i].delay_us : 1);
                    break;
                case EventType::IGN_DWELL:
                    if (fire_delay > 0)
                        s_ign_timer.trigger(fire_delay);
                    else
                        sched_fire_ign_dwell(s_events[i].channel);
                    break;
                case EventType::IGN_FIRE:
                    s_ign_timer.trigger(
                        s_events[i].delay_us > 0 ? s_events[i].delay_us : 1);
                    break;
            }
        }
    }
    s_prev_angle = current_angle_720_x10;
}

// ---- Direct Fire Callbacks ----------------------------------

void sched_fire_inj_open(uint8_t ch) {
    if (ch < ENGINE_CYLINDERS)
        digitalWriteFast(INJ_PINS[ch], HIGH);
}

void sched_fire_inj_close(uint8_t ch) {
    if (ch < ENGINE_CYLINDERS)
        digitalWriteFast(INJ_PINS[ch], LOW);
}

void sched_fire_ign_dwell(uint8_t ch) {
    (void)ch;
    digitalWriteFast(PIN_IGN_COIL, HIGH);
    digitalWriteFast(PIN_TACH_OUT, HIGH);  // Tach pulse HIGH at dwell start
}

void sched_fire_ign_spark(uint8_t ch) {
    (void)ch;
    digitalWriteFast(PIN_IGN_COIL, LOW);   // Drop coil → spark
    digitalWriteFast(PIN_TACH_OUT, LOW);   // Tach pulse LOW at fire = one pulse per cylinder
}
