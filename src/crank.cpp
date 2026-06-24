#include "crank.h"
#include "config.h"
#include <Arduino.h>

// ---- Module-level pointer to shared state -------------------
static CrankState* s_cs = nullptr;

// Number of teeth we average for RPM filtering
#define RPM_SMOOTH_TEETH  6

// Factor by which tooth period must exceed average to be a gap
#define MISSING_TOOTH_RATIO  1.6f

// ---- ISR Helpers --------------------------------------------

static inline uint32_t micros_isr() {
    // ARM DWT cycle counter gives 1-ns resolution; convert to μs
    // On Teensy 4.1 (F_CPU = 600 MHz) each tick = 1000/600 ns
    // micros() is safe to call inside ISR on Teensy 4.x
    return micros();
}

void crank_isr_tooth() {
    if (!s_cs) return;
    CrankState& cs = *s_cs;

    uint32_t now = micros_isr();
    uint32_t period = now - cs.last_tooth_us;
    cs.last_tooth_us = now;

    // Store in circular buffer for RPM averaging
    cs.tooth_times[cs.tooth_head % 40] = period;
    cs.tooth_head++;

    // --- Missing tooth detection (36-1: gap ≈ 2 tooth widths) ---
    if (cs.synced && period > (uint32_t)(cs.tooth_period_us * MISSING_TOOTH_RATIO)) {
        // This is the gap tooth.  After the gap, tooth_count resets to 0
        // so the NEXT real tooth becomes tooth 0 (= TRIGGER_SYNC_ANGLE_BTDC).
        cs.tooth_count  = 0xFF;  // sentinel: increment to 0 on next tooth
        cs.revolution  ^= 1;    // toggle 720° half on every gap detection
        // Already synced — no action needed
    } else if (!cs.synced && period > (uint32_t)(cs.tooth_period_us * MISSING_TOOTH_RATIO) &&
               cs.tooth_period_us > 0) {
        // First gap seen — acquire sync
        cs.tooth_count = 0xFF;
        cs.synced      = true;
        cs.revolution  = 0;
    } else {
        // Normal tooth
        cs.tooth_count++;
        if (cs.tooth_count >= (TRIGGER_WHEEL_TEETH - TRIGGER_WHEEL_MISSING))
            cs.tooth_count = 0;  // safety wrap
    }

    cs.tooth_period_us = period;

    // Crank angle in 360° space × 10
    // After gap: tooth 0 = TRIGGER_SYNC_ANGLE_BTDC degrees before TDC #1
    // Map to 0° at TDC #1:  angle_360 = (tooth * 10 + (360 - BTDC_offset)) % 360
    int32_t raw = (int32_t)cs.tooth_count * TRIGGER_DEGREES_PER_TOOTH;
    raw = (raw + (360 - TRIGGER_SYNC_ANGLE_BTDC)) % 360;
    if (raw < 0) raw += 360;
    cs.crank_angle_x10 = (uint16_t)(raw * 10);

    // 720° angle
    cs.angle_720_x10 = cs.crank_angle_x10 + (cs.revolution ? 3600 : 0);

    // Filtered RPM from average of recent tooth periods
    uint32_t sum = 0;
    uint8_t n = 0;
    for (uint8_t i = 0; i < RPM_SMOOTH_TEETH && i < (uint8_t)cs.tooth_head; i++) {
        sum += cs.tooth_times[(cs.tooth_head - 1 - i) % 40];
        n++;
    }
    if (n > 0 && sum > 0) {
        // period_us per tooth → RPM:  rpm = 60e6 / (period_us × total_teeth)
        uint32_t avg_period = sum / n;
        cs.rpm_filtered = (uint32_t)(60000000UL /
                          ((uint32_t)avg_period * (TRIGGER_WHEEL_TEETH)));
    }
}

void crank_isr_cam() {
    if (!s_cs) return;
    s_cs->revolution  = 0;
    s_cs->cam_synced  = true;
    s_cs->last_cam_us = micros();
}

// ---- Public API ---------------------------------------------

void crank_init(CrankState& cs) {
    memset(&cs, 0, sizeof(cs));
    s_cs = &cs;

    pinMode(PIN_CPS_IN, INPUT);
    pinMode(PIN_CAM_IN, INPUT);

    attachInterrupt(digitalPinToInterrupt(PIN_CPS_IN), crank_isr_tooth, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_CAM_IN), crank_isr_cam,   RISING);
}

bool crank_is_synced(const CrankState& cs) {
    return cs.synced;
}

uint16_t crank_get_rpm(const CrankState& cs) {
    return (uint16_t)cs.rpm_filtered;
}

uint16_t crank_angle_now_x10(const CrankState& cs) {
    if (!cs.synced) return 0;
    uint32_t elapsed = micros() - cs.last_tooth_us;
    if (cs.tooth_period_us == 0) return cs.crank_angle_x10;
    // Interpolate within current tooth
    uint32_t offset_x10 = (elapsed * TRIGGER_DEGREES_PER_TOOTH * 10) / cs.tooth_period_us;
    if (offset_x10 > (uint32_t)(TRIGGER_DEGREES_PER_TOOTH * 10))
        offset_x10 = TRIGGER_DEGREES_PER_TOOTH * 10;
    uint32_t angle = (uint32_t)cs.crank_angle_x10 + offset_x10;
    return (uint16_t)(angle % 3600);
}

uint16_t crank_angle720_now_x10(const CrankState& cs) {
    if (!cs.synced) return 0;
    uint32_t elapsed = micros() - cs.last_tooth_us;
    uint32_t offset_x10 = 0;
    if (cs.tooth_period_us > 0) {
        offset_x10 = (elapsed * TRIGGER_DEGREES_PER_TOOTH * 10) / cs.tooth_period_us;
        if (offset_x10 > (uint32_t)(TRIGGER_DEGREES_PER_TOOTH * 10))
            offset_x10 = TRIGGER_DEGREES_PER_TOOTH * 10;
    }
    uint32_t angle = (uint32_t)cs.angle_720_x10 + offset_x10;
    return (uint16_t)(angle % 7200);
}
