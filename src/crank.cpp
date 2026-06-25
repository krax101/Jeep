#include "crank.h"
#include "config.h"
#include "scheduler.h"
#include <Arduino.h>

// ---- Module-level pointer to shared state -------------------
static CrankState* s_cs = nullptr;

// Number of teeth we average for RPM filtering
#define RPM_SMOOTH_TEETH  6

// Gap detection: ratio of gap period to a normal tooth period.
//   36-1 (1 missing tooth): gap = 2 × normal → threshold 1.6 (between 1× and 2×).
//   44-2 (2 missing teeth): gap = 3 × normal → threshold 2.2 (between 2× and 3×).
#if TRIGGER_MODE == TRIGGER_MODE_RENIX_44
#define MISSING_TOOTH_RATIO  2.2f
#else
#define MISSING_TOOTH_RATIO  1.6f
#endif

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

    uint32_t now    = micros_isr();
    uint32_t period = now - cs.last_tooth_us;
    cs.last_tooth_us = now;

    // --- Missing-tooth detection (36-1: gap period ≈ 2 normal tooth widths) ---
    // When the gap is detected, the CURRENT tooth is tooth 0 — the first real
    // tooth after the absent slot.  Set tooth_count = 0 directly; the old 0xFF
    // sentinel was off-by-one and made every subsequent angle wrong by 10°.
    // Also: do NOT update tooth_period_us for the gap event — storing the 2×
    // period would double the detection threshold and break the next gap detection.
    bool gap = (cs.tooth_period_us > 0 &&
                period > (uint32_t)(cs.tooth_period_us * MISSING_TOOTH_RATIO));

    if (gap) {
        cs.tooth_count = 0;
        if (cs.synced) {
            cs.revolution ^= 1;      // toggle 720° half on every gap
        } else {
            cs.synced     = true;    // first gap: acquire sync
            cs.revolution = 0;
        }
    } else {
        cs.tooth_count++;
        if (cs.tooth_count >= (TRIGGER_WHEEL_TEETH - TRIGGER_WHEEL_MISSING))
            cs.tooth_count = 0;     // safety wrap (should not occur with good signal)
    }

    // Update RPM buffer and reference period only for normal (non-gap) teeth.
    // Including the gap's 2× period would contaminate the RPM average and
    // corrupt the threshold used by the next gap detection.
    if (!gap) {
        cs.tooth_times[cs.tooth_head % 40] = period;
        cs.tooth_head++;
        cs.tooth_period_us = period;
    }

    // Crank angle in 360° × 10 space, referenced so that TDC cyl-1 = 0°.
    // Tooth 0 (first tooth after gap) is TRIGGER_SYNC_ANGLE_BTDC before TDC.
    //
    // Use divide-last (tooth_count × 3600 / teeth) rather than multiplying by
    // TRIGGER_DEGREES_PER_TOOTH (integer truncated).  For 44-2, the truncated
    // 8°/tooth constant accumulates 8° of error by the last tooth of the
    // revolution; the divide-last form has at most 1 unit (0.1°) non-
    // accumulating error at any tooth — same as the 36-1 case (exact: 10°).
    int32_t angle_x10 = (int32_t)cs.tooth_count * 3600 / TRIGGER_WHEEL_TEETH;
    int32_t offset_x10 = (360 - TRIGGER_SYNC_ANGLE_BTDC) * 10;
    angle_x10 = (angle_x10 + offset_x10) % 3600;
    if (angle_x10 < 0) angle_x10 += 3600;
    cs.crank_angle_x10 = (uint16_t)angle_x10;

    // 720° angle (valid after both 360° sync and cam sync)
    cs.angle_720_x10 = cs.crank_angle_x10 + (cs.revolution ? 3600 : 0);

    // Filtered RPM from average of recent tooth periods
    {
        uint32_t sum = 0;
        uint8_t  n   = 0;
        for (uint8_t i = 0; i < RPM_SMOOTH_TEETH && i < (uint8_t)cs.tooth_head; i++) {
            sum += cs.tooth_times[(cs.tooth_head - 1 - i) % 40];
            n++;
        }
        if (n > 0 && sum > 0) {
            uint32_t avg_period = sum / n;
            cs.rpm_filtered = (uint32_t)(60000000UL /
                              ((uint32_t)avg_period * TRIGGER_WHEEL_TEETH));
        }
    }

    // Fire any angle-based engine events (injection opens, ignition dwell/spark).
    // This is the primary call site — events must be triggered here, from the ISR,
    // to achieve per-tooth (~10°) timing accuracy.  The main-loop sched_tick(0,0)
    // call only handles the RPM=0 / stalled-engine case.
    if (cs.synced)
        sched_tick(cs.angle_720_x10, cs.tooth_period_us);
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
    // Set last_tooth_us to now BEFORE attaching the interrupt so the first
    // tooth period reflects real elapsed time, not seconds of boot uptime
    // from the epoch-0 that memset left.  A boot-time period (~millions of µs)
    // stored in tooth_period_us would set the gap-detection threshold to
    // ~1.6 × boot_time, blocking gap detection for many teeth afterward.
    cs.last_tooth_us = micros();
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
    // One tooth spans 3600/TRIGGER_WHEEL_TEETH units in ×10-degree space.
    // Using the same divide-last formula as the ISR avoids truncation of
    // TRIGGER_DEGREES_PER_TOOTH (8 instead of 8.18 for the 44-2 wheel).
    const uint32_t tooth_span_x10 = 3600u / TRIGGER_WHEEL_TEETH;
    uint32_t offset_x10 = (elapsed * tooth_span_x10) / cs.tooth_period_us;
    if (offset_x10 > tooth_span_x10) offset_x10 = tooth_span_x10;
    uint32_t angle = (uint32_t)cs.crank_angle_x10 + offset_x10;
    return (uint16_t)(angle % 3600);
}

uint16_t crank_angle720_now_x10(const CrankState& cs) {
    if (!cs.synced) return 0;
    uint32_t elapsed = micros() - cs.last_tooth_us;
    uint32_t offset_x10 = 0;
    if (cs.tooth_period_us > 0) {
        const uint32_t tooth_span_x10 = 3600u / TRIGGER_WHEEL_TEETH;
        offset_x10 = (elapsed * tooth_span_x10) / cs.tooth_period_us;
        if (offset_x10 > tooth_span_x10) offset_x10 = tooth_span_x10;
    }
    uint32_t angle = (uint32_t)cs.angle_720_x10 + offset_x10;
    return (uint16_t)(angle % 7200);
}
