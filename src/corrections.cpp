#include "corrections.h"
#include "tables.h"
#include <math.h>

float corrections_calc(const SensorData& s, ECUState& state, const ECUConfig& cfg) {
    float mult = 1.0f;

    // --- Warm-Up Enrichment (CLT) ---
    if (state.engine_state != EngineState::CRANKING) {
        float wue = table1d_lookup(cfg.clt_wue, (int16_t)s.clt_c);
        mult += wue / 100.0f;
    }

    // --- After-Start Enrichment ---
    if (state.ase_events_left > 0) {
        // 20% extra, decaying — corrections_injection_event decrements counter
        float ase_total = table1d_lookup(cfg.ase_table, (int16_t)s.clt_c);
        if (ase_total > 0.0f)       // guard: table returns 0 at CLT ≥ 100°C
            mult += 0.20f * ((float)state.ase_events_left / ase_total);
    }

    // --- IAT Correction ---
    float iat_corr = table1d_lookup(cfg.iat_corr, (int16_t)s.iat_c);
    mult += iat_corr / 100.0f;

    // --- Accel Enrichment ---
    if (state.ae_amount > 0.0f && state.ae_teeth_left > 0) {
        mult += state.ae_amount;
    }

    // --- Battery Voltage Correction on dead-time ---
    // Applied separately in fuel.cpp; here we return multiplier only.

    // --- Closed-Loop STFT ---
    mult *= (1.0f + state.fuel.stft / 100.0f);
    // LTFT
    mult *= (1.0f + state.fuel.ltft / 100.0f);

    // Clamp to sane range
    if (mult < 0.5f) mult = 0.5f;
    if (mult > 3.0f) mult = 3.0f;

    state.fuel.total_corr = mult;
    return mult;
}

void corrections_injection_event(ECUState& state, const ECUConfig& cfg) {
    if (state.ase_events_left > 0)
        state.ase_events_left--;
}

void corrections_accel_update(ECUState& state, float tps_pct, uint32_t now_ms,
                               const ECUConfig& cfg) {
    uint32_t dt_ms = now_ms - state.tps_last_ms;
    if (dt_ms < 10) return;

    float tps_rate = (tps_pct - state.tps_last) / ((float)dt_ms / 1000.0f);
    state.tps_last    = tps_pct;
    state.tps_last_ms = now_ms;

    if (tps_rate > cfg.ae_tps_threshold) {
        // Throttle opening rapidly — add AE
        float ae = (tps_rate - cfg.ae_tps_threshold) / 200.0f * cfg.ae_multiplier;
        if (ae < 0.0f) ae = 0.0f;
        if (ae > 2.0f) ae = 2.0f;
        state.ae_amount    = ae;
        state.ae_teeth_left = AE_DURATION_TEETH;
    } else if (state.ae_teeth_left > 0) {
        state.ae_teeth_left--;
        if (state.ae_teeth_left == 0) state.ae_amount = 0.0f;
    }
}
