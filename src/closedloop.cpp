#include "closedloop.h"
#include "storage.h"

void cl_update(ECUState& state, const ECUConfig& cfg) {
    const SensorData& s = state.sensors;
    FuelState& fs       = state.fuel;

    // Conditions to allow closed-loop operation
    if (!cfg.cl_enabled)                                   return;
    if (state.fuel_mode != FuelMode::CLOSED)               return;
    if (s.clt_c < CL_MIN_CLT_C)                           return;
    if (s.rpm   < CL_MIN_RPM)                             return;
    if (state.engine_state == EngineState::CRANKING)       return;
    if (s.tps_pct > 85)                                   return;  // Skip CL at heavy load
    if (state.ae_amount > 0.0f && state.ae_teeth_left > 0) return;  // Skip during AE
    // Skip if O2 is inactive/faulted — a stuck-lean reading would otherwise
    // max out STFT at +25% and run the engine rich until LTFT adapts.
    if (state.diag.active[(uint8_t)FaultCode::O2_INACTIVE]) return;

    bool is_rich = (s.o2_mv > CL_STOICH_MV);

    // Step STFT toward stoich
    if (is_rich)
        fs.stft -= CL_STEP_PCT;
    else
        fs.stft += CL_STEP_PCT;

    // Clamp STFT
    if (fs.stft >  cfg.cl_max_stft) fs.stft =  cfg.cl_max_stft;
    if (fs.stft < -cfg.cl_max_stft) fs.stft = -cfg.cl_max_stft;

    // Bleed STFT into LTFT slowly when STFT is consistently non-zero
    const float LTFT_RATE = 0.05f;
    if (fs.stft > 2.0f)
        fs.ltft += LTFT_RATE;
    else if (fs.stft < -2.0f)
        fs.ltft -= LTFT_RATE;

    // Clamp LTFT
    if (fs.ltft >  cfg.cl_max_ltft) fs.ltft =  cfg.cl_max_ltft;
    if (fs.ltft < -cfg.cl_max_ltft) fs.ltft = -cfg.cl_max_ltft;
}

void cl_reset_stft(FuelState& fs) {
    fs.stft = 0.0f;
}

void cl_restore_ltft(FuelState& fs, const ECUConfig& cfg) {
    fs.ltft = cfg.saved_ltft;
}

void cl_save_ltft(ECUState& state, ECUConfig& cfg) {
    cfg.saved_ltft = state.fuel.ltft;
    storage_save(cfg);
}
