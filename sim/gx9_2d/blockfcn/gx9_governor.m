function thr_eff = gx9_governor(thr_cmd, w_e, P)
%GX9_GOVERNOR  Droop governor on the Baja spec engine.
%
%   Baja SAE engines keep the factory governor, so torque is progressively
%   pulled out above the governed speed rather than being cut abruptly.
%   Full command below w_gov, linearly to zero over w_droop above it.
%#codegen
    f       = 1 - (w_e - P.eng.w_gov) / P.eng.w_droop;
    f       = min(max(f, 0), 1);
    thr_eff = min(max(thr_cmd, 0), 1) * f;
end
