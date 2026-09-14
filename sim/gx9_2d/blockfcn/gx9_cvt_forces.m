function [Fp, Fs, F_shift, rp_dot, T_cap, engage] = ...
         gx9_cvt_forces(w_e, T_sec, rp, xp, xs, lever, theta_p, P)
%GX9_CVT_FORCES  Axial force balance across the GX9 primary and secondary.
%
%   PRIMARY (engine side).  Flyweights throw outwards against a ramp and push
%   the movable sheave face closed; the primary spring pushes it open:
%       F_fly  = m_w * w_e^2 * R_w * phi(xp)
%       F_p    = F_fly - (Fp0 + kp*xp)
%   phi(xp) is the combined ramp mechanical advantage and flyweight CG-radius
%   growth.  This is the curve to shape when moving the shift RPM.
%
%   SECONDARY (load side).  A compression spring plus a torque-sensing helix
%   hold the belt out at large radius; the helix force rises with transmitted
%   torque, which is what makes the CVT backshift under load:
%       F_s    = Fs0 + ks*xs + |T_sec| / (Rh * tan(theta_h))
%
%   NET SHIFT FORCE.  Virtual work on a belt-length-preserving shift
%   (dxp = tw*drp, dxs = tw*drs, drs/drp = -theta_p/theta_s) collapses the two
%   axial forces onto one axis:
%       F_shift = F_p - (theta_p/theta_s) * F_s
%   F_shift > 0 upshifts (primary radius grows), F_shift < 0 backshifts.
%   Shift rate is damping-limited, faster downwards than upwards.
%
%   T_cap is the V-belt friction capacity at the primary, from the capstan
%   relation with the wedge-amplified friction coefficient mu/sin(alpha).
%#codegen

    % ---- primary --------------------------------------------------
    xn    = xp / max(P.cvt.xp_max, 1e-9);
    phi   = local_interp(P.cvt.ramp_x, P.cvt.ramp_a, xn);
    F_fly = P.cvt.mw * w_e^2 * P.cvt.Rw * phi;
    Fp    = F_fly - (P.cvt.Fp0 + P.cvt.kp * xp);

    % ---- secondary ------------------------------------------------
    F_helix = abs(T_sec) / (P.cvt.Rh * tan(P.cvt.th));
    Fs      = P.cvt.Fs0 + P.cvt.ks * xs + F_helix;

    % ---- net shift force and rate ---------------------------------
    F_shift = Fp - lever * Fs;

    clamp  = max(Fp, 0);
    engage = min(max(clamp / P.cvt.F_eng_ref, 0), 1);

    if F_shift >= 0
        c = P.cvt.c_up;
    else
        c = P.cvt.c_dn;
    end
    % A belt that is not clamped cannot drive the sheaves anywhere.
    rp_dot = engage * F_shift / c;

    % ---- belt torque capacity at the primary ----------------------
    sa    = sin(P.cvt.alpha);
    mu_e  = P.cvt.mu_b / sa;                       % wedge-amplified friction
    wrap  = 1 - exp(-mu_e * max(theta_p, 0));
    T_cap = P.cvt.mu_b * clamp * rp / sa * wrap;
end

function y = local_interp(xb, yb, x)
%#codegen
    n = numel(xb);
    if x <= xb(1)
        y = yb(1); return
    elseif x >= xb(n)
        y = yb(n); return
    end
    i = 1;
    for k = 1:n-1
        if x >= xb(k) && x < xb(k+1)
            i = k;
        end
    end
    t = (x - xb(i)) / (xb(i+1) - xb(i));
    y = yb(i) + t * (yb(i+1) - yb(i));
end
