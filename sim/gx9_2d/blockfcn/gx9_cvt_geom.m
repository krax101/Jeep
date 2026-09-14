function [rs, ratio, theta_p, theta_s, lever, xp, xs] = gx9_cvt_geom(rp, P)
%GX9_CVT_GEOM  Belt-length-constrained sheave geometry of the GX9.
%
%   This is the "2-D" core of the CVT: the belt runs on two pitch circles
%   separated by a fixed centre distance, so the radial position on one
%   sheave and the radial position on the other are not independent - they
%   are locked together by the (inextensible) belt length, and each is in
%   turn locked to its sheave's axial position by the groove half-angle.
%
%   Open-belt geometry, signed beta so that it works either side of 1:1:
%       beta    = asin((rp - rs)/C)
%       theta_p = pi + 2*beta      (wrap angle on the primary)
%       theta_s = pi - 2*beta      (wrap angle on the secondary)
%       Lbelt   = pi*(rp+rs) + 2*beta*(rp-rs) + 2*C*cos(beta)
%
%   Differentiating Lbelt = const gives the exact shift lever
%       drs/drp = -theta_p/theta_s
%   which gx9_cvt_forces uses to reduce the two axial sheave forces to a
%   single net shift force.  rs itself is pre-solved in gx9_params and
%   looked up here so that this block stays iteration-free.
%#codegen

    rs = local_interp(P.cvt.rp_brk, P.cvt.rs_tab, rp);

    d    = rp - rs;
    s    = d / P.cvt.C;
    s    = min(max(s, -0.999), 0.999);
    beta = asin(s);

    theta_p = pi + 2*beta;
    theta_s = pi - 2*beta;
    lever   = theta_p / theta_s;

    ratio = rs / max(rp, 1e-6);          % w_primary / w_secondary

    xp = P.cvt.tw * (rp - P.cvt.rp_min); % axial travel of the movable faces
    xs = P.cvt.tw * (rs - P.cvt.rs_min);
end

function y = local_interp(xb, yb, x)
%LOCAL_INTERP  Clamped linear interpolation on a monotonically rising grid.
%#codegen
    n = numel(xb);
    if x <= xb(1)
        y = yb(1);
        return
    elseif x >= xb(n)
        y = yb(n);
        return
    end
    % Uniform grid -> direct index, no search.
    dx = (xb(n) - xb(1)) / (n - 1);
    i  = floor((x - xb(1)) / dx) + 1;
    i  = min(max(i, 1), n-1);
    t  = (x - xb(i)) / dx;
    y  = yb(i) + t * (yb(i+1) - yb(i));
end
