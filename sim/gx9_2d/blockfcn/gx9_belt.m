function [T_pri, slip_w] = gx9_belt(w_e, w_sec, ratio, T_cap, engage, P)
%GX9_BELT  Slipping-belt coupling between the primary and the secondary.
%
%   The belt is not a rigid ratio constraint.  Below engagement the primary
%   sheaves do not grip at all and the engine free-revs; just above it the
%   clamp force - and therefore the torque capacity - is small, so the belt
%   slips heavily and the car launches on a slipping belt exactly as it does
%   in reality.  Once clamped, slip collapses to a fraction of a rad/s and
%   the coupling behaves as a stiff ratio constraint.
%
%   Slip is measured on the primary side:  slip = w_e - ratio*w_sec.
%   Transmitted torque is a stiff linear function of slip, saturated at the
%   friction capacity computed in gx9_cvt_forces.
%#codegen

    slip_w = w_e - ratio * w_sec;

    T_lin = P.cvt.k_slip * slip_w;
    T_pri = min(max(T_lin, -T_cap), T_cap) * engage;
end
