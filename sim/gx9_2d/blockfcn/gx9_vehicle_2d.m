function [ax, F_trac, Nf, Nr, F_res, tract_lim] = gx9_vehicle_2d(F_dem, v, grade, P)
%GX9_VEHICLE_2D  Planar (longitudinal + pitch) vehicle model.
%
%   This is the 2-D half of the model.  The car is treated as a rigid body in
%   the x-z plane: longitudinal acceleration plus the pitch moment it creates
%   about the CG, which transfers normal load from the front axle onto the
%   driven rear axle.  That coupling is what makes a Baja launch prediction
%   meaningful - the rear grip that limits the launch is itself a function of
%   the acceleration being predicted.
%
%       Nf = [ m*g*cos(th)*b - m*ax*h - m*g*sin(th)*h ] / L
%       Nr = [ m*g*cos(th)*a + m*ax*h + m*g*sin(th)*h ] / L
%
%   Rather than iterating on that circular dependency, the traction-limited
%   branch is solved in closed form.  Substituting F_trac = mu*Nr into
%   m_eff*ax = F_trac - F_res gives
%
%       ax = [ mu*m*g*(cos(th)*a + sin(th)*h)/L - F_res ] / (m_eff - mu*m*h/L)
%
%   The denominator is the acceleration feedback through load transfer; it
%   going non-positive means the rear grip grows faster than the inertia
%   resisting it, i.e. the car is at the point of lifting the front wheels.
%   That case is trapped and reported through Nf rather than dividing by it.
%#codegen

    g  = P.env.g;
    m  = P.veh.m;
    L  = P.veh.Lwb;
    h  = P.veh.h;
    a  = P.veh.a;
    b  = P.veh.b;
    mu = P.veh.mu_t;

    cth = cos(grade);
    sth = sin(grade);

    % ---- resistances ---------------------------------------------
    % Rolling resistance is faded through zero speed with a tanh so that a
    % stationary car on a grade is held rather than shoved backwards.
    F_aero  = 0.5 * P.env.rho * P.veh.CdA * v * abs(v);
    F_roll  = P.veh.Crr * m * g * cth * tanh(v / 0.30);
    F_grade = m * g * sth;
    F_res   = F_aero + F_roll + F_grade;

    % ---- free (grip-unlimited) acceleration ----------------------
    ax_free = (F_dem - F_res) / P.veh.m_eff;

    % ---- rear grip available at that acceleration ----------------
    Nr_free = (m*g*cth*a + m*ax_free*h + m*g*sth*h) / L;
    F_max   = mu * max(Nr_free, 0);

    if F_dem <= F_max
        tract_lim = 0;
        ax     = ax_free;
        F_trac = F_dem;
    else
        tract_lim = 1;
        den = P.veh.m_eff - mu * m * h / L;
        if den > 1e-6
            ax = (mu * m * g * (cth*a + sth*h) / L - F_res) / den;
        else
            % Load transfer is self-reinforcing: grip is not the limit here,
            % the front axle is unloading.  Fall back to the free solution.
            ax = ax_free;
        end
        F_trac = P.veh.m_eff * ax + F_res;
    end

    Nf = (m*g*cth*b - m*ax*h - m*g*sth*h) / L;
    Nr = (m*g*cth*a + m*ax*h + m*g*sth*h) / L;
end
