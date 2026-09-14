function P = gx9_params(jsonFile)
%GX9_PARAMS  Parameter set for the GX9 2-D performance model.
%
%   P = GX9_PARAMS()           reads gx9_params.json next to this file.
%   P = GX9_PARAMS(jsonFile)   reads an alternative parameter file.
%
%   gx9_params.json holds every *primitive* number (hardware dimensions,
%   masses, spring rates, map data).  Everything that can be computed from
%   those primitives is derived here so that the JSON file stays the single
%   source of truth, shared with verify/gx9_reference.py.
%
%   The returned struct P is consumed directly by the Simulink model: every
%   MATLAB Function block in GX9_2D takes P as a *parameter* input.
%
%   See also BUILD_GX9_2D_MODEL, RUN_GX9_2D.

    if nargin < 1
        here     = fileparts(mfilename('fullpath'));
        jsonFile = fullfile(here, 'gx9_params.json');
    end
    J = jsondecode(fileread(jsonFile));

    % ---------------------------------------------------------------
    % Environment
    % ---------------------------------------------------------------
    P.env.rho = J.environment.air_density_kgm3;
    P.env.g   = J.environment.gravity_mps2;

    % ---------------------------------------------------------------
    % Engine
    % ---------------------------------------------------------------
    P.eng.rpm_brk   = J.engine.rpm_brk(:)';
    P.eng.trq_brk   = J.engine.torque_wot_Nm(:)';
    P.eng.w_brk     = P.eng.rpm_brk * pi/30;          % rad/s breakpoints
    P.eng.w_idle    = J.engine.idle_rpm    * pi/30;
    P.eng.w_gov     = J.engine.governor_rpm* pi/30;
    P.eng.w_droop   = J.engine.governor_droop_rpm * pi/30;
    P.eng.J         = J.engine.inertia_kgm2;

    % ---------------------------------------------------------------
    % CVT - primitives
    % ---------------------------------------------------------------
    C  = J.cvt.center_distance_m;
    L  = J.cvt.belt_pitch_length_m;
    P.cvt.C      = C;
    P.cvt.L      = L;
    P.cvt.rp_min = J.cvt.rp_min_m;
    P.cvt.rp_max = J.cvt.rp_max_m;
    P.cvt.alpha  = J.cvt.sheave_half_angle_deg * pi/180;

    P.cvt.mw     = J.cvt.flyweight_mass_total_kg;
    P.cvt.Rw     = J.cvt.flyweight_radius_m;
    P.cvt.ramp_x = J.cvt.ramp_travel_norm_brk(:)';
    P.cvt.ramp_a = J.cvt.ramp_advantage(:)';

    P.cvt.kp     = J.cvt.primary_spring_rate_Npm;
    P.cvt.Fp0    = J.cvt.primary_spring_preload_N;
    P.cvt.ks     = J.cvt.secondary_spring_rate_Npm;
    P.cvt.Fs0    = J.cvt.secondary_spring_preload_N;
    P.cvt.Rh     = J.cvt.helix_radius_m;
    P.cvt.th     = J.cvt.helix_angle_deg * pi/180;

    P.cvt.tau_T    = J.cvt.torque_sense_lag_s;
    P.cvt.mu_b     = J.cvt.belt_friction_coeff;
    P.cvt.k_slip   = J.cvt.belt_slip_stiffness_Nms_per_rad;
    P.cvt.F_eng_ref= J.cvt.engagement_clamp_ref_N;
    P.cvt.c_up     = J.cvt.shift_damping_up_Nspm;
    P.cvt.c_dn     = J.cvt.shift_damping_down_Nspm;

    P.cvt.eff_r    = J.cvt.eff_ratio_brk(:)';
    P.cvt.eff_T    = J.cvt.eff_torque_brk(:)';
    P.cvt.eff_tab  = J.cvt.eff_table;                 % rows = ratio, cols = torque

    % ---------------------------------------------------------------
    % CVT - derived belt geometry
    % ---------------------------------------------------------------
    % Open-belt length with pitch radii rp, rs and centre distance C:
    %
    %   beta = asin((rp-rs)/C)                (signed)
    %   Lbelt = pi*(rp+rs) + 2*beta*(rp-rs) + 2*C*cos(beta)
    %
    % Wrap angles follow directly: theta_p = pi + 2*beta, theta_s = pi - 2*beta.
    %
    % Differentiating at constant belt length gives the exact shift coupling
    %
    %   drs/drp = -(pi + 2*beta)/(pi - 2*beta) = -theta_p/theta_s
    %
    % which is the geometric lever between the two sheave axial forces.
    P.cvt.rp_brk = linspace(P.cvt.rp_min, P.cvt.rp_max, 201);
    P.cvt.rs_tab = zeros(1, numel(P.cvt.rp_brk));
    rs_guess = 0.08;
    for k = 1:numel(P.cvt.rp_brk)
        rs_guess          = gx9_solve_rs(P.cvt.rp_brk(k), C, L, rs_guess);
        P.cvt.rs_tab(k)   = rs_guess;
    end
    P.cvt.rs_max = P.cvt.rs_tab(1);
    P.cvt.rs_min = P.cvt.rs_tab(end);
    P.cvt.ratio_low  = P.cvt.rs_max / P.cvt.rp_min;   % numerically highest ratio
    P.cvt.ratio_high = P.cvt.rs_min / P.cvt.rp_max;   % overdrive

    % Axial sheave travel. Moving one movable face by dx closes the groove by
    % dx, which moves the belt pitch line by dx/(2*tan(alpha)).
    P.cvt.tw     = 2*tan(P.cvt.alpha);
    P.cvt.xp_max = P.cvt.tw * (P.cvt.rp_max - P.cvt.rp_min);
    P.cvt.xs_max = P.cvt.tw * (P.cvt.rs_max - P.cvt.rs_min);

    % Engagement speed: flyweight force at rest position equals spring preload.
    w2 = P.cvt.Fp0 / (P.cvt.mw * P.cvt.Rw * P.cvt.ramp_a(1));
    P.cvt.w_engage   = sqrt(max(w2, 0));
    P.cvt.rpm_engage = P.cvt.w_engage * 30/pi;

    % ---------------------------------------------------------------
    % Driveline
    % ---------------------------------------------------------------
    P.dl.i_gb  = J.driveline.gearbox_ratio;
    P.dl.eta_gb= J.driveline.gearbox_efficiency;
    P.dl.J_sec = J.driveline.secondary_side_inertia_kgm2;

    % ---------------------------------------------------------------
    % Vehicle (2-D: longitudinal + pitch load transfer)
    % ---------------------------------------------------------------
    P.veh.m     = J.vehicle.mass_vehicle_kg + J.vehicle.mass_driver_kg;
    P.veh.Lwb   = J.vehicle.wheelbase_m;
    P.veh.a     = J.vehicle.cg_to_front_axle_m;    % front axle -> CG
    P.veh.b     = P.veh.Lwb - P.veh.a;             % CG -> rear axle
    P.veh.h     = J.vehicle.cg_height_m;
    P.veh.rt    = J.vehicle.tire_radius_m;
    P.veh.Jw    = J.vehicle.wheel_inertia_total_kgm2;
    P.veh.mu_t  = J.vehicle.tire_friction_coeff;
    P.veh.Crr   = J.vehicle.rolling_resistance_coeff;
    P.veh.CdA   = J.vehicle.drag_coefficient * J.vehicle.frontal_area_m2;

    % Effective translating mass: vehicle mass plus every rotating inertia
    % downstream of the belt, referred to the road.  The engine/primary
    % inertia is NOT included here - it is a separate state, because the belt
    % can slip and let the engine speed run away from the vehicle speed.
    P.veh.m_eff = P.veh.m + (P.veh.Jw + P.dl.J_sec * P.dl.i_gb^2) / P.veh.rt^2;

    P.veh.static_rear_frac = P.veh.a / P.veh.Lwb;

    % ---------------------------------------------------------------
    % Scenario defaults
    % ---------------------------------------------------------------
    P.scn.grade    = J.scenario.grade_deg * pi/180;
    P.scn.throttle = J.scenario.throttle;
    P.scn.t_end    = J.scenario.sim_time_s;
    P.scn.dt_max   = J.scenario.max_step_s;
    P.scn.accel_m  = J.scenario.accel_distance_m;
end

% -------------------------------------------------------------------
function rs = gx9_solve_rs(rp, C, L, rs0)
%GX9_SOLVE_RS  Secondary pitch radius that keeps the belt length at L.
%   Newton iteration on Lbelt(rp,rs) - L = 0.  dLbelt/drs = pi - 2*beta,
%   which is the secondary wrap angle and therefore always positive, so the
%   iteration is unconditionally well behaved.
    rs = rs0;
    for it = 1:60
        d    = rp - rs;
        beta = asin(max(min(d/C, 0.999), -0.999));
        f    = pi*(rp+rs) + 2*beta*d + 2*C*cos(beta) - L;
        df   = pi - 2*beta;
        step = f/df;
        rs   = rs - step;
        if abs(step) < 1e-14
            break
        end
    end
end
