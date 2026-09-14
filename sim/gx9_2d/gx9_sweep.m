function T = gx9_sweep(paramPath, values, varargin)
%GX9_SWEEP  Sweep one GX9 parameter and tabulate what it does to performance.
%
%   T = GX9_SWEEP('cvt.mw', 0.18:0.02:0.30)
%   T = GX9_SWEEP('cvt.Fp0', 700:100:1200)
%   T = GX9_SWEEP('cvt.th', deg2rad(38:2:50))
%   T = GX9_SWEEP('veh.mu_t', 0.6:0.05:0.95, 'Plots', false)
%
%   paramPath is a dotted path into the struct returned by gx9_params, so any
%   scalar in the model can be swept.  The four knobs a Baja team actually
%   turns on a GX9 are:
%
%     cvt.mw    flyweight mass        - raises clamp force everywhere, shifts
%                                       the whole shift curve up in speed
%     cvt.Fp0   primary preload       - sets engagement RPM directly
%     cvt.th    helix angle           - how hard it backshifts under load
%     cvt.Fs0   secondary preload     - belt tension and backshift bias
%
%   Returns a table of 150 ft time, held engine speed, top speed, engagement
%   speed and peak belt slip against the swept value.

    opt = struct('Plots',true,'Model','GX9_2D');
    for k = 1:2:numel(varargin), opt.(varargin{k}) = varargin{k+1}; end

    here = fileparts(mfilename('fullpath'));
    addpath(here);
    mdl = opt.Model;
    if ~exist(fullfile(here,[mdl '.slx']),'file')
        build_gx9_2d_model('Name',mdl,'Open',false);
    end
    load_system(fullfile(here,[mdl '.slx']));

    P0 = gx9_params(fullfile(here,'gx9_params.json'));
    n  = numel(values);
    val = values(:);
    t150 = nan(n,1); heldRpm = nan(n,1); vmax = nan(n,1);
    engRpm = nan(n,1); slipMax = nan(n,1); shiftS = nan(n,1);

    for k = 1:n
        P = local_setfield(P0, paramPath, values(k));
        P = local_rederive(P);

        [Ma, ~] = local_run(mdl, P, 12);
        [Mt, ~] = local_run(mdl, P, 40);

        t150(k)    = Ma.t_150ft;
        heldRpm(k) = Ma.mean_locked_rpm;
        shiftS(k)  = Ma.shift_start_s;
        slipMax(k) = Ma.max_belt_slip;
        engRpm(k)  = P.cvt.rpm_engage;
        vmax(k)    = Mt.top_speed_kmh;
    end

    T = table(val, t150, heldRpm, shiftS, vmax, engRpm, slipMax, ...
        'VariableNames', {char(matlab.lang.makeValidName(paramPath)), ...
                          't150ft_s','held_rpm','shift_start_s', ...
                          'top_speed_kmh','engage_rpm','max_slip_radps'});
    disp(T);

    if opt.Plots
        figure('Name',['GX9 sweep: ' paramPath],'Color','w');
        subplot(1,3,1); plot(val, t150,'o-','LineWidth',1.4); grid on;
            xlabel(paramPath); ylabel('150 ft time [s]'); title('acceleration');
        subplot(1,3,2); plot(val, heldRpm,'o-','LineWidth',1.4); grid on;
            xlabel(paramPath); ylabel('held engine speed [rpm]'); title('shift point');
        subplot(1,3,3); plot(val, vmax,'o-','LineWidth',1.4); grid on;
            xlabel(paramPath); ylabel('top speed [km/h]'); title('top speed');
    end
end

% =====================================================================
function [M, S] = local_run(mdl, P, tEnd)
    si = Simulink.SimulationInput(mdl);
    si = si.setVariable('thr_cmd',   P.scn.throttle, 'Workspace', mdl);
    si = si.setVariable('grade_rad', P.scn.grade,    'Workspace', mdl);
    si = si.setVariable('P',         P,              'Workspace', mdl);
    si = si.setModelParameter('StopTime', num2str(tEnd));
    [M, S] = gx9_postprocess(sim(si), P);
end

function S = local_setfield(S, path, value)
    f = strsplit(path,'.');
    S = setfield(S, f{:}, value); %#ok<SFLD>
end

function P = local_rederive(P)
%LOCAL_REDERIVE  Refresh the quantities that depend on swept primitives.
%   Only the cheap algebraic ones: belt geometry is fixed hardware and is not
%   affected by any of the tunable knobs, so the rs table is left alone.
    P.cvt.tw     = 2*tan(P.cvt.alpha);
    P.cvt.xp_max = P.cvt.tw * (P.cvt.rp_max - P.cvt.rp_min);
    P.cvt.xs_max = P.cvt.tw * (P.cvt.rs_max - P.cvt.rs_min);
    P.cvt.w_engage   = sqrt(max(P.cvt.Fp0/(P.cvt.mw*P.cvt.Rw*P.cvt.ramp_a(1)),0));
    P.cvt.rpm_engage = P.cvt.w_engage*30/pi;
    P.veh.m_eff  = P.veh.m + (P.veh.Jw + P.dl.J_sec*P.dl.i_gb^2)/P.veh.rt^2;
    P.veh.b      = P.veh.Lwb - P.veh.a;
end
