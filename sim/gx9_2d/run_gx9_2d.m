function R = run_gx9_2d(varargin)
%RUN_GX9_2D  Run the GX9 2-D performance model and report the numbers.
%
%   R = RUN_GX9_2D()                    acceleration + top speed + gradeability
%   R = RUN_GX9_2D('Scenario','accel')  one scenario only
%   R = RUN_GX9_2D('Grade',15)          hold a 15 degree grade
%   R = RUN_GX9_2D('Throttle',0.5)      part throttle
%   R = RUN_GX9_2D('Plots',false)       numbers only
%
%   Scenarios: 'accel' | 'topspeed' | 'grade' | 'gradeability' | 'all'
%
%   R.accel / R.topspeed hold {M, S} from gx9_postprocess; R.gradeability_deg
%   is the steepest grade the car still climbs.
%
%   See also BUILD_GX9_2D_MODEL, GX9_POSTPROCESS, GX9_SWEEP, GX9_SELFTEST.

    opt = struct('Scenario','all','Grade',0,'Throttle',[], ...
                 'Plots',true,'Model','GX9_2D','Rebuild',false);
    for k = 1:2:numel(varargin), opt.(varargin{k}) = varargin{k+1}; end

    here = fileparts(mfilename('fullpath'));
    addpath(here);
    mdl = opt.Model;

    if opt.Rebuild || ~exist(fullfile(here,[mdl '.slx']),'file')
        build_gx9_2d_model('Name',mdl,'Open',false);
    end
    load_system(fullfile(here,[mdl '.slx']));

    P = gx9_params(fullfile(here,'gx9_params.json'));
    if ~isempty(opt.Throttle), P.scn.throttle = opt.Throttle; end

    R = struct('P',P);
    doAll = strcmpi(opt.Scenario,'all');

    if doAll || strcmpi(opt.Scenario,'accel')
        [M,S] = local_run(mdl, P, P.scn.throttle, 0, 12);
        R.accel = struct('M',M,'S',S);
        fprintf('\n--- acceleration run (WOT, flat) ---');
        gx9_report(M);
    end

    if doAll || strcmpi(opt.Scenario,'topspeed')
        [M,S] = local_run(mdl, P, P.scn.throttle, 0, 40);
        R.topspeed = struct('M',M,'S',S);
        fprintf('\n--- top speed ---\n');
        fprintf('  %.1f km/h at %.0f rpm, ratio %.2f:1\n', ...
                M.top_speed_kmh, M.rpm_at_top_speed, M.ratio_at_top_speed);
    end

    if strcmpi(opt.Scenario,'grade')
        [M,S] = local_run(mdl, P, P.scn.throttle, opt.Grade*pi/180, 20);
        R.grade = struct('M',M,'S',S,'deg',opt.Grade);
        fprintf('\n--- %.1f deg grade (%.0f%%) ---\n', ...
                opt.Grade, 100*tan(opt.Grade*pi/180));
        fprintf('  settled speed %.1f km/h, held %.0f rpm, ratio %.2f:1\n', ...
                S.kmh(end), S.rpm(end), S.ratio(end));
    end

    if doAll || strcmpi(opt.Scenario,'gradeability')
        R.gradeability_deg = local_gradeability(mdl, P);
        fprintf('\n--- gradeability ---\n');
        fprintf('  max sustained grade %.1f deg (%.0f%%)\n\n', ...
                R.gradeability_deg, 100*tan(R.gradeability_deg*pi/180));
    end

    if opt.Plots && isfield(R,'accel')
        gx9_plots(R, P);
    end
end

% =====================================================================
function [M, S] = local_run(mdl, P, thr, grade, tEnd)
    si = Simulink.SimulationInput(mdl);
    si = si.setVariable('thr_cmd',   thr,   'Workspace', mdl);
    si = si.setVariable('grade_rad', grade, 'Workspace', mdl);
    si = si.setVariable('P',         P,     'Workspace', mdl);
    si = si.setModelParameter('StopTime', num2str(tEnd));
    out = sim(si);
    [M, S] = gx9_postprocess(out, P);
end

% =====================================================================
function deg = local_gradeability(mdl, P)
%LOCAL_GRADEABILITY  Bisection on grade for the steepest sustained climb.
%   "Sustained" means still moving above 0.5 m/s after 12 s, which catches
%   both the grip limit and the torque limit without needing to know which
%   one bit first.
    lo = 0; hi = 45;
    if ~local_climbs(mdl, P, lo), deg = 0; return; end
    if  local_climbs(mdl, P, hi), deg = hi; return; end
    for k = 1:8
        mid = 0.5*(lo+hi);
        if local_climbs(mdl, P, mid), lo = mid; else, hi = mid; end
    end
    deg = lo;
end

function ok = local_climbs(mdl, P, deg)
    [~, S] = local_run(mdl, P, P.scn.throttle, deg*pi/180, 12);
    ok = S.v(end) > 0.5;
end

% =====================================================================
function gx9_report(M)
    fprintf('\n  0 -> 30 km/h          %s\n', local_fmt(M.t_0_30kmh));
    fprintf('  0 -> 50 km/h          %s\n', local_fmt(M.t_0_50kmh));
    fprintf('  100 ft                %s\n', local_fmt(M.t_100ft));
    fprintf('  150 ft (Baja event)   %s\n', local_fmt(M.t_150ft));
    fprintf('  peak acceleration     %.2f m/s^2 (%.2f g)\n', M.peak_ax, M.peak_ax_g);
    fprintf('  upshift begins        %.2f s\n', M.shift_start_s);
    fprintf('  held engine speed     %.0f rpm\n', M.mean_locked_rpm);
    fprintf('  peak belt slip        %.1f rad/s\n', M.max_belt_slip);
    fprintf('  min front axle load   %.0f N\n', M.min_front_load_N);
    fprintf('  traction limited      %.0f%% of the run\n', 100*M.traction_limited_frac);
end

function s = local_fmt(t)
    if isnan(t), s = 'not reached'; else, s = sprintf('%.2f s', t); end
end

% =====================================================================
function gx9_plots(R, P)
    S = R.accel.S;

    figure('Name','GX9 - acceleration','Color','w');
    subplot(2,2,1);
      yyaxis left;  plot(S.t, S.kmh,'LineWidth',1.4); ylabel('speed [km/h]');
      yyaxis right; plot(S.t, S.dist,'LineWidth',1.1); ylabel('distance [m]');
      yline(P.scn.accel_m,'k:'); xlabel('t [s]'); grid on; title('speed and distance');
    subplot(2,2,2);
      yyaxis left;  plot(S.t, S.rpm,'LineWidth',1.4); ylabel('engine [rpm]');
      yyaxis right; plot(S.t, S.ratio,'LineWidth',1.1); ylabel('CVT ratio');
      xlabel('t [s]'); grid on; title('engine speed and ratio');
    subplot(2,2,3);
      plot(S.t, S.F_trac,'LineWidth',1.4); hold on;
      plot(S.t, P.veh.mu_t*S.Nr,'--','LineWidth',1.1);
      plot(S.t, S.F_res,'LineWidth',1.1);
      legend('tractive','grip limit \mu N_r','resistance','Location','best');
      xlabel('t [s]'); ylabel('force [N]'); grid on; title('longitudinal forces');
    subplot(2,2,4);
      plot(S.t, S.Nf,'LineWidth',1.4); hold on; plot(S.t, S.Nr,'LineWidth',1.4);
      yline(0,'k:'); legend('front','rear','Location','best');
      xlabel('t [s]'); ylabel('axle normal load [N]'); grid on;
      title('load transfer (the 2-D part)');

    figure('Name','GX9 - CVT behaviour','Color','w');
    subplot(2,2,1);
      plot(S.kmh, S.rpm,'LineWidth',1.4); xlabel('speed [km/h]');
      ylabel('engine [rpm]'); grid on; title('shift curve');
    subplot(2,2,2);
      plot(S.kmh, S.ratio,'LineWidth',1.4); xlabel('speed [km/h]');
      ylabel('ratio'); grid on; title('ratio vs speed');
    subplot(2,2,3);
      plot(S.t, S.Fp,'LineWidth',1.2); hold on;
      plot(S.t, S.Fs,'LineWidth',1.2); plot(S.t, S.F_shift,'LineWidth',1.4);
      yline(0,'k:'); legend('F_p primary','F_s secondary','F_{shift} net','Location','best');
      xlabel('t [s]'); ylabel('axial force [N]'); grid on; title('sheave force balance');
    subplot(2,2,4);
      yyaxis left;  plot(S.t, S.slip_w,'LineWidth',1.4); ylabel('belt slip [rad/s]');
      yyaxis right; plot(S.t, S.cap_margin,'LineWidth',1.1); ylabel('capacity margin [N*m]');
      xlabel('t [s]'); grid on; title('belt slip and torque headroom');

    local_tractive_effort(P);
end

% =====================================================================
function local_tractive_effort(P)
%LOCAL_TRACTIVE_EFFORT  Quasi-static tractive effort vs speed.
%   The classic performance diagram: one curve per CVT ratio, the envelope
%   they trace out, the rear-grip ceiling, and the road load at several
%   grades.  Where the envelope crosses a road-load curve is the top speed on
%   that grade; where it stops clearing one is the gradeability limit.
    figure('Name','GX9 - tractive effort','Color','w'); hold on; grid on;

    rpSel = P.cvt.rp_brk(1:20:end);
    vAll = []; fAll = [];
    for rp = rpSel
        rs    = interp1(P.cvt.rp_brk, P.cvt.rs_tab, rp);
        ratio = rs/rp;
        rpm   = linspace(P.cvt.rpm_engage, P.eng.w_gov*30/pi, 60);
        w     = rpm*pi/30;
        Tw    = interp1(P.eng.rpm_brk, P.eng.trq_brk, ...
                        min(max(rpm, P.eng.rpm_brk(1)), P.eng.rpm_brk(end)));
        Tsec  = Tw * ratio;
        eta   = interp2(P.cvt.eff_T, P.cvt.eff_r, P.cvt.eff_tab, abs(Tsec), ratio*ones(size(Tsec)));
        eta(isnan(eta)) = P.cvt.eff_tab(end,end);
        F     = Tsec .* eta * P.dl.i_gb * P.dl.eta_gb / P.veh.rt;
        v     = w * P.veh.rt / (ratio * P.dl.i_gb) * 3.6;
        hRatio = plot(v, F, 'Color',[0.75 0.78 0.82]);
        vAll = [vAll v]; fAll = [fAll F]; %#ok<AGROW>
    end

    [vs, idx] = sort(vAll); fs = fAll(idx);
    vGrid = linspace(0, max(vs), 200);
    env   = zeros(size(vGrid));
    for k = 1:numel(vGrid)-1
        sel = vs >= vGrid(k) & vs < vGrid(k+1);
        if any(sel), env(k) = max(fs(sel)); else, env(k) = NaN; end
    end
    hEnv = plot(vGrid, env, 'k', 'LineWidth', 2);

    % rear-grip ceiling at steady state (no load transfer from acceleration)
    Nr0 = P.veh.m*P.env.g*P.veh.a/P.veh.Lwb;
    hGrip = yline(P.veh.mu_t*Nr0, 'r--', 'LineWidth',1.4);

    vv = linspace(0, max(vGrid), 200)/3.6;
    for deg = [0 10 20 30]
        th = deg*pi/180;
        RL = 0.5*P.env.rho*P.veh.CdA*vv.^2 + P.veh.Crr*P.veh.m*P.env.g*cos(th) ...
             + P.veh.m*P.env.g*sin(th);
        hRoad = plot(vv*3.6, RL, 'LineWidth',1.2);
        text(max(vv)*3.6*0.82, RL(end), sprintf(' %d deg', deg), 'FontSize',8);
    end

    xlabel('speed [km/h]'); ylabel('tractive force [N]');
    title('GX9 tractive effort envelope vs road load');
    legend([hRatio hEnv hGrip hRoad], ...
           {'constant-ratio curves','envelope','rear grip limit','road load by grade'}, ...
           'Location','northeast');
end
