function [M, S] = gx9_postprocess(simOut, P, verbose)
%GX9_POSTPROCESS  Turn a GX9_2D run into named signals and performance metrics.
%
%   [M, S] = GX9_POSTPROCESS(simOut, P)        metrics M, signal struct S
%   [M, S] = GX9_POSTPROCESS(simOut, P, true)  also print the report
%
%   S has one field per channel of gx9_sig_names, plus S.t and a few derived
%   channels (rpm, kmh, power).  M holds the numbers you would actually quote:
%   event times, top speed, held engine speed, grip and slip margins.

    if nargin < 3, verbose = false; end

    names = gx9_sig_names();
    ts    = simOut.sig;
    S.t   = ts.Time(:);
    D     = squeeze(ts.Data);
    if size(D,1) ~= numel(S.t), D = D.'; end
    for k = 1:numel(names)
        S.(names{k}) = D(:,k);
    end

    % derived
    S.rpm     = S.w_e * 30/pi;
    S.kmh     = S.v   * 3.6;
    S.P_eng_W = S.T_e .* S.w_e;
    S.P_whl_W = S.F_trac .* S.v;
    S.cap_margin = S.T_cap - abs(S.T_pri);   % belt torque headroom

    % ---- metrics --------------------------------------------------
    M.t_100ft   = local_cross(S.t, S.dist, 30.48);
    M.t_150ft   = local_cross(S.t, S.dist, P.scn.accel_m);
    M.t_0_30kmh = local_cross(S.t, S.kmh, 30);
    M.t_0_50kmh = local_cross(S.t, S.kmh, 50);

    [M.top_speed_kmh, iTop] = max(S.kmh);
    M.rpm_at_top_speed   = S.rpm(iTop);
    M.ratio_at_top_speed = S.ratio(iTop);

    M.peak_ax            = max(S.ax);
    M.peak_ax_g          = M.peak_ax / P.env.g;
    M.min_front_load_N   = min(S.Nf);
    M.traction_limited_frac = mean(S.tract_lim);

    M.max_rpm            = max(S.rpm);
    M.max_belt_slip      = max(abs(S.slip_w));
    clamped = S.engage > 0.99;
    if any(clamped)
        M.min_cap_margin = min(S.cap_margin(clamped));
    else
        M.min_cap_margin = NaN;   % belt never fully clamped in this run
    end
    M.peak_engine_kW     = max(S.P_eng_W)/1e3;

    locked = S.engage >= 0.999 & abs(S.slip_w) < 5;
    if any(locked)
        M.mean_locked_rpm = mean(S.rpm(locked));
    else
        M.mean_locked_rpm = NaN;  % belt slipped for the whole run
    end

    iShift = find(S.ratio < P.cvt.ratio_low - 0.05, 1, 'first');
    if isempty(iShift), M.shift_start_s = NaN; else, M.shift_start_s = S.t(iShift); end

    M.ratio_low  = P.cvt.ratio_low;
    M.ratio_high = P.cvt.ratio_high;
    M.rpm_engage = P.cvt.rpm_engage;

    if verbose
        fprintf('\n%s\n  Gaged GX9 - 2-D performance model\n%s\n', ...
                repmat('=',1,60), repmat('=',1,60));
        fprintf('\n[ hardware ]\n');
        fprintf('  ratio range          %.3f:1 -> %.3f:1\n', M.ratio_low, M.ratio_high);
        fprintf('  engagement speed     %.0f rpm\n', M.rpm_engage);
        fprintf('\n[ acceleration, WOT ]\n');
        fprintf('  0 -> 30 km/h         %s\n', local_fmt(M.t_0_30kmh));
        fprintf('  0 -> 50 km/h         %s\n', local_fmt(M.t_0_50kmh));
        fprintf('  100 ft               %s\n', local_fmt(M.t_100ft));
        fprintf('  150 ft               %s\n', local_fmt(M.t_150ft));
        fprintf('  peak acceleration    %.2f m/s^2 (%.2f g)\n', M.peak_ax, M.peak_ax_g);
        fprintf('  traction limited     %.0f%% of the run\n', 100*M.traction_limited_frac);
        fprintf('  min front axle load  %.0f N\n', M.min_front_load_N);
        fprintf('\n[ CVT ]\n');
        fprintf('  upshift begins       %.2f s\n', M.shift_start_s);
        fprintf('  held engine speed    %.0f rpm\n', M.mean_locked_rpm);
        fprintf('  peak engine speed    %.0f rpm\n', M.max_rpm);
        fprintf('  peak belt slip       %.1f rad/s\n', M.max_belt_slip);
        fprintf('  min capacity margin  %.1f N*m\n', M.min_cap_margin);
        fprintf('\n[ steady state ]\n');
        fprintf('  top speed            %.1f km/h at %.0f rpm, ratio %.2f:1\n\n', ...
                M.top_speed_kmh, M.rpm_at_top_speed, M.ratio_at_top_speed);
    end
end

function t = local_cross(tv, x, level)
%LOCAL_CROSS  First time x rises through level, linearly interpolated.
    i = find(x(1:end-1) < level & x(2:end) >= level, 1, 'first');
    if isempty(i)
        t = NaN;
    else
        f = (level - x(i)) / (x(i+1) - x(i));
        t = tv(i) + f*(tv(i+1) - tv(i));
    end
end

function s = local_fmt(t)
    if isnan(t), s = 'not reached'; else, s = sprintf('%.2f s', t); end
end
