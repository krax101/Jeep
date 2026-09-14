function ok = gx9_selftest(varargin)
%GX9_SELFTEST  Check GX9_2D against the standalone reference solver.
%
%   ok = GX9_SELFTEST()               build if needed, run, compare, report
%   ok = GX9_SELFTEST('Rebuild',true) force a rebuild first
%   ok = GX9_SELFTEST('Tol',0.05)     relative tolerance (default 3%)
%
%   verify/gx9_reference.py implements the same equations in plain Python and
%   writes verify/expected_metrics.json.  If the Simulink model and that
%   solver disagree by more than Tol, one of them has drifted - which is
%   exactly what this test is for.  Regenerate the baseline after an
%   intentional parameter change with:
%
%       python3 verify/gx9_reference.py --write
%
%   The two solvers are not identical numerically: the reference uses fixed
%   step explicit Euler, the model a variable-step implicit solver, so a few
%   percent of disagreement on event times is expected and tolerated.

    opt = struct('Rebuild',false,'Tol',0.03,'Model','GX9_2D');
    for k = 1:2:numel(varargin), opt.(varargin{k}) = varargin{k+1}; end

    here = fileparts(mfilename('fullpath'));
    addpath(here);
    mdl  = opt.Model;

    if opt.Rebuild || ~exist(fullfile(here,[mdl '.slx']),'file')
        build_gx9_2d_model('Name',mdl,'Open',false);
    end
    load_system(fullfile(here,[mdl '.slx']));

    P   = gx9_params(fullfile(here,'gx9_params.json'));
    ref = jsondecode(fileread(fullfile(here,'verify','expected_metrics.json')));

    [Ma, ~] = local_run(mdl, P, 12);
    [Mt, ~] = local_run(mdl, P, 40);

    % name, model value, reference value, tolerance (relative unless absolute
    % given as a negative number, in which case |diff| is compared instead)
    checks = {
        'ratio_low',        P.cvt.ratio_low,     ref.ratio_low,        1e-6
        'ratio_high',       P.cvt.ratio_high,    ref.ratio_high,       1e-6
        'engagement rpm',   P.cvt.rpm_engage,    ref.rpm_engage,       1e-6
        'primary travel mm',P.cvt.xp_max*1e3,    ref.xp_max_mm,        1e-6
        'effective mass kg',P.veh.m_eff,         ref.m_eff_kg,         1e-6
        '100 ft time',      Ma.t_100ft,          ref.t_100ft_s,        opt.Tol
        '150 ft time',      Ma.t_150ft,          ref.t_150ft_s,        opt.Tol
        '0-30 km/h',        Ma.t_0_30kmh,        ref.t_0_30kmh_s,      opt.Tol
        'peak accel',       Ma.peak_ax,          ref.peak_ax_mps2,     opt.Tol
        'held rpm',         Ma.mean_locked_rpm,  ref.mean_locked_rpm,  opt.Tol
        'min front load',   Ma.min_front_load_N, ref.min_front_load_N, opt.Tol
        'top speed km/h',   Mt.top_speed_kmh,    ref.top_speed_kmh,    opt.Tol
        };

    fprintf('\n%s\n  GX9_2D self-test vs reference solver\n%s\n', ...
            repmat('=',1,64), repmat('=',1,64));
    fprintf('  %-20s %12s %12s %10s  %s\n','quantity','model','reference','rel err','');
    ok = true;
    for k = 1:size(checks,1)
        [nm, got, want, tol] = checks{k,:};
        if want == 0
            relerr = abs(got - want);
        else
            relerr = abs(got - want)/abs(want);
        end
        pass = relerr <= tol;
        ok   = ok && pass;
        if pass, mark = 'PASS'; else, mark = 'FAIL'; end
        fprintf('  %-20s %12.4f %12.4f %9.3f%%  %s\n', nm, got, want, 100*relerr, mark);
    end

    % Physical invariants - these must hold regardless of the reference.
    [~, S] = local_run(mdl, P, 12);
    inv = {
        'belt length constant',   local_belt_err(S, P) < 1e-6
        'ratio within hardware',  all(S.ratio >= P.cvt.ratio_high-1e-6) && ...
                                  all(S.ratio <= P.cvt.ratio_low +1e-6)
        'axle loads sum to mg',   max(abs(S.Nf + S.Nr - P.veh.m*P.env.g)) < 1e-6
        'no wheelie (Nf >= 0)',   all(S.Nf >= 0) && all(S.Nr >= 0)
        'efficiency in (0,1]',    all(S.eta_cvt > 0 & S.eta_cvt <= 1)
        'belt within capacity',   all(abs(S.T_pri) <= S.T_cap + 1e-6)
        'engine speed bounded',   max(S.rpm) <= P.eng.w_gov*30/pi + P.eng.w_droop*30/pi
        };
    fprintf('\n  physical invariants\n');
    for k = 1:size(inv,1)
        pass = inv{k,2};
        ok   = ok && pass;
        if pass, mark = 'PASS'; else, mark = 'FAIL'; end
        fprintf('  %-28s %s\n', inv{k,1}, mark);
    end

    if ok
        fprintf('\n  all checks passed\n\n');
    else
        fprintf('\n  SOME CHECKS FAILED\n\n');
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

function e = local_belt_err(S, P)
%LOCAL_BELT_ERR  Worst belt-length error over the run.
%   The whole shift model rests on the belt being inextensible, so this is
%   the one invariant worth checking every single time.
    d    = S.rp - S.rs;
    beta = asin(min(max(d/P.cvt.C,-0.999),0.999));
    L    = pi*(S.rp + S.rs) + 2*beta.*d + 2*P.cvt.C*cos(beta);
    e    = max(abs(L - P.cvt.L));
end
