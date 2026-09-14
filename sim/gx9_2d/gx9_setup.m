function ok = gx9_setup(varargin)
%GX9_SETUP  One-command entry point: check the environment, build, verify, run.
%
%   gx9_setup            check -> build GX9_2D.slx -> self-test -> scenarios
%   gx9_setup('Run',false)   stop after the self-test
%   gx9_setup('Plots',false) no figures
%
%   Nothing to install beyond MATLAB and Simulink. This checks that both are
%   actually there and licensed before it tries anything, because a missing
%   Simulink licence otherwise shows up as a confusing error deep inside
%   build_gx9_2d_model.
%
%   See also BUILD_GX9_2D_MODEL, GX9_SELFTEST, RUN_GX9_2D.

    opt = struct('Run',true,'Plots',true);
    for k = 1:2:numel(varargin), opt.(varargin{k}) = varargin{k+1}; end

    here = fileparts(mfilename('fullpath'));
    addpath(here);
    ok = true;

    fprintf('\n%s\n  GX9 2-D model - environment check\n%s\n', ...
            repmat('=',1,58), repmat('=',1,58));

    % ---- 1. MATLAB release ---------------------------------------
    v = version('-release');
    fprintf('  MATLAB                 %s\n', v);
    yr = str2double(v(1:4));
    if yr < 2018 || (yr == 2018 && v(5) == 'a')
        warning('gx9:oldRelease', ...
            ['R2018b or newer is recommended - the plots use yline/xline. ' ...
             'The model itself should still build.']);
    end

    % ---- 2. Simulink ---------------------------------------------
    hasSL = ~isempty(ver('simulink'));
    if hasSL
        licOK = license('test','Simulink');
    else
        licOK = false;
    end
    fprintf('  Simulink installed     %s\n', local_yn(hasSL));
    fprintf('  Simulink licensed      %s\n', local_yn(licOK));
    if ~hasSL || ~licOK
        fprintf(['\n  Simulink is required to build GX9_2D.slx.\n' ...
                 '  The physics can still be exercised with no MATLAB at all:\n' ...
                 '      python3 verify/gx9_reference.py\n\n']);
        ok = false;
        return
    end

    % ---- 3. Files -------------------------------------------------
    need = {'gx9_params.json','gx9_params.m','build_gx9_2d_model.m', ...
            'gx9_postprocess.m','gx9_sig_names.m','gx9_selftest.m', ...
            'run_gx9_2d.m', ...
            fullfile('blockfcn','gx9_cvt_geom.m'), ...
            fullfile('blockfcn','gx9_cvt_forces.m'), ...
            fullfile('blockfcn','gx9_belt.m'), ...
            fullfile('blockfcn','gx9_governor.m'), ...
            fullfile('blockfcn','gx9_vehicle_2d.m'), ...
            fullfile('verify','expected_metrics.json')};
    missing = need(~cellfun(@(f) exist(fullfile(here,f),'file')>0, need));
    fprintf('  Source files           %s\n', ...
            local_yn(isempty(missing)));
    if ~isempty(missing)
        fprintf('    missing: %s\n', strjoin(missing, ', '));
        ok = false;
        return
    end

    % ---- 4. Parameters load and are self-consistent ---------------
    try
        P = gx9_params(fullfile(here,'gx9_params.json'));
    catch err
        fprintf('  Parameters             FAIL (%s)\n', err.message);
        ok = false; return
    end
    fprintf('  Parameters             OK\n');
    fprintf('    ratio range          %.3f:1 -> %.3f:1\n', ...
            P.cvt.ratio_low, P.cvt.ratio_high);
    fprintf('    engagement           %.0f rpm\n', P.cvt.rpm_engage);
    fprintf('    sheave travel        %.1f mm primary, %.1f mm secondary\n', ...
            P.cvt.xp_max*1e3, P.cvt.xs_max*1e3);

    % ---- 5. Build -------------------------------------------------
    fprintf('\n  building GX9_2D.slx ...\n');
    try
        build_gx9_2d_model('Open',false);
    catch err
        fprintf('\n  BUILD FAILED: %s\n', err.message);
        fprintf('  at %s\n', local_where(err));
        fprintf(['\n  This model has been built from source but never run on a\n' ...
                 '  machine with Simulink. Send me the message above and the\n' ...
                 '  stack line and I will fix the builder.\n\n']);
        ok = false; return
    end
    fprintf('  build OK\n');

    % ---- 6. Verify against the reference solver --------------------
    try
        ok = gx9_selftest();
    catch err
        fprintf('\n  SELF-TEST FAILED TO RUN: %s\n', err.message);
        fprintf('  at %s\n\n', local_where(err));
        ok = false; return
    end

    % ---- 7. Scenarios ---------------------------------------------
    if opt.Run && ok
        run_gx9_2d('Plots', opt.Plots);
    end

    if ok
        fprintf(['  Everything checks out. Open the model with:\n' ...
                 '      open_system(''GX9_2D'')\n\n']);
    end
end

function s = local_yn(tf)
    if tf, s = 'yes'; else, s = 'NO'; end
end

function s = local_where(err)
    if isempty(err.stack)
        s = '(no stack)';
    else
        s = sprintf('%s line %d', err.stack(1).name, err.stack(1).line);
    end
end
