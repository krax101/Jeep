function mdl = build_gx9_2d_model(varargin)
%BUILD_GX9_2D_MODEL  Programmatically build the GX9 2-D performance model.
%
%   mdl = BUILD_GX9_2D_MODEL()                builds and saves GX9_2D.slx
%   mdl = BUILD_GX9_2D_MODEL('Name','X')      builds under a different name
%   mdl = BUILD_GX9_2D_MODEL('Open',false)    build without opening the editor
%   mdl = BUILD_GX9_2D_MODEL('Save',false)    build in memory only
%
%   The model is built from source rather than shipped as a binary .slx so
%   that it can be reviewed, diffed and merged like the rest of this repo.
%   Re-run it after editing gx9_params.json - the breakpoint tables are baked
%   into the model workspace at build time.
%
%   Structure
%   ---------
%     GX9_2D
%       |- Engine       Briggs & Stratton Model 19, WOT map + droop governor
%       |- CVT_GX9      Gaged GX9: belt geometry, sheave force balance,
%       |               shift dynamics, slipping-belt coupling, loss map
%       |- Driveline    gearbox reduction and efficiency, wheel torque
%       |- Vehicle_2D   planar vehicle: longitudinal + pitch load transfer
%
%   States: engine speed, primary pitch radius (the shift state), vehicle
%   speed, distance travelled.
%
%   See also GX9_PARAMS, RUN_GX9_2D, GX9_POSTPROCESS.

    opt = struct('Name','GX9_2D','Open',true,'Save',true);
    for k = 1:2:numel(varargin)
        opt.(varargin{k}) = varargin{k+1};
    end
    mdl  = opt.Name;
    here = fileparts(mfilename('fullpath'));

    P = gx9_params(fullfile(here,'gx9_params.json'));

    % ---- fresh model ---------------------------------------------
    if bdIsLoaded(mdl)
        close_system(mdl, 0);
    end
    new_system(mdl);
    load_system('simulink');

    mws = get_param(mdl,'ModelWorkspace');
    assignin(mws,'P',        P);
    assignin(mws,'thr_cmd',  P.scn.throttle);
    assignin(mws,'grade_rad',P.scn.grade);

    set_param(mdl, ...
        'Solver','ode23t', ...
        'SolverType','Variable-step', ...
        'StopTime', num2str(P.scn.t_end), ...
        'MaxStep',  num2str(P.scn.dt_max), ...
        'RelTol','1e-6', ...
        'AbsTol','1e-8', ...
        'ZeroCrossControl','UseLocalSettings', ...
        'Description', sprintf([ ...
            'Gaged Engineering GX9 CVT - 2-D longitudinal vehicle performance model.\n' ...
            'Ratio range %.2f:1 (low) to %.2f:1 (overdrive), engagement ~%.0f rpm.\n' ...
            'Parameters live in the model workspace variable P (see gx9_params.m).'], ...
            P.cvt.ratio_low, P.cvt.ratio_high, P.cvt.rpm_engage));

    % ---- subsystems ----------------------------------------------
    local_engine   (mdl, here);
    local_cvt      (mdl, here);
    local_driveline(mdl);
    local_vehicle  (mdl, here);

    % ---- top-level sources ---------------------------------------
    add_block('simulink/Sources/Constant',[mdl '/Throttle'], ...
        'Value','thr_cmd','Position',[30 60 80 90]);
    add_block('simulink/Sources/Constant',[mdl '/Grade'], ...
        'Value','grade_rad','Position',[30 430 80 460]);

    % ---- top-level wiring ----------------------------------------
    add_line(mdl,'Throttle/1','Engine/1','autorouting','on');
    add_line(mdl,'CVT_GX9/1','Engine/2','autorouting','on');   % belt torque = engine load
    add_line(mdl,'Engine/1','CVT_GX9/1','autorouting','on');   % engine speed
    add_line(mdl,'Driveline/2','CVT_GX9/2','autorouting','on');% secondary speed
    add_line(mdl,'CVT_GX9/2','Driveline/1','autorouting','on');% secondary torque
    add_line(mdl,'Vehicle_2D/2','Driveline/2','autorouting','on'); % wheel speed
    add_line(mdl,'Driveline/1','Vehicle_2D/1','autorouting','on'); % tractive demand
    add_line(mdl,'Grade/1','Vehicle_2D/2','autorouting','on');

    % ---- logging --------------------------------------------------
    % One wide bus rather than twenty scattered To Workspace blocks; the
    % channel order is owned by gx9_sig_names.m.
    add_block('simulink/Signal Routing/Mux',[mdl '/LogMux'], ...
        'Inputs','10','Position',[780 60 790 520]);
    add_block('simulink/Sinks/To Workspace',[mdl '/Log'], ...
        'VariableName','sig','SaveFormat','Timeseries', ...
        'Position',[840 270 900 310]);

    add_line(mdl,'Engine/1',    'LogMux/1','autorouting','on');
    add_line(mdl,'Engine/2',    'LogMux/2','autorouting','on');
    add_line(mdl,'CVT_GX9/1',   'LogMux/3','autorouting','on');
    add_line(mdl,'CVT_GX9/2',   'LogMux/4','autorouting','on');
    add_line(mdl,'CVT_GX9/3',   'LogMux/5','autorouting','on');
    add_line(mdl,'CVT_GX9/4',   'LogMux/6','autorouting','on');
    add_line(mdl,'CVT_GX9/5',   'LogMux/7','autorouting','on');
    add_line(mdl,'Vehicle_2D/1','LogMux/8','autorouting','on');
    add_line(mdl,'Vehicle_2D/3','LogMux/9','autorouting','on');
    add_line(mdl,'Vehicle_2D/4','LogMux/10','autorouting','on');
    add_line(mdl,'LogMux/1','Log/1','autorouting','on');

    if opt.Save
        save_system(mdl, fullfile(here,[mdl '.slx']));
        fprintf('Built and saved %s\n', fullfile(here,[mdl '.slx']));
    end
    if opt.Open
        open_system(mdl);
    end
end

% =====================================================================
function local_engine(mdl, here)
    s = [mdl '/Engine'];
    local_newsub(s, [150 40 260 120]);

    add_block('simulink/Sources/In1',   [s '/thr_cmd'],'Port','1','Position',[20  40  50  60]);
    add_block('simulink/Sources/In1',   [s '/T_load'], 'Port','2','Position',[20 200  50 220]);
    add_block('simulink/Sinks/Out1',    [s '/w_e'],    'Port','1','Position',[560 100 590 120]);
    add_block('simulink/Sinks/Out1',    [s '/T_e'],    'Port','2','Position',[560 170 590 190]);

    local_mlfcn([s '/Governor'], fullfile(here,'blockfcn','gx9_governor.m'), [110 35 190 85]);

    add_block('simulink/Math Operations/Gain',[s '/rad2rpm'], ...
        'Gain','30/pi','Position',[110 260 150 290]);
    add_block('simulink/Lookup Tables/1-D Lookup Table',[s '/WOT_Torque_Map'], ...
        'BreakpointsForDimension1','P.eng.rpm_brk', ...
        'Table','P.eng.trq_brk', ...
        'Position',[200 250 260 300]);
    local_setopt([s '/WOT_Torque_Map'],'ExtrapMethod','Clip');
    add_block('simulink/Math Operations/Product',[s '/ApplyThrottle'], ...
        'Inputs','2','Position',[310 150 340 190]);
    add_block('simulink/Math Operations/Sum',[s '/NetTorque'], ...
        'Inputs','+-','Position',[390 150 420 190]);
    add_block('simulink/Math Operations/Gain',[s '/invJ'], ...
        'Gain','1/P.eng.J','Position',[440 155 480 185]);
    add_block('simulink/Continuous/Integrator',[s '/w_engine'], ...
        'InitialCondition','P.eng.w_idle', ...
        'LimitOutput','on','UpperSaturationLimit','inf', ...
        'LowerSaturationLimit','0','Position',[500 150 530 190]);

    add_line(s,'thr_cmd/1','Governor/1','autorouting','on');
    add_line(s,'w_engine/1','Governor/2','autorouting','on');
    add_line(s,'w_engine/1','rad2rpm/1','autorouting','on');
    add_line(s,'rad2rpm/1','WOT_Torque_Map/1','autorouting','on');
    add_line(s,'WOT_Torque_Map/1','ApplyThrottle/1','autorouting','on');
    add_line(s,'Governor/1','ApplyThrottle/2','autorouting','on');
    add_line(s,'ApplyThrottle/1','NetTorque/1','autorouting','on');
    add_line(s,'T_load/1','NetTorque/2','autorouting','on');
    add_line(s,'NetTorque/1','invJ/1','autorouting','on');
    add_line(s,'invJ/1','w_engine/1','autorouting','on');
    add_line(s,'w_engine/1','w_e/1','autorouting','on');
    add_line(s,'ApplyThrottle/1','T_e/1','autorouting','on');
end

% =====================================================================
function local_cvt(mdl, here)
    s = [mdl '/CVT_GX9'];
    local_newsub(s, [330 40 460 200]);

    add_block('simulink/Sources/In1',[s '/w_e'],  'Port','1','Position',[20  40  50  60]);
    add_block('simulink/Sources/In1',[s '/w_sec'],'Port','2','Position',[20 120  50 140]);

    add_block('simulink/Sinks/Out1',[s '/T_primary'],  'Port','1','Position',[760  40 790  60]);
    add_block('simulink/Sinks/Out1',[s '/T_secondary'],'Port','2','Position',[760 110 790 130]);
    add_block('simulink/Sinks/Out1',[s '/ratio'],      'Port','3','Position',[760 180 790 200]);
    add_block('simulink/Sinks/Out1',[s '/rp'],         'Port','4','Position',[760 250 790 270]);
    add_block('simulink/Sinks/Out1',[s '/diag'],       'Port','5','Position',[760 330 790 350]);

    % Shift state: primary pitch radius, hard-limited to the sheave travel.
    add_block('simulink/Continuous/Integrator',[s '/rp_state'], ...
        'InitialCondition','P.cvt.rp_min', ...
        'LimitOutput','on', ...
        'UpperSaturationLimit','P.cvt.rp_max', ...
        'LowerSaturationLimit','P.cvt.rp_min', ...
        'Position',[120 400 150 440]);

    local_mlfcn([s '/Geometry'], fullfile(here,'blockfcn','gx9_cvt_geom.m'),  [210 380 300 470]);
    local_mlfcn([s '/Forces'],   fullfile(here,'blockfcn','gx9_cvt_forces.m'),[380 230 480 360]);
    local_mlfcn([s '/Belt'],     fullfile(here,'blockfcn','gx9_belt.m'),      [540  40 630 130]);

    % The helix force depends on secondary torque, which depends on the belt
    % capacity, which depends back on the helix force.  Belt and sheave
    % compliance means the helix cannot in fact react instantaneously, so a
    % small first-order lag is both physical and enough to break the loop -
    % no algebraic loop solver, and no discrete state in a continuous model.
    add_block('simulink/Continuous/Transfer Fcn',[s '/TorqueSenseLag'], ...
        'Numerator','[1]','Denominator','[P.cvt.tau_T 1]', ...
        'Position',[360 165 440 215]);

    add_block('simulink/Math Operations/Product',[s '/IdealSecTorque'], ...
        'Inputs','2','Position',[660 150 690 190]);
    add_block('simulink/Math Operations/Abs',[s '/absT'], ...
        'Position',[540 250 570 280]);
    add_block('simulink/Lookup Tables/2-D Lookup Table',[s '/Efficiency_Map'], ...
        'BreakpointsForDimension1','P.cvt.eff_r', ...
        'BreakpointsForDimension2','P.cvt.eff_T', ...
        'Table','P.cvt.eff_tab', ...
        'Position',[600 290 660 350]);
    local_setopt([s '/Efficiency_Map'],'ExtrapMethod','Clip');
    add_block('simulink/Math Operations/Product',[s '/ApplyEfficiency'], ...
        'Inputs','2','Position',[710 110 740 150]);

    add_block('simulink/Signal Routing/Mux',[s '/DiagMux'], ...
        'Inputs','8','Position',[700 380 710 560]);
    add_block('simulink/Sinks/Terminator',[s '/theta_s_unused'], ...
        'Position',[330 520 350 540]);

    % geometry
    add_line(s,'rp_state/1','Geometry/1','autorouting','on');
    add_line(s,'Geometry/4','theta_s_unused/1','autorouting','on');   % theta_s

    % forces:  (w_e, T_sec, rp, xp, xs, lever, theta_p)
    add_line(s,'w_e/1',            'Forces/1','autorouting','on');
    add_line(s,'TorqueSenseLag/1', 'Forces/2','autorouting','on');
    add_line(s,'rp_state/1',       'Forces/3','autorouting','on');
    add_line(s,'Geometry/6',       'Forces/4','autorouting','on');    % xp
    add_line(s,'Geometry/7',       'Forces/5','autorouting','on');    % xs
    add_line(s,'Geometry/5',       'Forces/6','autorouting','on');    % lever
    add_line(s,'Geometry/3',       'Forces/7','autorouting','on');    % theta_p
    add_line(s,'Forces/4','rp_state/1','autorouting','on');           % rp_dot

    % belt: (w_e, w_sec, ratio, T_cap, engage)
    add_line(s,'w_e/1',      'Belt/1','autorouting','on');
    add_line(s,'w_sec/1',    'Belt/2','autorouting','on');
    add_line(s,'Geometry/2', 'Belt/3','autorouting','on');
    add_line(s,'Forces/5',   'Belt/4','autorouting','on');            % T_cap
    add_line(s,'Forces/6',   'Belt/5','autorouting','on');            % engage

    % secondary torque = primary torque * ratio * efficiency(ratio, |T|)
    add_line(s,'Belt/1',          'IdealSecTorque/1','autorouting','on');
    add_line(s,'Geometry/2',      'IdealSecTorque/2','autorouting','on');
    add_line(s,'IdealSecTorque/1','absT/1','autorouting','on');
    add_line(s,'Geometry/2',      'Efficiency_Map/1','autorouting','on');
    add_line(s,'absT/1',          'Efficiency_Map/2','autorouting','on');
    add_line(s,'IdealSecTorque/1','ApplyEfficiency/1','autorouting','on');
    add_line(s,'Efficiency_Map/1','ApplyEfficiency/2','autorouting','on');
    add_line(s,'ApplyEfficiency/1','TorqueSenseLag/1','autorouting','on');

    % outputs
    add_line(s,'Belt/1',           'T_primary/1','autorouting','on');
    add_line(s,'ApplyEfficiency/1','T_secondary/1','autorouting','on');
    add_line(s,'Geometry/2',       'ratio/1','autorouting','on');
    add_line(s,'rp_state/1',       'rp/1','autorouting','on');

    % diagnostics: rs, F_shift, Fp, Fs, T_cap, slip, engage, eta
    add_line(s,'Geometry/1',      'DiagMux/1','autorouting','on');
    add_line(s,'Forces/3',        'DiagMux/2','autorouting','on');
    add_line(s,'Forces/1',        'DiagMux/3','autorouting','on');
    add_line(s,'Forces/2',        'DiagMux/4','autorouting','on');
    add_line(s,'Forces/5',        'DiagMux/5','autorouting','on');
    add_line(s,'Belt/2',          'DiagMux/6','autorouting','on');
    add_line(s,'Forces/6',        'DiagMux/7','autorouting','on');
    add_line(s,'Efficiency_Map/1','DiagMux/8','autorouting','on');
    add_line(s,'DiagMux/1','diag/1','autorouting','on');
end

% =====================================================================
function local_driveline(mdl)
    s = [mdl '/Driveline'];
    local_newsub(s, [530 40 640 120]);

    add_block('simulink/Sources/In1',[s '/T_secondary'],'Port','1','Position',[20  40  50  60]);
    add_block('simulink/Sources/In1',[s '/w_wheel'],    'Port','2','Position',[20 140  50 160]);
    add_block('simulink/Sinks/Out1', [s '/F_demand'],   'Port','1','Position',[260  40 290  60]);
    add_block('simulink/Sinks/Out1', [s '/w_secondary'],'Port','2','Position',[260 140 290 160]);

    % Secondary torque -> wheel torque -> tractive force demand.
    add_block('simulink/Math Operations/Gain',[s '/ToTractiveForce'], ...
        'Gain','P.dl.i_gb*P.dl.eta_gb/P.veh.rt','Position',[120  35 200  65]);
    add_block('simulink/Math Operations/Gain',[s '/ToSecondarySpeed'], ...
        'Gain','P.dl.i_gb','Position',[120 135 200 165]);

    add_line(s,'T_secondary/1','ToTractiveForce/1','autorouting','on');
    add_line(s,'ToTractiveForce/1','F_demand/1','autorouting','on');
    add_line(s,'w_wheel/1','ToSecondarySpeed/1','autorouting','on');
    add_line(s,'ToSecondarySpeed/1','w_secondary/1','autorouting','on');
end

% =====================================================================
function local_vehicle(mdl, here)
    s = [mdl '/Vehicle_2D'];
    local_newsub(s, [330 400 460 540]);

    add_block('simulink/Sources/In1',[s '/F_demand'],'Port','1','Position',[20  40  50  60]);
    add_block('simulink/Sources/In1',[s '/grade'],   'Port','2','Position',[20 160  50 180]);

    add_block('simulink/Sinks/Out1',[s '/v'],       'Port','1','Position',[560  40 590  60]);
    add_block('simulink/Sinks/Out1',[s '/w_wheel'], 'Port','2','Position',[560 120 590 140]);
    add_block('simulink/Sinks/Out1',[s '/distance'],'Port','3','Position',[560 200 590 220]);
    add_block('simulink/Sinks/Out1',[s '/diag'],    'Port','4','Position',[560 300 590 320]);

    local_mlfcn([s '/Planar_Dynamics'], fullfile(here,'blockfcn','gx9_vehicle_2d.m'), [130 30 250 190]);

    add_block('simulink/Continuous/Integrator',[s '/v_state'], ...
        'InitialCondition','0', ...
        'LimitOutput','on','UpperSaturationLimit','inf','LowerSaturationLimit','0', ...
        'Position',[310  35 340  75]);
    add_block('simulink/Continuous/Integrator',[s '/distance_state'], ...
        'InitialCondition','0','Position',[400 195 430 235]);
    add_block('simulink/Math Operations/Gain',[s '/ToWheelSpeed'], ...
        'Gain','1/P.veh.rt','Position',[400 115 460 145]);
    add_block('simulink/Signal Routing/Mux',[s '/DiagMux'], ...
        'Inputs','6','Position',[470 290 480 430]);

    add_line(s,'F_demand/1','Planar_Dynamics/1','autorouting','on');
    add_line(s,'v_state/1', 'Planar_Dynamics/2','autorouting','on');
    add_line(s,'grade/1',   'Planar_Dynamics/3','autorouting','on');

    add_line(s,'Planar_Dynamics/1','v_state/1','autorouting','on');     % ax
    add_line(s,'v_state/1','ToWheelSpeed/1','autorouting','on');
    add_line(s,'v_state/1','distance_state/1','autorouting','on');

    add_line(s,'v_state/1','v/1','autorouting','on');
    add_line(s,'ToWheelSpeed/1','w_wheel/1','autorouting','on');
    add_line(s,'distance_state/1','distance/1','autorouting','on');

    % diagnostics: ax, F_trac, F_res, Nf, Nr, traction_limited
    add_line(s,'Planar_Dynamics/1','DiagMux/1','autorouting','on');
    add_line(s,'Planar_Dynamics/2','DiagMux/2','autorouting','on');
    add_line(s,'Planar_Dynamics/5','DiagMux/3','autorouting','on');
    add_line(s,'Planar_Dynamics/3','DiagMux/4','autorouting','on');
    add_line(s,'Planar_Dynamics/4','DiagMux/5','autorouting','on');
    add_line(s,'Planar_Dynamics/6','DiagMux/6','autorouting','on');
    add_line(s,'DiagMux/1','diag/1','autorouting','on');
end

% =====================================================================
function local_setopt(blkPath, varargin)
%LOCAL_SETOPT  Best-effort set_param for options whose accepted values have
%   changed across Simulink releases.  Clipped extrapolation matters here:
%   the efficiency map must never extrapolate above 1.0.
    try
        set_param(blkPath, varargin{:});
    catch err
        warning('gx9:setoptFailed','Could not set %s on %s (%s)', ...
                varargin{1}, blkPath, err.message);
    end
end

% =====================================================================
function local_newsub(blkPath, pos)
%LOCAL_NEWSUB  Create an empty subsystem, whatever the release's default is.
    add_block('built-in/Subsystem', blkPath, 'Position', pos);
    try
        inner = find_system(blkPath,'SearchDepth',1,'LookUnderMasks','all', ...
                            'FollowLinks','on');
        for k = 1:numel(inner)
            if ~strcmp(inner{k}, blkPath)
                delete_block(inner{k});
            end
        end
    catch
        % Nothing to clear - built-in/Subsystem comes up empty anyway.
    end
end

% =====================================================================
function local_mlfcn(blkPath, srcFile, pos)
%LOCAL_MLFCN  Drop in a MATLAB Function block from a source file on disk.
%   The parameter struct P is demoted from an input port to Parameter scope,
%   so it resolves from the model workspace instead of having to be wired
%   into every block.
    add_block('simulink/User-Defined Functions/MATLAB Function', blkPath, ...
        'Position', pos);

    chart = local_findchart(blkPath);
    chart.Script = fileread(srcFile);

    data = chart.find('-isa','Stateflow.Data');
    for k = 1:numel(data)
        if strcmp(data(k).Name,'P')
            data(k).Scope = 'Parameter';
        end
    end
end

% =====================================================================
function chart = local_findchart(blkPath)
%LOCAL_FINDCHART  Stateflow chart object behind a MATLAB Function block.
%   Stateflow reports Path as the PARENT system, so the full block path has
%   to be reassembled rather than matched against Path directly.
    S      = sfroot;
    charts = S.find('-isa','Stateflow.EMChart');
    for k = 1:numel(charts)
        if strcmp([charts(k).Path '/' charts(k).Name], blkPath)
            chart = charts(k);
            return
        end
    end
    error('gx9:chartNotFound','No MATLAB Function block found at %s', blkPath);
end
