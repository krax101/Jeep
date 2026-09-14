function [names, units] = gx9_sig_names()
%GX9_SIG_NAMES  Channel map of the logged signal bus from GX9_2D.
%
%   The model logs one wide vector ('sig') rather than a scatter of To
%   Workspace blocks.  This function owns the channel order; LogMux in
%   build_gx9_2d_model.m must match it.
%
%   See also GX9_POSTPROCESS.

    names = { ...
        'w_e'        % 1  engine / primary speed
        'T_e'        % 2  engine brake torque
        'T_pri'      % 3  torque through the belt at the primary
        'T_sec'      % 4  torque at the secondary, after belt losses
        'ratio'      % 5  CVT ratio, w_primary/w_secondary
        'rp'         % 6  primary pitch radius (the shift state)
        'rs'         % 7  secondary pitch radius
        'F_shift'    % 8  net shift force, +ve upshifts
        'Fp'         % 9  net primary axial force
        'Fs'         % 10 secondary axial force
        'T_cap'      % 11 belt friction torque capacity at the primary
        'slip_w'     % 12 belt slip speed, primary side
        'engage'     % 13 clutch engagement fraction, 0..1
        'eta_cvt'    % 14 belt efficiency from the loss map
        'v'          % 15 vehicle speed
        'dist'       % 16 distance travelled
        'ax'         % 17 longitudinal acceleration
        'F_trac'     % 18 tractive force at the rear contact patch
        'F_res'      % 19 aero + rolling + grade resistance
        'Nf'         % 20 front axle normal load
        'Nr'         % 21 rear axle normal load
        'tract_lim'  % 22 1 when rear grip is the limit
        };

    units = { ...
        'rad/s','N*m','N*m','N*m','-','m','m','N','N','N','N*m','rad/s', ...
        '-','-','m/s','m','m/s^2','N','N','N','N','-'};
end
