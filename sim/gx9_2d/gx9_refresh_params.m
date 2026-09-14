function P = gx9_refresh_params(mdl)
%GX9_REFRESH_PARAMS  Push edited gx9_params.json into a loaded model.
%
%   P = GX9_REFRESH_PARAMS()         refreshes GX9_2D
%   P = GX9_REFRESH_PARAMS('MyMdl')  refreshes another copy
%
%   The model workspace holds a snapshot of the parameter struct, so editing
%   gx9_params.json alone does not change what the model runs.  Call this
%   after an edit, or rebuild with BUILD_GX9_2D_MODEL if you changed anything
%   the block wiring depends on.

    if nargin < 1, mdl = 'GX9_2D'; end
    here = fileparts(mfilename('fullpath'));
    if ~bdIsLoaded(mdl)
        load_system(fullfile(here,[mdl '.slx']));
    end
    P = gx9_params(fullfile(here,'gx9_params.json'));
    assignin(get_param(mdl,'ModelWorkspace'),'P',P);
    fprintf('Refreshed P in %s (ratio %.2f:1 -> %.2f:1, engagement %.0f rpm)\n', ...
            mdl, P.cvt.ratio_low, P.cvt.ratio_high, P.cvt.rpm_engage);
end
