function [Fb_global, bearingState] = F_bearing(q, qd, params, N_dof)
%F_BEARING Validation-only shim for the frozen Stage4-B1 contact entry.

[Fb_global, bearingState] = nonlinear_bearing_force_stage4B1_frozen( ...
    q, qd, params, params.bearing, N_dof);
end
