function [Fb_global, bearingState] = nonlinear_bearing_force(q, qd, params, bearing, MM_size)
%NONLINEAR_BEARING_FORCE Real-time nonlinear bearing force assembly.
% Inputs:
%   q, qd   - current global displacement and velocity.
%   params  - global parameter structure.
%   bearing - bearing parameter structure array.
%   MM_size - global DOF count.
% Outputs:
%   Fb_global    - assembled global bearing force vector.
%   bearingState - contact state for every bearing.

if nargin < 4 || isempty(bearing)
    bearing = params.bearing;
end
if nargin < 5 || isempty(MM_size)
    MM_size = numel(q);
end

Fb_global = zeros(MM_size, 1);
nb = numel(bearing);
bearingState.bearings = struct([]);

num_rotor = params.modelInfo.num_rotor_dof;
t = 0;
if isfield(params, 'current_time')
    t = params.current_time;
end
model_case = 'full_tribology';
if isfield(params, 'model_case') && ~isempty(params.model_case)
    model_case = lower(params.model_case);
end

for ib = 1:nb
    brg = bearing(ib);
    if isfield(params, 'contact')
        brg.contact = params.contact;
    end
    local = bearing_relative_state(q, qd, brg, num_rotor);
    xr = local.xr; yr = local.yr; vxr = local.vxr; vyr = local.vyr;
    opx = get_field_default(brg, 'operating_offset_x', 0);
    opy = get_field_default(brg, 'operating_offset_y', 0);
    xr_eff = xr + opx;
    yr_eff = yr + opy;

    switch model_case
        case 'linear'
            [Fx, Fy, state] = linear_bearing_force(xr, yr, vxr, vyr, brg);
        case 'fixed_nonlinear'
            [Fx, Fy, state] = bearing_force_by_type(xr_eff, yr_eff, vxr, vyr, brg, params.omega, 0);
            state.model_case = model_case;
        otherwise
            [Fx, Fy, state] = bearing_force_by_type(xr_eff, yr_eff, vxr, vyr, brg, params.omega, t);
            state.model_case = model_case;
    end

    if get_field_default(brg, 'subtract_preload_baseline', false) && ~strcmp(model_case, 'linear')
        [Fx0, Fy0] = bearing_force_by_type(opx, opy, 0, 0, brg, params.omega, 0);
        Fx = Fx - Fx0;
        Fy = Fy - Fy0;
    end

    Fb_global = assemble_bearing_force(Fb_global, Fx, Fy, brg.rotor_node, brg.case_node, num_rotor);

    state.Fx = Fx;
    state.Fy = Fy;
    state.xr = xr;
    state.yr = yr;
    state.vxr = vxr;
    state.vyr = vyr;
    state.xr_eff = xr_eff;
    state.yr_eff = yr_eff;
    state.operating_offset_x = opx;
    state.operating_offset_y = opy;
    state.rotor_node = brg.rotor_node;
    state.case_node = brg.case_node;
    state.name = brg.name;
    state.bearing_model_stage = get_field_default(params, 'bearing_model_stage', '2D_force_on_6DOF_structure');
    if ib == 1
        bearingState.bearings = state;
    else
        bearingState.bearings(ib) = state;
    end
end

end

function [Fx, Fy, state] = bearing_force_by_type(xr, yr, vxr, vyr, brg, omega, t)
switch lower(brg.type)
    case 'roller'
        [Fx, Fy, state] = roller_bearing_force(xr, yr, vxr, vyr, brg, omega, t);
    case 'ball'
        [Fx, Fy, state] = ball_bearing_force(xr, yr, vxr, vyr, brg, omega, t);
    otherwise
        error('Unknown bearing type: %s', brg.type);
end
end

function [Fx, Fy, state] = linear_bearing_force(xr, yr, vxr, vyr, bearing)
k = get_field_default(bearing, 'linear_k', get_field_default(bearing, 'oil_stiffness', 1e7));
c = get_field_default(bearing, 'linear_c', get_field_default(bearing, 'oil_damping', 1e3));
Fx = -k*xr - c*vxr;
Fy = -k*yr - c*vyr;
state.theta = [];
state.delta = [];
state.Q = [];
state.h = [];
state.PV = [];
state.mu_i = [];
state.loaded_count = 0;
state.loaded_index = [];
state.sliding_speed = [];
state.slip_ratio = 0;
state.cage_speed_ideal = 0;
state.cage_speed_estimated = 0;
state.ball_spin_speed = NaN;
state.centrifugal_force = 0;
state.c_work = get_field_default(bearing, 'clearance0', 0);
state.clearInfo = struct();
state.model_case = 'linear';
end

function v = get_field_default(s, field, default_value)
if isfield(s, field) && ~isempty(s.(field))
    v = s.(field);
else
    v = default_value;
end
end

function local = bearing_relative_state(q, qd, brg, num_rotor)
% Relative transverse state: rotor minus casing at the bearing seat.
irx = 6*brg.rotor_node - 5; iry = irx + 1;
icx = num_rotor + 6*brg.case_node - 5; icy = icx + 1;
local.xr = q(irx) - q(icx); local.yr = q(iry) - q(icy);
local.vxr = qd(irx) - qd(icx); local.vyr = qd(iry) - qd(icy);
end
