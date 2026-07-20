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
micro_cfg = microphysics_config(get_field_default(params, 'microphysics', struct()));

for ib = 1:nb
    brg = bearing(ib);
    if isfield(params, 'contact')
        brg.contact = params.contact;
    end
    local = bearing_relative_state(q, qd, brg, num_rotor);
    if isfield(params,'stage4A') && get_field_default(params.stage4A,'enable',false)
        contact_base = build_base_contact_state(local, brg, t);
        operating_state = struct('time', t, 'bearing_index', ib, 'omega', params.omega, 'E_star', params.E/(2*(1-params.nu^2)));
        operating_state.evaluate_raw_contact = ...
            @(contact_trial) evaluate_raw_contact(contact_trial, local, brg, t);
        [contact_mod, micro_state] = apply_microphysics(contact_base, operating_state, struct(), micro_cfg);
        [f5, state, raw_contact] = solve_contact_force(contact_mod, operating_state);
        state = stage4a_normalize_state(state);
        state.raw_contact = raw_contact;
        state.microphysics_state = micro_state;
        if micro_state.temperature.enabled
            [film, film_state] = update_contact_film_from_load(raw_contact.Q, micro_state.temperature.contact_geometry, micro_state.temperature.input.eta, micro_state.temperature.input.alpha_p);
            state.h = film.h_m; state.film = film; state.film_state = film_state;
            state.raw_contact.film = film;
            state.microphysics_state.temperature.input.film_thickness = film.hmin_m;
        end
        Fb_global = assemble_bearing_force(Fb_global, f5, brg.rotor_node, brg.case_node, num_rotor);
        state.Fx=f5(1); state.Fy=f5(2); state.Fz=f5(3); state.Mx=f5(4); state.My=f5(5); state.r=local.r; state.rdot=local.rdot; state.rotor_node=brg.rotor_node; state.case_node=brg.case_node; state.name=brg.name; state.bearing_model_stage='5DOF_static_interface';
        if ib == 1, bearingState.bearings=state; else, bearingState.bearings(ib)=state; end
        continue;
    end
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

function contact_base = build_base_contact_state(local, brg, t)
%BUILD_BASE_CONTACT_STATE Local-only contact record for microphysics modules.
contact_base = struct('local', local, 'brg', brg, 'time', t, ...
    'viscosity', [], 'pressure_viscosity', [], ...
    'working_clearance', get_field_default(brg.assembly, 'radial_clearance', 0), ...
    'film_thickness', [], 'surface_height', [], ...
    'effective_deformation', [], 'asperity_contact_ratio', [], ...
    'contact_stiffness', contact_stiffness_value(brg), ...
    'contact_damping', [], 'characteristic_displacement', []);
end

function [f5, state, raw_contact] = solve_contact_force(contact_mod, operating_state)
%SOLVE_CONTACT_FORCE Extract force and state from the one raw contact kernel.
raw_contact = operating_state.evaluate_raw_contact(contact_mod);
f5 = raw_contact.f5;
state = raw_contact.state;
end

function result = evaluate_raw_contact(contact_trial, local, brg, t)
brg = apply_frozen_contact_state(brg, contact_trial);
[f5, state] = stage4a_bearing_force(local, brg, [], t);
result = struct('f5', f5, 'Q', state.Q, 'delta', state.delta, ...
    'delta_raw', state.delta_raw, 'loaded', state.Q > 0, ...
    'loaded_count', state.loaded_count, 'element_angle', state.theta, ...
    'contact_angle', state.contact_angle, 'normal_direction', [], ...
    'contact_position', [], 'slice_z', [], 'state', state);
if isfield(state, 'slice_z'), result.slice_z = state.slice_z; end
end

function brg = apply_frozen_contact_state(brg, contact_trial)
%APPLY_FROZEN_CONTACT_STATE Maps only local thermal inputs into the existing kernel.
if ~isempty(contact_trial.viscosity), brg.oil_viscosity = contact_trial.viscosity; end
if ~isempty(contact_trial.pressure_viscosity), brg.pressure_viscosity = contact_trial.pressure_viscosity; end
if ~isempty(contact_trial.working_clearance), brg.assembly.radial_clearance = contact_trial.working_clearance; end
end

function value = contact_stiffness_value(brg)
if strcmpi(brg.type, 'ball')
    value = get_field_default(brg, 'K_point', 0);
else
    value = get_field_default(brg, 'K_line', 0);
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
% Relative five-DOF state: [ux uy uz theta_x theta_y].
ir=6*brg.rotor_node+(-5:-1); ic=num_rotor+6*brg.case_node+(-5:-1); local.r=q(ir)-q(ic); local.rdot=qd(ir)-qd(ic);
local.xr=local.r(1); local.yr=local.r(2); local.vxr=local.rdot(1); local.vyr=local.rdot(2);
end

function [f,state] = stage4a_bearing_force(local, brg, ~, t)
if strcmpi(brg.type,'ball'), [f,state]=stage4a_ball_force(local,brg,[],t); else, [f,state]=stage4a_roller_force(local,brg,[],t); end
end

function [f,state] = stage4a_ball_force(local,b,~,t)
n=b.n; theta=2*pi*(0:n-1)/n; r=local.r; a0=b.assembly.contact_angle0; a=max(1*pi/180,min(b.assembly.contact_angle_max,a0+r(3)/max(b.Dm/2,eps))); c=b.assembly.radial_clearance; z=b.stage4A_width_m/2; Q=zeros(1,n); delta=zeros(1,n); f=zeros(5,1);
for j=1:n
    cj=cos(theta(j)); sj=sin(theta(j)); x_contact=r(1)+z*r(5); y_contact=r(2)-z*r(4); delta(j)=x_contact*cj+y_contact*sj+(r(3)+b.assembly.preload_displacement)*sin(a)-c;
    if delta(j)>0
        Q(j)=b.K_point*delta(j)^(3/2); fj=-Q(j)*[cos(a)*cj;cos(a)*sj;sin(a)]; f(1:3)=f(1:3)+fj; f(4)=f(4)-z*fj(2); f(5)=f(5)+z*fj(1);
    end
end
state.theta=theta; state.Q=Q; state.delta=max(delta,0); state.delta_raw=delta; state.loaded_count=sum(Q>0); state.loaded_index=find(Q>0); state.max_contact_load=max(Q); state.contact_angle=a; state.c_work=c; state.preload_status=b.assembly.preload_mode; state.message=stage4a_warning(b); state.slice_count=1; state.time=t;
end

function [f,state] = stage4a_roller_force(local,b,~,t)
n=b.n; ns=b.stage4A_slice_count; theta=2*pi*(0:n-1)/n; zs=linspace(-b.L/2,b.L/2,ns); r=local.r; c=b.assembly.radial_clearance; Q=zeros(n,ns); delta=zeros(n,ns); f=zeros(5,1);
for j=1:n
    cj=cos(theta(j)); sj=sin(theta(j));
    for k=1:ns
        z=zs(k); delta(j,k)=r(1)*cj+r(2)*sj+z*(r(5)*cj-r(4)*sj)-c;
        if delta(j,k)>0
            Q(j,k)=(b.K_line/ns)*delta(j,k)^(10/9); fj=-Q(j,k)*[cj;sj]; f(1:2)=f(1:2)+fj; f(4)=f(4)-z*fj(2); f(5)=f(5)+z*fj(1);
        end
    end
end
state.theta=theta; state.slice_z=zs; state.Q=Q; state.delta=max(delta,0); state.delta_raw=delta; state.loaded_count=sum(any(Q>0,2)); state.loaded_index=find(any(Q>0,2)); state.max_contact_load=max(Q,[],'all'); state.contact_angle=0; state.c_work=c; state.preload_status=b.assembly.preload_mode; state.message='rear floating: Fz fixed to zero; roller crowning unknown'; state.slice_count=ns; state.slice_validation_count=b.stage4A_slice_validation_count; state.time=t;
end

function msg = stage4a_warning(b)
if b.assembly.preload_displacement==0, msg='assembly preload not calibrated; axial locating load path not calibrated'; else, msg='assembly preload displacement supplied'; end
end

function out = stage4a_normalize_state(in)
out=struct('theta',[],'slice_z',[],'Q',[],'delta',[],'delta_raw',[],'loaded_count',0,'loaded_index',[],'max_contact_load',0,'contact_angle',0,'c_work',0,'preload_status','','message','','slice_count',0,'slice_validation_count',0,'time',0);
names=fieldnames(out); for k=1:numel(names), if isfield(in,names{k}), out.(names{k})=in.(names{k}); end, end
end
