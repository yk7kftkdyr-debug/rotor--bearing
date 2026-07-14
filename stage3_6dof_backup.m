function [MM, CC, KK, modelInfo] = build_rotor_case_model(params)
%BUILD_ROTOR_CASE_MODEL Build global rotor-case matrices.
% Inputs:
%   params - setup structure from setup_rotor_bearing_input.
% Outputs:
%   MM, CC, KK - global mass, damping and stiffness matrices.
%   modelInfo  - indexing information for rotor and case DOFs.
%
% If params.use_uploaded_rotor_model is true, this function first tries to
% use the uploaded Newmark rotor-case files. The fallback builder keeps the
% same 4-DOF-per-node convention and runs without additional toolboxes.

use_uploaded = isfield(params, 'use_uploaded_rotor_model') && params.use_uploaded_rotor_model;
if use_uploaded
    model_dir = params.uploaded_model_dir;
    required_files = {'Mst_Msr_Ks_Ge.m','M_G_K.m','K_D.m','K_D_case.m', ...
        'rotor_parameters.m','case_parameters.m'};
    has_all = isfolder(model_dir);
    for k = 1:numel(required_files)
        has_all = has_all && isfile(fullfile(model_dir, required_files{k}));
    end

    if has_all
        addpath(model_dir);

        [N, density, Ef, L, R, RO, miu] = rotor_parameters();
        [N_C, density_C, Ef_C, L_C, R_C, RO_C, miu_C] = case_parameters();

        evalc('[Mst, Msr, Ks, Ge] = Mst_Msr_Ks_Ge(N, density, R, RO, L, Ef, miu);');
        evalc('[Mst_C, Msr_C, Ks_C, Ge_C] = Mst_Msr_Ks_Ge(N_C, density_C, R_C, RO_C, L_C, Ef_C, miu_C);');

        evalc('[M, G, K] = M_G_K(N, Ef, R, RO, Mst, Msr, Ge, Ks, miu, L);');
        evalc('[M_C, G_C, K_C] = M_G_K(N_C, Ef_C, R_C, RO_C, Mst_C, Msr_C, Ge_C, Ks_C, miu_C, L_C);');

        % is_coupled = 1 means the uploaded K_D does not add fixed linear
        % bearing support. Bearing coupling is supplied by nonlinear force.
        is_coupled = 1;
        evalc('[K, C] = K_D(N, K, M, G, params.omega, is_coupled);');
        evalc('[K_C, C_C] = K_D_case(N_C, K_C, M_C, G_C, is_coupled);');

        MM = blkdiag(M, M_C);
        CC = blkdiag(C, C_C);
        KK = blkdiag(K, K_C);

        ndR = size(M, 1);
        ndC = size(M_C, 1);
        modelInfo.num_rotor_nodes = N + 1;
        modelInfo.num_case_nodes = N_C + 1;
        modelInfo.num_rotor_dof = ndR;
        modelInfo.num_case_dof = ndC;
        modelInfo.rotor_trans_dof = reshape([1:4:ndR; 2:4:ndR], [], 1);
        modelInfo.case_trans_dof = ndR + reshape([1:4:ndC; 2:4:ndC], [], 1);
        modelInfo.dof_per_node = 4;
        modelInfo.source = 'uploaded Newmark rotor-case model';
        modelInfo.uploaded_model_dir = model_dir;
        modelInfo.omega = params.omega;
        modelInfo.rotor.N = N;
        modelInfo.rotor.L = L;
        modelInfo.rotor.R = R;
        modelInfo.rotor.RO = RO;
        modelInfo.case.N = N_C;
        modelInfo.case.L = L_C;
        return;
    else
        if isfield(params, 'require_uploaded_rotor_model') && params.require_uploaded_rotor_model
            error('Official run requires uploaded Newmark rotor-case files, but the required files were not found in: %s', model_dir);
        end
        warning('Uploaded rotor model files were not found. Fallback beam model is used.');
    end
end

node_pos = params.node_pos(:).';
case_pos = params.case_node_pos(:).';
nR = numel(node_pos);
nC = numel(case_pos);
ndR = 4*nR;
ndC = 4*nC;

MR = zeros(ndR); KR = zeros(ndR);
MC = zeros(ndC); KC = zeros(ndC);

A = pi/4*(params.shaft_od^2 - params.shaft_id^2);
I = pi/64*(params.shaft_od^4 - params.shaft_id^4);
shaft_mass_rho = get_field_default(params, 'shaft_mass_rho', params.rho);
[MR, KR] = assemble_beam_line(MR, KR, node_pos, params.E, I, shaft_mass_rho, A);

Ac = pi/4*(params.case_od^2 - params.case_id^2);
Ic = pi/64*(params.case_od^4 - params.case_id^4);
case_mass_rho = get_field_default(params, 'case_mass_rho', params.case_rho);
[MC, KC] = assemble_beam_line(MC, KC, case_pos, params.case_E, Ic, case_mass_rho, Ac);

% Add only the specified translational masses; disk-region beam stiffness is
% already present in KR and no legacy distributed disk masses are retained.
if isfield(params, 'concentrated_mass')
    for k = 1:numel(params.concentrated_mass.nodes)
        MR = add_translational_mass(MR, params.concentrated_mass.nodes(k), params.concentrated_mass.kg(k));
    end
end
if isfield(params, 'disk_mass')
    MR = add_translational_mass(MR, params.disk_mass.node, params.disk_mass.kg);
end

% Foundation support acts only at C1 and C13.
ground_nodes = get_field_default(params, 'case_ground_nodes', [1 nC]);
for i = ground_nodes
    ix = 4*i - 3;
    iy = 4*i - 2;
    KC(ix,ix) = KC(ix,ix) + params.case_ground_k;
    KC(iy,iy) = KC(iy,iy) + params.case_ground_k;
end

MM = blkdiag(MR, MC);
KK = blkdiag(KR, KC);
CC = params.rayleigh_alpha*MM + params.rayleigh_beta*KK;
for i = ground_nodes
    ix = ndR + 4*i - 3;
    iy = ndR + 4*i - 2;
    CC(ix,ix) = CC(ix,ix) + params.case_ground_c;
    CC(iy,iy) = CC(iy,iy) + params.case_ground_c;
end

% Small rotational grounding removes rigid numerical drift in fallback mode.
rot_k = 1.0e2;
for i = 1:nR
    KK(4*i-1,4*i-1) = KK(4*i-1,4*i-1) + rot_k;
    KK(4*i,4*i) = KK(4*i,4*i) + rot_k;
end

modelInfo.num_rotor_nodes = nR;
modelInfo.num_case_nodes = nC;
modelInfo.num_rotor_dof = ndR;
modelInfo.num_case_dof = ndC;
modelInfo.rotor_trans_dof = reshape([1:4:ndR; 2:4:ndR], [], 1);
modelInfo.case_trans_dof = ndR + reshape([1:4:ndC; 2:4:ndC], [], 1);
modelInfo.dof_per_node = 4;
modelInfo.source = 'internal fallback beam model';
modelInfo.uploaded_model_required = false;
modelInfo.mass_components.shaft_N = get_field_default(get_field_default(params, 'mass_target', struct()), 'shaft_N', NaN);
modelInfo.mass_components.concentrated_N = get_field_default(get_field_default(params, 'mass_target', struct()), 'concentrated_N', NaN);
modelInfo.mass_components.disk_N = get_field_default(get_field_default(params, 'mass_target', struct()), 'disk_N', NaN);
modelInfo.mass_components.case_N = get_field_default(get_field_default(params, 'mass_target', struct()), 'case_N', NaN);

end

function M = add_translational_mass(M, node, mass)
if mass < 0, error('Concentrated mass must be nonnegative.'); end
dofs = 4*node + (-3:-2);
M(dofs,dofs) = M(dofs,dofs) + mass*eye(2);
end

function v = get_field_default(s, field, default_value)
if isfield(s, field) && ~isempty(s.(field)), v = s.(field); else, v = default_value; end
end

function [M, K] = assemble_beam_line(M, K, x, E, I, rho, A)
for e = 1:(numel(x)-1)
    Le = x(e+1) - x(e);
    if Le <= 0
        error('Node positions must be strictly increasing.');
    end
    [me, ke] = beam2d_mk(E, I, rho, A, Le);
    dx = [4*e-3, 4*e-1, 4*(e+1)-3, 4*(e+1)-1];
    dy = [4*e-2, 4*e,   4*(e+1)-2, 4*(e+1)];
    M(dx,dx) = M(dx,dx) + me;
    K(dx,dx) = K(dx,dx) + ke;
    M(dy,dy) = M(dy,dy) + me;
    K(dy,dy) = K(dy,dy) + ke;
end
end

function [m, k] = beam2d_mk(E, I, rho, A, L)
% Euler-Bernoulli bending element used as a compact fallback.
k = E*I/L^3 * [ ...
    12,    6*L,   -12,    6*L;
    6*L,   4*L^2, -6*L,   2*L^2;
   -12,   -6*L,    12,   -6*L;
    6*L,   2*L^2, -6*L,   4*L^2];
m = rho*A*L/420 * [ ...
    156,    22*L,   54,    -13*L;
    22*L,   4*L^2,  13*L,  -3*L^2;
    54,     13*L,   156,   -22*L;
   -13*L,  -3*L^2, -22*L,   4*L^2];
end
