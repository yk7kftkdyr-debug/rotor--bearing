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
[MR, KR] = assemble_beam_line(MR, KR, node_pos, params.E, I, params.rho, A);

Ac = pi/4*(params.case_od^2 - params.case_id^2);
Ic = pi/64*(params.case_od^4 - params.case_id^4);
[MC, KC] = assemble_beam_line(MC, KC, case_pos, params.case_E, Ic, params.case_rho, Ac);

% Add discs as concentrated mass and rotary inertia.
for k = 1:numel(params.disc_nodes)
    nd = params.disc_nodes(k);
    od = params.disc_od(k);
    id = params.disc_id(k);
    th = params.disc_thick(k);
    md = params.rho*pi/4*(od^2 - id^2)*th;
    Jd = md/12*(3*((od/2)^2 + (id/2)^2) + th^2);
    dofs = 4*nd + (-3:0);
    MR(dofs(1),dofs(1)) = MR(dofs(1),dofs(1)) + md;
    MR(dofs(2),dofs(2)) = MR(dofs(2),dofs(2)) + md;
    MR(dofs(3),dofs(3)) = MR(dofs(3),dofs(3)) + Jd;
    MR(dofs(4),dofs(4)) = MR(dofs(4),dofs(4)) + Jd;
end

% Ground support for case nodes in the fallback model.
for i = 1:nC
    ix = 4*i - 3;
    iy = 4*i - 2;
    KC(ix,ix) = KC(ix,ix) + params.case_ground_k;
    KC(iy,iy) = KC(iy,iy) + params.case_ground_k;
end

MM = blkdiag(MR, MC);
KK = blkdiag(KR, KC);
CC = params.rayleigh_alpha*MM + params.rayleigh_beta*KK;
for i = 1:nC
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
