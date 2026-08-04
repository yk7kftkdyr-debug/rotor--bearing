function response = solve_stage_e_full_order_1X_case(M,G,K,C,Fhat,Omega,omega,case_info)
%SOLVE_STAGE_E_FULL_ORDER_1X_CASE Solve the unregularized full-order 1X system.

if nargin < 8 || isempty(case_info), case_info = struct(); end
n = size(M,1);
matrices = {M,G,K,C};
if ~isscalar(Omega) || ~isscalar(omega) || ~isfinite(Omega) || ~isfinite(omega) || ...
        n == 0 || any(cellfun(@(A) ~isnumeric(A) || ~isequal(size(A),[n n]) || ...
        any(~isfinite(A),'all'),matrices)) || ~isequal(size(Fhat(:)),[n 1]) || ...
        any(~isfinite(Fhat(:)))
    error('StageE:DynamicSystem','Inputs must define finite square full-order matrices and force.');
end
Z = K-omega^2*M+1i*omega*(C+Omega*G);
warning_state = warning; cleanup = onCleanup(@() warning(warning_state));
warning('on','all');
lastwarn('');
Fhat = Fhat(:);
DZ = [];
qhat_full = [];
solver_output = evalc("DZ = decomposition(Z,'lu'); qhat_full = DZ\Fhat;");
[warning_message,warning_identifier] = lastwarn;
solver_warning_record = struct('identifier',warning_identifier, ...
    'message',warning_message);
cfg = build_equivalent_damping_frequency_domain_config();
options = cfg.stage_e.solve_quality;
options.omega_source_match = true;
options.Omega_source_match = true;
options.solver_validator_Z_relative_difference = 0;
options.solver_validator_diagnostic_relative_difference = 0;
quality = evaluate_stage_e_linear_solve_quality( ...
    Z,Fhat,qhat_full,DZ,options,solver_warning_record);
qhat_full = quality.qhat_full;
norm_qhat_2 = norm(qhat_full,2);
norm_qhat_inf = norm(qhat_full,inf);
if quality.accepted
    failure_gate = '';
else
    failure_gate = quality.solve_quality_gate;
end
if quality.rcond_Z < options.rcond_hard_floor
    conditioning_flag = 'SEVERELY_ILL_CONDITIONED_DYNAMIC_STIFFNESS';
elseif quality.rcond_Z < 1e-10
    conditioning_flag = 'ILL_CONDITIONED_DYNAMIC_STIFFNESS';
else
    conditioning_flag = 'WELL_CONDITIONED';
end

response = quality;
response.normwise_backward_error_inf = quality.normwise_backward_error_inf;
response.rhs_relative_residual = quality.rhs_relative_residual;
response.relative_residual = quality.relative_residual;
response.norm_qhat_2 = norm_qhat_2;
response.norm_qhat_inf = norm_qhat_inf;
response.conditioning_flag = conditioning_flag;
response.failure_gate = failure_gate;
response.solver = 'backslash';
response.solver_warning = struct('message',warning_message, ...
    'identifier',warning_identifier,'output',solver_output);
response.gyroscopic_sign_convention = ...
    'Mddot_plus_C_plus_OmegaG_qdot_plus_Kq';
response.gyroscopic_matrix_symmetrized = false;
response.omega_exc_rad_s = omega;
response.Omega_rotor_rad_s = Omega;
if isfield(case_info,'temperature_case_C'), response.temperature_case_C = case_info.temperature_case_C; end
if isfield(case_info,'scenario'), response.scenario = case_info.scenario; end
if isfield(case_info,'gyroscopic_sign_convention')
    response.gyroscopic_sign_convention = case_info.gyroscopic_sign_convention;
end
end
