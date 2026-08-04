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
rcond_Z = rcond(Z);
warning_state = warning; cleanup = onCleanup(@() warning(warning_state));
warning('on','all');
lastwarn('');
solver_output = evalc('qhat_full = Z\Fhat(:);');
[warning_message,warning_identifier] = lastwarn;
finite_solution = all(isfinite(qhat_full));
if finite_solution
    relative_residual = norm(Z*qhat_full-Fhat(:))/max(norm(Fhat(:)),eps);
else
    relative_residual = Inf;
end
norm_qhat_2 = norm(qhat_full,2);
norm_qhat_inf = norm(qhat_full,inf);
if ~finite_solution
    failure_gate = 'FREQUENCY_RESPONSE_NONFINITE_FAILED';
elseif ~isfinite(rcond_Z) || rcond_Z < 1e-14
    failure_gate = 'SEVERELY_ILL_CONDITIONED_DYNAMIC_STIFFNESS';
elseif ~(relative_residual <= 1e-8)
    failure_gate = 'FREQUENCY_RESPONSE_RESIDUAL_FAILED';
elseif ~(norm_qhat_inf > 100*eps)
    failure_gate = 'FREQUENCY_RESPONSE_NEAR_ZERO_FAILED';
else
    failure_gate = '';
end
accepted = isempty(failure_gate);
if rcond_Z < 1e-14
    conditioning_flag = 'SEVERELY_ILL_CONDITIONED_DYNAMIC_STIFFNESS';
elseif rcond_Z < 1e-10
    conditioning_flag = 'ILL_CONDITIONED_DYNAMIC_STIFFNESS';
else
    conditioning_flag = 'WELL_CONDITIONED';
end

response = struct('qhat_full',qhat_full, ...
    'rcond_Z',rcond_Z,'relative_residual',relative_residual, ...
    'finite_solution',finite_solution,'norm_qhat_2',norm_qhat_2, ...
    'norm_qhat_inf',norm_qhat_inf, ...
    'conditioning_flag',conditioning_flag,'accepted',accepted, ...
    'failure_gate',failure_gate, ...
    'solver','backslash','solver_warning',struct('message',warning_message, ...
    'identifier',warning_identifier,'output',solver_output), ...
    'gyroscopic_sign_convention','Mddot_plus_C_plus_OmegaG_qdot_plus_Kq', ...
    'gyroscopic_matrix_symmetrized',false,'omega_exc_rad_s',omega, ...
    'Omega_rotor_rad_s',Omega);
if isfield(case_info,'temperature_case_C'), response.temperature_case_C = case_info.temperature_case_C; end
if isfield(case_info,'scenario'), response.scenario = case_info.scenario; end
if isfield(case_info,'gyroscopic_sign_convention')
    response.gyroscopic_sign_convention = case_info.gyroscopic_sign_convention;
end
end
