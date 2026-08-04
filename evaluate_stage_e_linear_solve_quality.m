function quality = evaluate_stage_e_linear_solve_quality( ...
        Z,Fhat,qhat,DZ,options,solver_warning_record)
%EVALUATE_STAGE_E_LINEAR_SOLVE_QUALITY Audit and, if needed, refine one solve.

Fhat = Fhat(:);
qhat = qhat(:);
history = repmat(struct('backward_error_before',NaN, ...
    'backward_error_after',NaN,'correction_relative_norm',NaN),0,1);

solver_warning_id = char(solver_warning_record.identifier);
solver_warning_message = char(solver_warning_record.message);
caution_flags = {};
if strcmp(solver_warning_id,'MATLAB:nearlySingularMatrix')
    caution_flags{end+1} = 'NEARLY_SINGULAR_SOLVER_WARNING';
end
rank_warning = any(strcmp(solver_warning_id, ...
    {'MATLAB:singularMatrix','MATLAB:rankDeficientMatrix'}));
finite_Z = all(isfinite(Z),'all');
finite_Fhat = all(isfinite(Fhat));
finite_solution = all(isfinite(qhat));
finite_system = finite_Z && finite_Fhat && finite_solution;
if finite_Z
    rcond_Z = rcond(Z);
else
    rcond_Z = NaN;
end
if finite_system
    normwise_backward_error_inf = backward_error_inf(Z,Fhat,qhat);
else
    normwise_backward_error_inf = Inf;
end

refinement_allowed = ~rank_warning && finite_system && isfinite(rcond_Z) && ...
    rcond_Z >= options.rcond_hard_floor && ...
    isfinite(normwise_backward_error_inf) && ...
    normwise_backward_error_inf > options.backward_error_hard_limit;
if refinement_allowed
    for step = 1:options.max_iterative_refinement_steps
        eta_before = backward_error_inf(Z,Fhat,qhat);
        correction = DZ\(Fhat-Z*qhat);
        candidate = qhat+correction;
        eta_after = backward_error_inf(Z,Fhat,candidate);
        correction_relative_norm = norm(correction,inf)/ ...
            max(norm(candidate,inf),realmin('double'));
        if ~(isfinite(eta_after) && eta_after < eta_before)
            break;
        end
        history(end+1,1) = struct( ...
            'backward_error_before',eta_before, ...
            'backward_error_after',eta_after, ...
            'correction_relative_norm',correction_relative_norm); %#ok<AGROW>
        qhat = candidate;
        if eta_after <= options.backward_error_hard_limit
            break;
        end
    end
end

if finite_system
    residual = Z*qhat-Fhat;
    rhs_relative_residual = norm(residual,2)/ ...
        max(norm(Fhat,2),realmin('double'));
    normwise_backward_error_inf = backward_error_inf(Z,Fhat,qhat);
else
    rhs_relative_residual = Inf;
end

if rank_warning
    solve_quality_gate = 'FREQUENCY_RESPONSE_RANK_FAILED';
    accepted = false;
elseif ~finite_system
    solve_quality_gate = 'FREQUENCY_RESPONSE_NONFINITE_FAILED';
    accepted = false;
elseif ~isfinite(rcond_Z) || rcond_Z < options.rcond_hard_floor
    solve_quality_gate = 'SEVERELY_ILL_CONDITIONED_DYNAMIC_STIFFNESS';
    accepted = false;
elseif ~isfinite(normwise_backward_error_inf) || ...
        normwise_backward_error_inf > options.backward_error_hard_limit
    solve_quality_gate = 'FREQUENCY_RESPONSE_BACKWARD_ERROR_FAILED';
    accepted = false;
elseif ~(norm(qhat,inf) > 100*eps)
    solve_quality_gate = 'FREQUENCY_RESPONSE_NEAR_ZERO_FAILED';
    accepted = false;
elseif rhs_relative_residual > options.rhs_residual_caution_limit
    solve_quality_gate = 'RHS_RELATIVE_RESIDUAL_CAUTION';
    caution_flags{end+1} = 'RHS_RELATIVE_RESIDUAL_CAUTION';
    accepted = true;
else
    solve_quality_gate = 'ACCEPTED';
    accepted = true;
end

if isempty(history)
    correction_relative_norm = [];
else
    correction_relative_norm = history(end).correction_relative_norm;
end
quality = struct( ...
    'qhat_full',qhat, ...
    'rhs_relative_residual',rhs_relative_residual, ...
    'relative_residual',rhs_relative_residual, ...
    'relative_residual_definition','norm(Z*q-F,2)/norm(F,2)', ...
    'normwise_backward_error_inf',normwise_backward_error_inf, ...
    'rcond_Z',rcond_Z, ...
    'finite_solution',finite_solution, ...
    'accepted',accepted, ...
    'solve_quality_gate',solve_quality_gate, ...
    'caution_flags',{caution_flags}, ...
    'refinement_history',history, ...
    'refinement_step_count',numel(history), ...
    'correction_relative_norm',correction_relative_norm, ...
    'solver_warning_id',solver_warning_id, ...
    'solver_warning_message',solver_warning_message, ...
    'omega_source_match',logical(options.omega_source_match), ...
    'Omega_source_match',logical(options.Omega_source_match), ...
    'solver_validator_Z_relative_difference', ...
        options.solver_validator_Z_relative_difference, ...
    'solver_validator_diagnostic_relative_difference', ...
        options.solver_validator_diagnostic_relative_difference);
end

function eta = backward_error_inf(Z,Fhat,qhat)
residual = Z*qhat-Fhat;
denominator = norm(Z,inf)*norm(qhat,inf)+norm(Fhat,inf);
eta = norm(residual,inf)/max(denominator,realmin('double'));
end
