function reference_state = extract_dynamic_ehl_reference_state_20C()
%EXTRACT_DYNAMIC_EHL_REFERENCE_STATE_20C Get a formal 20 C local reference.
%
% This isolated entry reuses the formal 20 C thermal-mechanical outer loop,
% including its existing thermal-property update, working-clearance update,
% oil-film update, adaptive-load Newton-KKT equilibrium, line search, and
% convergence gates.  It returns immediately after the final converged
% front-ball contact state is available.  No linearization, EHL damping
% matrix, modal, Newmark, free-decay, postprocessing, or file export is run.
%
% This is a manual diagnostic only.  It is not called by the D2 sample-input
% framework and is not read by any formal program.  A full thermal-consistent
% extraction can exceed 420 s; D2 pilot sampling instead reads the frozen
% cfg.pilot_reference_20C domain.  Formal D4/D5 work must obtain the actual
% runtime contact state again.

soft_warning_s = 240;
hard_timeout_s = 420;
started = tic;

params = initial_conditions();
try
    thermal_result = solve_thermal_mechanical_outer_loop(params, params.bearing, 20, []);
catch ME
    if startsWith(string(ME.identifier), "FrozenThermalStatic:")
        error('DynamicEHLReference:StaticNotConverged', 'REFERENCE_STATIC_EQUILIBRIUM_NOT_CONVERGED');
    end
    rethrow(ME);
end
elapsed_s = toc(started);

if elapsed_s > hard_timeout_s
    error('DynamicEHLReference:Timeout', 'REFERENCE_EXTRACTION_TIMEOUT');
end
if elapsed_s > soft_warning_s
    fprintf('D2P1R_SOFT_WARNING elapsed_s=%.6g\n', elapsed_s);
end
if ~formal_thermal_gate_passed(thermal_result)
    error('DynamicEHLReference:ThermalNotConverged', 'REFERENCE_THERMAL_STATE_NOT_CONVERGED');
end

contacts = thermal_result.contact_state.bearings;
if numel(contacts) < 1 || ~isfield(contacts(1), 'Q') || ~isfield(contacts(1), 'contact_angle')
    error('DynamicEHLReference:Incomplete', 'REFERENCE_STATE_INCOMPLETE');
end
ball_contact = contacts(1);
Q_N = reshape(ball_contact.Q, 1, []);
[Q_reference_N, ball_id] = max(Q_N);
if isempty(ball_id) || ~isfinite(Q_reference_N) || Q_reference_N <= 0
    error('DynamicEHLReference:Incomplete', 'REFERENCE_STATE_INCOMPLETE');
end

reference_state = empty_reference_state();
reference_state.reference_type = "DYNAMIC_EHL_LOCAL_SINGLE_CONTACT_REFERENCE";
reference_state.Q_reference_N = Q_reference_N;
reference_state.T_K = thermal_result.thermal_state.ball.T_film + 273.15;
reference_state.U_m_s = formal_entrainment_velocity(ball_contact);
[reference_state.Rx_m, reference_state.Ry_m] = outer_race_geometry(params.bearing(1), ball_contact.contact_angle);
config = dynamic_ehl_model_config();
reference_state.eta_Pa_s = thermal_result.thermal_state.ball.eta;
reference_state.alpha_p_Pa_inv = thermal_result.thermal_state.ball.alpha_p;
reference_state.E_star_Pa = config.material.E_star_Pa;
reference_state.ball_id = ball_id;
reference_state.contact_type = "FRONT_BALL_OUTER_RACE_MAX_LOADED_BALL";
reference_state.thermal_converged = thermal_result.converged;
reference_state.mechanical_converged = thermal_result.converged;
reference_state.mechanical_residual = thermal_result.mechanical_residual;
reference_state.thermal_residual_W = thermal_result.power_residual_W;
reference_state.elapsed_s = elapsed_s;
reference_state.source_note = "Maximum loaded ball from the final formally converged 20 C thermal-mechanical workpoint; T, eta, alpha_p and U are read from the same final frozen thermal contact state. This is a single outer-race contact reference, not an entire-bearing dynamic-EHL model.";

values = [reference_state.Q_reference_N, reference_state.T_K, reference_state.U_m_s, ...
    reference_state.Rx_m, reference_state.Ry_m, reference_state.eta_Pa_s, ...
    reference_state.alpha_p_Pa_inv, reference_state.E_star_Pa];
if any(~isfinite(values)) || any(values <= 0) || ~isfinite(reference_state.mechanical_residual) || ...
        ~isfinite(reference_state.thermal_residual_W) || ~isscalar(reference_state.ball_id) || reference_state.ball_id < 1
    error('DynamicEHLReference:Incomplete', 'REFERENCE_STATE_INCOMPLETE');
end
reference_state.available = true;
end

function pass = formal_thermal_gate_passed(result)
required = {'converged','final_consistency_recompute_count','mechanical_residual', ...
    'axial_constraint_error_m','temperature_residual_C','power_residual_W','contact_state','thermal_state'};
pass = isstruct(result) && all(isfield(result, required)) && result.converged && ...
    result.final_consistency_recompute_count == 1 && isfinite(result.mechanical_residual) && ...
    isfinite(result.axial_constraint_error_m) && isfinite(result.temperature_residual_C) && ...
    isfinite(result.power_residual_W);
end

function U_m_s = formal_entrainment_velocity(contact)
if ~isfield(contact, 'microphysics_state') || ~isfield(contact.microphysics_state, 'temperature') || ...
        ~isfield(contact.microphysics_state.temperature, 'contact_geometry') || ...
        ~isfield(contact.microphysics_state.temperature.contact_geometry, 'entrainment_velocity')
    error('DynamicEHLReference:Incomplete', 'REFERENCE_STATE_INCOMPLETE');
end
U_m_s = contact.microphysics_state.temperature.contact_geometry.entrainment_velocity;
end

function [Rx_m, Ry_m] = outer_race_geometry(bearing, contact_angle)
config = dynamic_ehl_model_config();
required = {'Db','Dm'};
if ~isstruct(bearing) || ~all(isfield(bearing, required)) || ~isfield(config, 'geometry_definition') || ...
        ~isfield(config.geometry_definition, 'groove_conformity_f')
    error('DynamicEHLReference:Geometry', 'REFERENCE_GEOMETRY_INCOMPLETE');
end
Dw_m = bearing.Db;
Dm_m = bearing.Dm;
f = config.geometry_definition.groove_conformity_f;
if ~isscalar(contact_angle) || ~isfinite(contact_angle) || ~isfinite(Dw_m) || ~isfinite(Dm_m) || ...
        ~isfinite(f) || Dw_m <= 0 || Dm_m <= 0 || f <= 0.5 || abs(cos(contact_angle)) <= eps
    error('DynamicEHLReference:Geometry', 'REFERENCE_GEOMETRY_INCOMPLETE');
end
contact_span_m = Dm_m/cos(contact_angle);
Rx_m = Dw_m*(0.5*contact_span_m + 0.5*Dw_m)/contact_span_m;
Ry_m = f*Dw_m/(2*f - 1);
if ~isfinite(Rx_m) || ~isfinite(Ry_m) || Rx_m <= 0 || Ry_m <= 0
    error('DynamicEHLReference:Geometry', 'REFERENCE_GEOMETRY_INCOMPLETE');
end
end

function state = empty_reference_state()
state = struct('available',false,'reference_type',"",'Q_reference_N',NaN,'T_K',NaN, ...
    'U_m_s',NaN,'Rx_m',NaN,'Ry_m',NaN,'eta_Pa_s',NaN,'alpha_p_Pa_inv',NaN, ...
    'E_star_Pa',NaN,'ball_id',NaN,'contact_type',"",'thermal_converged',false, ...
    'mechanical_converged',false,'mechanical_residual',NaN,'thermal_residual_W',NaN, ...
    'elapsed_s',NaN,'source_note',"");
end
