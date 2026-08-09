function projection = build_stage_f_frozen_input_projection(results)
%BUILD_STAGE_F_FROZEN_INPUT_PROJECTION Hash only Stage F scientific inputs.

temperatures = [20 50 80 100];
scenarios = {'LOW','NOMINAL','HIGH'};
if ~isstruct(results) || ~all(isfield(results,{'static','damping', ...
        'frequency_response'}))
    error('StageF:FrozenInputProjection', ...
        'STAGE_F_FROZEN_INPUT_PROJECTION_FAILED: scientific inputs are incomplete.');
end

projection = struct('schema_version', ...
    'stage-f-frozen-input-projection-v1','temperature_cases',struct(), ...
    'C_formal',struct());
formal = isfield(results.static,'T20') && ...
    isfield(results.static.T20,'bearing_mapping');
for temperature = temperatures
    tf = sprintf('T%d',temperature);
    if ~isfield(results.static,tf) || ...
            ~isfield(results.frequency_response,tf)
        incomplete(tf);
    end
    state = results.static.(tf);
    required_static = {'q_static','thermal_state'};
    if ~all(isfield(state,required_static))
        incomplete(tf);
    end

    thermal_hash = serialized_sha256(state.thermal_state);
    clearance_hash = serialized_sha256(clearance_source(state));
    [front_contact,rear_contact] = contact_sources(state,formal,tf);
    front_contact_hash = serialized_sha256(front_contact);
    rear_contact_hash = serialized_sha256(rear_contact);
    [front_mask,rear_mask,front_J,rear_J,front_k,rear_k] = ...
        mapping_sources(results,temperature,formal);
    K_t = tangent_source(state,temperature,formal,tf);
    K_t_hash = serialized_sha256(K_t);

    qhat_hashes = struct(); Fhat_hashes = struct();
    for scenario_index = 1:numel(scenarios)
        scenario = scenarios{scenario_index};
        if ~isfield(results.frequency_response.(tf),scenario)
            incomplete([tf '.' scenario]);
        end
        response = results.frequency_response.(tf).(scenario);
        qhat_hashes.(scenario) = serialized_sha256( ...
            required_or_empty(response,'qhat_full',formal,[tf '.' scenario]));
        Fhat_hashes.(scenario) = serialized_sha256( ...
            required_or_empty(response,'Fhat',formal,[tf '.' scenario]));
    end

    item = struct( ...
        'static_state_summary_sha256','', ...
        'thermal_state_sha256',thermal_hash, ...
        'clearance_sha256',clearance_hash, ...
        'contact_state_front_sha256',front_contact_hash, ...
        'contact_state_rear_sha256',rear_contact_hash, ...
        'loaded_mask_front_sha256',serialized_sha256(front_mask), ...
        'loaded_mask_rear_sha256',serialized_sha256(rear_mask), ...
        'J_front_sha256',serialized_sha256(front_J), ...
        'J_rear_sha256',serialized_sha256(rear_J), ...
        'k_static_front_sha256',serialized_sha256(front_k), ...
        'k_static_rear_sha256',serialized_sha256(rear_k), ...
        'qhat_source_sha256',qhat_hashes, ...
        'Fhat_source_sha256',Fhat_hashes, ...
        'K_t_sha256',K_t_hash);
    summary = struct('q_static_sha256',serialized_sha256(state.q_static), ...
        'thermal_state_sha256',thermal_hash, ...
        'clearance_sha256',clearance_hash, ...
        'contact_state_front_sha256',front_contact_hash, ...
        'contact_state_rear_sha256',rear_contact_hash, ...
        'loaded_mask_front_sha256',item.loaded_mask_front_sha256, ...
        'loaded_mask_rear_sha256',item.loaded_mask_rear_sha256, ...
        'J_front_sha256',item.J_front_sha256, ...
        'J_rear_sha256',item.J_rear_sha256, ...
        'k_static_front_sha256',item.k_static_front_sha256, ...
        'k_static_rear_sha256',item.k_static_rear_sha256, ...
        'K_t_sha256',K_t_hash);
    item.static_state_summary_sha256 = serialized_sha256(summary);
    projection.temperature_cases.(tf) = item;
end

for scenario_index = 1:numel(scenarios)
    scenario = scenarios{scenario_index};
    field = ['C_formal_' scenario];
    if ~isfield(results.damping,field), incomplete(field); end
    projection.C_formal.(scenario) = serialized_sha256(results.damping.(field));
end
end

function source = clearance_source(state)
if isfield(state,'clearance_state')
    source = state.clearance_state;
elseif isfield(state.thermal_state,'ball') && ...
        isfield(state.thermal_state.ball,'working_clearance') && ...
        isfield(state.thermal_state,'roller') && ...
        isfield(state.thermal_state.roller,'working_clearance')
    source = struct('front_m',state.thermal_state.ball.working_clearance, ...
        'rear_m',state.thermal_state.roller.working_clearance);
else
    source = [];
end
end

function [front,rear] = contact_sources(state,formal,tf)
front = []; rear = [];
if isfield(state,'bearing_state_front'), front = state.bearing_state_front; end
if isfield(state,'bearing_state_rear'), rear = state.bearing_state_rear; end
if formal && (isempty(front) || isempty(rear)), incomplete(tf); end
end

function [front_mask,rear_mask,front_J,rear_J,front_k,rear_k] = ...
        mapping_sources(results,temperature,formal)
if formal
    front = build_stage_f_rolling_element_linearization_mapping( ...
        results,temperature,'front');
    rear = build_stage_f_rolling_element_linearization_mapping( ...
        results,temperature,'rear');
    front_mask = front.loaded_mask_static_slice;
    rear_mask = rear.loaded_mask_static_slice;
    front_J = front.contact_jacobian_local;
    rear_J = rear.contact_jacobian_local;
    front_k = front.k_static_slice;
    rear_k = rear.k_static_slice;
else
    front_mask = []; rear_mask = []; front_J = []; rear_J = [];
    front_k = []; rear_k = [];
end
end

function value = tangent_source(state,temperature,formal,tf)
if temperature == 20 && isfield(state,'K_tangent_original')
    value = state.K_tangent_original;
elseif temperature ~= 20 && isfield(state,'K_tangent')
    value = state.K_tangent;
elseif formal
    incomplete(tf);
else
    value = [];
end
end

function value = required_or_empty(source,name,formal,label)
if isfield(source,name)
    value = source.(name);
elseif formal
    incomplete([label '.' name]);
else
    value = [];
end
end

function hash = serialized_sha256(value)
bytes = getByteStreamFromArray(value);
engine = java.security.MessageDigest.getInstance('SHA-256');
engine.update(typecast(uint8(bytes),'int8'));
digest = typecast(engine.digest(),'uint8');
hash = lower(reshape(dec2hex(digest,2).',1,[]));
end

function incomplete(label)
error('StageF:FrozenInputProjection', ...
    'STAGE_F_FROZEN_INPUT_PROJECTION_FAILED: missing %s scientific input.',label);
end
