function snapshot = stage_e_frozen_input_snapshot(results)
%STAGE_E_FROZEN_INPUT_SNAPSHOT Capture all Stage A--D inputs frozen by Stage E.

temperatures = [20 50 80 100];
required_damping = {'K_ref_20C','alpha','beta','C_foundation_full', ...
    'C_rayleigh_ref_LOW','C_rayleigh_ref_NOMINAL','C_rayleigh_ref_HIGH', ...
    'C_formal_LOW','C_formal_NOMINAL','C_formal_HIGH'};
if ~isstruct(results) || ~all(isfield(results,{'static','damping','modal'})) || ...
        ~isfield(results.modal,'T20') || ...
        ~isfield(results.modal.T20,'anchor_modes') || ...
        ~all(isfield(results.damping,required_damping))
    error('StageE:FrozenSnapshot','Stage A--D frozen inputs are incomplete.');
end
snapshot = struct();
snapshot.static = results.static;
snapshot.damping = struct();
for k = 1:numel(required_damping)
    name = required_damping{k};
    snapshot.damping.(name) = results.damping.(name);
end
snapshot.modal_T20_anchor_modes = results.modal.T20.anchor_modes;

hash_values = struct();
for temperature = temperatures
    field = sprintf('T%d',temperature);
    if ~isfield(results.static,field) || ...
            ~all(isfield(results.static.(field),{'q_static','thermal_state'}))
        error('StageE:FrozenSnapshot','Accepted %s static state is incomplete.',field);
    end
    state = results.static.(field);
    hash_values.([field '_q_static']) = serialized_sha256(state.q_static);
    hash_values.([field '_thermal_state']) = serialized_sha256(state.thermal_state);
    if temperature == 20 && isfield(state,'K_tangent_original')
        K_t = state.K_tangent_original;
    elseif temperature ~= 20 && isfield(state,'K_tangent')
        K_t = state.K_tangent;
    else
        error('StageE:FrozenSnapshot','Accepted %s tangent stiffness is incomplete.',field);
    end
    hash_values.([field '_K_t']) = serialized_sha256(K_t);
end
hash_values.M = serialized_sha256(results.static.T20.M);
hash_values.G = serialized_sha256(results.static.T20.G);
for k = 1:numel(required_damping)
    name = required_damping{k};
    hash_values.(name) = serialized_sha256(results.damping.(name));
end
hash_values.T20_anchor_modes = serialized_sha256(results.modal.T20.anchor_modes);
snapshot.hashes = hash_values;
snapshot.schema_version = 'stage-e-frozen-input-snapshot-v1';
end

function hash = serialized_sha256(value)
bytes = getByteStreamFromArray(value);
engine = java.security.MessageDigest.getInstance('SHA-256');
engine.update(bytes);
digest = typecast(engine.digest(),'uint8');
hash = lower(reshape(dec2hex(digest,2).',1,[]));
end
