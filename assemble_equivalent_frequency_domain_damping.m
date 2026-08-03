function out = assemble_equivalent_frequency_domain_damping( ...
    mode, C_rayleigh_legacy, C_foundation, C_ehl_old, C_rayleigh_ref)
%ASSEMBLE_EQUIVALENT_FREQUENCY_DOMAIN_DAMPING Pure Stage B damping adapter.

mode = validate_mode(mode);
matrices = {C_rayleigh_legacy, C_foundation, C_ehl_old, C_rayleigh_ref};
validate_matrices(matrices);

out = struct();
out.mode = mode;
out.C_base = C_foundation;
out.C_rayleigh_legacy = C_rayleigh_legacy;
out.C_rayleigh_legacy_audit = C_rayleigh_legacy;
out.C_ehl_old_audit = C_ehl_old;
out.C_rayleigh_ref = C_rayleigh_ref;
out.old_ehl_in_formal = false;
out.legacy_rayleigh_in_formal = false;
out.gyro_is_separate = true;
out.verification_scope = 'SOURCE_AND_SYNTHETIC_CONTRACT';
out.runtime_legacy_equivalence_pending = true;

switch mode
    case 'legacy_ehl_audit'
        out.C_legacy_audit = ...
            C_rayleigh_legacy + C_foundation + C_ehl_old;
        out.C_formal = [];
    case 'fixed_equivalent_system'
        out.C_legacy_audit = [];
        out.C_formal = C_foundation + C_rayleigh_ref;
end

assembled = {out.C_legacy_audit, out.C_formal};
assembled = assembled(~cellfun(@isempty, assembled));
if any(cellfun(@(matrix) any(~isfinite(matrix(:))), assembled))
    error('StageB:DampingOutputFinite', ...
        'The assembled damping matrix must contain only finite values.');
end

component_norm_values = [stable_norm(C_rayleigh_legacy), ...
    stable_norm(C_foundation), stable_norm(C_ehl_old), ...
    stable_norm(C_rayleigh_ref), stable_norm(out.C_legacy_audit), ...
    stable_norm(out.C_formal)];
if any(~isfinite(component_norm_values))
    error('StageB:DampingOutputFinite', ...
        'Damping component norms must remain finite.');
end
out.component_norms = struct( ...
    'rayleigh_legacy_fro', component_norm_values(1), ...
    'foundation_fro', component_norm_values(2), ...
    'ehl_old_fro', component_norm_values(3), ...
    'rayleigh_ref_fro', component_norm_values(4), ...
    'legacy_audit_fro', component_norm_values(5), ...
    'formal_fro', component_norm_values(6));
out.symmetry_errors = struct( ...
    'C_rayleigh_legacy', symmetry_error(C_rayleigh_legacy), ...
    'C_foundation', symmetry_error(C_foundation), ...
    'C_ehl_old', symmetry_error(C_ehl_old), ...
    'C_rayleigh_ref', symmetry_error(C_rayleigh_ref), ...
    'C_legacy_audit', symmetry_error(out.C_legacy_audit), ...
    'C_formal', symmetry_error(out.C_formal));
out.finite_flags = struct( ...
    'C_rayleigh_legacy', all(isfinite(C_rayleigh_legacy(:))), ...
    'C_foundation', all(isfinite(C_foundation(:))), ...
    'C_ehl_old', all(isfinite(C_ehl_old(:))), ...
    'C_rayleigh_ref', all(isfinite(C_rayleigh_ref(:))), ...
    'all', true);
end

function mode = validate_mode(mode)
if isstring(mode) && isscalar(mode)
    mode = char(mode);
end
if ~(ischar(mode) && isrow(mode)) || ...
        ~any(strcmp(mode, {'legacy_ehl_audit','fixed_equivalent_system'}))
    error('StageB:DampingMode', ...
        'mode must be legacy_ehl_audit or fixed_equivalent_system.');
end
end

function validate_matrices(matrices)
for k = 1:numel(matrices)
    matrix = matrices{k};
    if ~((isa(matrix, 'double') || isa(matrix, 'single')) && ...
            isreal(matrix) && ismatrix(matrix))
        error('StageB:DampingType', ...
            'All damping components must be real single or double matrices.');
    end
    if isempty(matrix) || size(matrix,1) ~= size(matrix,2)
        error('StageB:DampingShape', ...
            'All damping components must be nonempty square matrices.');
    end
end

classes = cellfun(@class, matrices, 'UniformOutput', false);
if ~all(strcmp(classes, classes{1}))
    error('StageB:DampingClass', ...
        'All damping components must use the same floating-point class.');
end
reference_size = size(matrices{1});
if any(cellfun(@(matrix) ~isequal(size(matrix), reference_size), matrices))
    error('StageB:DampingSize', ...
        'All damping components must have identical dimensions.');
end
if any(cellfun(@(matrix) any(~isfinite(matrix(:))), matrices))
    error('StageB:DampingFinite', ...
        'All damping components must contain only finite values.');
end
symmetry_errors = cellfun(@symmetry_error, matrices);
if any(~isfinite(symmetry_errors)) || any(symmetry_errors > 1e-12)
    error('StageB:DampingSymmetry', ...
        'All damping components must be symmetric within 1e-12.');
end
end

function value = symmetry_error(matrix)
if isempty(matrix)
    value = 0;
    return;
end
scale = max(abs(matrix(:)));
if scale == 0
    value = 0;
    return;
end
scaled = matrix/scale;
value = norm(scaled-scaled.', 'fro')/max(norm(scaled, 'fro'), eps);
end

function value = stable_norm(matrix)
if isempty(matrix)
    value = 0;
    return;
end
scale = max(abs(matrix(:)));
if scale == 0
    value = 0;
else
    value = scale*norm(matrix/scale, 'fro');
end
end
