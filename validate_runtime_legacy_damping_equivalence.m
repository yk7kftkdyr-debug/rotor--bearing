function audit = validate_runtime_legacy_damping_equivalence( ...
    C_rayleigh_r, C_foundation_r, C_ehl_total_r, C_total_r)
%VALIDATE_RUNTIME_LEGACY_DAMPING_EQUIVALENCE Stage C entry Gate contract.

matrices = {C_rayleigh_r, C_foundation_r, C_ehl_total_r, C_total_r};
if any(cellfun(@(matrix) ...
        ~((isa(matrix, 'double') || isa(matrix, 'single')) && ...
        isreal(matrix) && ismatrix(matrix)), matrices))
    error('StageB:RuntimeDampingType', ...
        'Runtime damping components must be real single or double matrices.');
end
classes = cellfun(@class, matrices, 'UniformOutput', false);
if ~all(strcmp(classes, classes{1}))
    error('StageB:RuntimeDampingClass', ...
        'Runtime damping components must use one floating-point class.');
end

audit = struct();
audit.relative_error = Inf;
audit.all_finite = all(cellfun(@(matrix) ...
    all(isfinite(matrix(:))), matrices));
audit.size_match = size_contract(matrices);
audit.symmetry_errors = struct( ...
    'C_rayleigh_r', NaN, 'C_foundation_r', NaN, ...
    'C_ehl_total_r', NaN, 'C_total_r', NaN);
audit.passed = false;
audit.space = 'FULL_ORDER_OR_ACTUAL_RUNTIME_SPACE';

if ~audit.size_match
    return;
end

audit.symmetry_errors = struct( ...
    'C_rayleigh_r', symmetry_error(C_rayleigh_r), ...
    'C_foundation_r', symmetry_error(C_foundation_r), ...
    'C_ehl_total_r', symmetry_error(C_ehl_total_r), ...
    'C_total_r', symmetry_error(C_total_r));
if ~audit.all_finite
    return;
end

C_reconstructed = C_rayleigh_r + C_foundation_r + C_ehl_total_r;
if any(~isfinite(C_reconstructed(:)))
    audit.all_finite = false;
    return;
end
audit.relative_error = relative_error(C_reconstructed, C_total_r);
symmetry_values = cell2mat(struct2cell(audit.symmetry_errors));
audit.passed = isfinite(audit.relative_error) && ...
    audit.relative_error <= 1e-12 && all(isfinite(symmetry_values)) && ...
    all(symmetry_values <= 1e-12);
end

function matches = size_contract(matrices)
square = cellfun(@(matrix) ~isempty(matrix) && ...
    size(matrix,1) == size(matrix,2), matrices);
reference_size = size(matrices{1});
matches = all(square) && all(cellfun(@(matrix) ...
    isequal(size(matrix), reference_size), matrices));
end

function value = symmetry_error(matrix)
scale = max(abs(matrix(:)));
if scale == 0
    value = 0;
    return;
end
scaled = matrix/scale;
value = norm(scaled-scaled.', 'fro')/max(norm(scaled, 'fro'), eps);
end

function value = relative_error(actual, expected)
scale = max([abs(actual(:)); abs(expected(:))]);
if scale == 0
    value = 0;
    return;
end
actual_scaled = actual/scale;
expected_scaled = expected/scale;
value = norm(actual_scaled-expected_scaled, 'fro')/ ...
    max(norm(expected_scaled, 'fro'), eps);
end
