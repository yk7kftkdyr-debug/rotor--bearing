function damping = build_ehl_damping_matrix(contact_state, thermal_state, contact_geometry, brg)
%BUILD_EHL_DAMPING_MATRIX Fixed workpoint normal squeeze-film damping matrix.

[contact, thermal, geometry] = resolve_existing_state(contact_state, thermal_state, contact_geometry);
validateattributes(brg, {'struct'}, {'nonempty'});
valid_mask = contact.loaded_mask & contact.Q > 0 & contact.delta > 0 & ...
    contact.film_thickness > 0 & isfinite(contact.Q) & isfinite(contact.delta) & isfinite(contact.film_thickness);
assert_no_invalid_loaded_contact(contact, valid_mask);
if ~any(valid_mask, 'all')
    error('EhlDamping:NoValidContact', 'No loaded contact has finite positive Q, delta and film thickness.');
end

active_dof_indices = admissible_dofs(brg);
S = zeros(5, numel(active_dof_indices)); S(active_dof_indices,:) = eye(numel(active_dof_indices));
switch lower(brg.type)
    case 'ball'
        [C_raw_full, coefficient] = ball_damping(contact, thermal.viscosity_Pa_s, geometry, brg, valid_mask);
    case 'roller'
        [C_raw_full, coefficient] = roller_damping(contact, thermal.viscosity_Pa_s, geometry, brg, valid_mask);
    otherwise
        error('EhlDamping:BearingType', 'Unsupported bearing type: %s.', brg.type);
end
if any(~isfinite(C_raw_full), 'all') || any(~isfinite(coefficient(valid_mask))) || any(coefficient(valid_mask) < 0)
    error('EhlDamping:NonFiniteCoefficient', 'EHL damping contains non-finite or negative valid-contact coefficients.');
end

C_active_raw = S'*C_raw_full*S;
C_active_sym = 0.5*(C_active_raw+C_active_raw');
C_local = zeros(5,5); C_local(active_dof_indices, active_dof_indices) = C_active_sym;
symmetry_error = norm(C_raw_full-C_raw_full', 'fro')/max(norm(C_raw_full, 'fro'), 1);
eigenvalues_active = eig(C_active_sym);
minimum_eigenvalue = min(eigenvalues_active); maximum_eigenvalue = max(eigenvalues_active);
dissipative_work_pass = dissipative_work_check(C_active_sym, minimum_eigenvalue, maximum_eigenvalue);
finite_pass = all(isfinite(C_raw_full), 'all') && all(isfinite(C_local), 'all') && ...
    isfinite(thermal.viscosity_Pa_s) && isfinite(thermal.T_film_C) && thermal.viscosity_Pa_s > 0;
h_min_loaded_m = min(contact.film_thickness(valid_mask));
pass = finite_pass && symmetry_error <= 1e-10 && dissipative_work_pass && ...
    isfinite(h_min_loaded_m) && h_min_loaded_m > 0 && nnz(valid_mask) >= 1;

damping = struct('bearing_type', lower(brg.type), ...
    'local_dof_order', {{'ux','uy','uz','theta_x','theta_y'}}, ...
    'active_dof_indices', active_dof_indices, 'valid_mask', valid_mask, ...
    'valid_contact_count', nnz(valid_mask), 'viscosity_Pa_s', thermal.viscosity_Pa_s, ...
    'h_min_loaded_m', h_min_loaded_m, 'contact_damping_coefficient', coefficient, ...
    'C_raw_full', C_raw_full, 'C_active_raw', C_active_raw, 'C_local', C_local, ...
    'symmetry_error', symmetry_error, 'eigenvalues_active', eigenvalues_active, ...
    'minimum_eigenvalue', minimum_eigenvalue, 'maximum_eigenvalue', maximum_eigenvalue, ...
    'dissipative_work_pass', dissipative_work_pass, 'finite_pass', finite_pass, 'pass', pass);
end

function [contact, thermal, geometry] = resolve_existing_state(contact_state, thermal_state, contact_geometry)
contact = struct('Q', require_field(contact_state, 'Q'), 'delta', require_field(contact_state, 'delta'), ...
    'film_thickness', extract_film(contact_state), 'loaded_mask', extract_loaded_mask(contact_state), ...
    'element_angle', extract_angle(contact_state), 'contact_angle', require_field(contact_state, 'contact_angle'), ...
    'slice_z', extract_slice_z(contact_state));
thermal = struct('viscosity_Pa_s', extract_viscosity(thermal_state), 'T_film_C', extract_film_temperature(thermal_state));
geometry = struct('R_effective_m', extract_geometry(contact_geometry, 'R_effective_m', 'R_effective'), ...
    'E_star_Pa', extract_geometry(contact_geometry, 'E_star_Pa', 'E_star'), ...
    'contact_length_m', extract_geometry(contact_geometry, 'contact_length_m', 'contact_length'));
if ~isequal(size(contact.Q), size(contact.delta)) || ~isequal(size(contact.Q), size(contact.film_thickness)) || ~isequal(size(contact.Q), size(contact.loaded_mask))
    error('EhlDamping:ContactDimensions', 'Q, delta, film_thickness and loaded_mask must have identical dimensions.');
end
validateattributes(thermal.viscosity_Pa_s, {'numeric'}, {'scalar','real','positive','finite'});
validateattributes(thermal.T_film_C, {'numeric'}, {'scalar','real','finite'});
validateattributes(geometry.R_effective_m, {'numeric'}, {'scalar','real','positive','finite'});
validateattributes(geometry.E_star_Pa, {'numeric'}, {'scalar','real','positive','finite'});
validateattributes(geometry.contact_length_m, {'numeric'}, {'scalar','real','positive','finite'});
end

function [C, coefficient] = ball_damping(contact, eta, geometry, brg, valid_mask)
theta = contact.element_angle(:); Q = contact.Q(:); h = contact.film_thickness(:); mask = valid_mask(:);
if numel(theta) ~= numel(Q), error('EhlDamping:BallAngles', 'Ball element_angle must correspond one-to-one with Q.'); end
alpha = contact.contact_angle;
if ~isscalar(alpha) || ~isfinite(alpha), error('EhlDamping:BallContactAngle', 'Ball contact_angle must be one finite workpoint value.'); end
z = brg.stage4A_width_m/2; coefficient = zeros(size(Q)); C = zeros(5,5);
for j = find(mask).'
    a = (3*Q(j)*geometry.R_effective_m/(4*geometry.E_star_Pa))^(1/3);
    coefficient(j) = 3*pi*eta*a^4/(2*h(j)^3);
    g = [cos(alpha)*cos(theta(j)); cos(alpha)*sin(theta(j)); sin(alpha); ...
        -z*cos(alpha)*sin(theta(j)); z*cos(alpha)*cos(theta(j))];
    C = C + coefficient(j)*(g*g.');
end
coefficient = reshape(coefficient, size(contact.Q));
end

function [C, coefficient] = roller_damping(contact, eta, geometry, brg, valid_mask)
Q = contact.Q; h = contact.film_thickness; ns = brg.stage4A_slice_count;
if size(Q,2) ~= ns, error('EhlDamping:RollerSlices', 'Roller Q must use brg.stage4A_slice_count columns.'); end
theta = contact.element_angle(:); slice_z = contact.slice_z(:).';
if numel(theta) ~= size(Q,1) || numel(slice_z) ~= ns
    error('EhlDamping:RollerGeometry', 'Roller element_angle and slice_z must match Q dimensions.');
end
deltaL = brg.L/ns; coefficient = zeros(size(Q)); C = zeros(5,5);
for j = 1:size(Q,1)
    for s = find(valid_mask(j,:))
        qjs = Q(j,s)/deltaL;
        b = sqrt(4*qjs*geometry.R_effective_m/(pi*geometry.E_star_Pa));
        coefficient(j,s) = 8*eta*deltaL*b^3/h(j,s)^3;
        g = [cos(theta(j)); sin(theta(j)); 0; -slice_z(s)*sin(theta(j)); slice_z(s)*cos(theta(j))];
        C = C + coefficient(j,s)*(g*g.');
    end
end
end

function indices = admissible_dofs(brg)
if strcmpi(brg.type, 'ball')
    indices = [1 2 4 5];
else
    indices = 1:5;
end
if strcmpi(brg.type, 'ball') && ~isequal(indices, [1 2 4 5]), error('EhlDamping:BallConstraint', 'Ball active indices must exclude constrained uz.'); end
end

function pass = dissipative_work_check(C, lambda_min, lambda_max)
tolerance = 1e-10*max(1, lambda_max);
if lambda_min < -tolerance, pass = false; return; end
n = size(C,1); directions = eye(n);
if n >= 2, directions = [directions, ([1;1;zeros(n-2,1)])/sqrt(2)]; end
if n >= 3, directions = [directions, ([zeros(n-2,1);1;1])/sqrt(2)]; end
work = diag(directions'*C*directions);
pass = all(work >= -1e-10*max(1, norm(C,2)*sum(directions.^2,1)).');
end

function assert_no_invalid_loaded_contact(contact, valid_mask)
candidate = contact.loaded_mask & (contact.Q > 0) & (contact.delta > 0);
invalid = candidate & ~valid_mask;
if any(invalid, 'all')
    index = find(invalid, 1, 'first');
    [row, column] = ind2sub(size(invalid), index);
    error('EhlDamping:InvalidLoadedContact', 'Invalid loaded contact at index (%d,%d): Q=%g, delta=%g, h=%g.', row, column, contact.Q(index), contact.delta(index), contact.film_thickness(index));
end
if any(contact.loaded_mask & (~isfinite(contact.Q) | ~isfinite(contact.delta) | ~isfinite(contact.film_thickness)), 'all')
    index = find(contact.loaded_mask & (~isfinite(contact.Q) | ~isfinite(contact.delta) | ~isfinite(contact.film_thickness)), 1, 'first');
    [row, column] = ind2sub(size(contact.Q), index);
    error('EhlDamping:NonFiniteLoadedContact', 'Non-finite loaded contact at index (%d,%d).', row, column);
end
end

function value = require_field(s, name)
if ~isfield(s, name), error('EhlDamping:MissingField', 'Missing required field %s.', name); end
value = s.(name);
end

function value = extract_film(s)
if isfield(s, 'film_thickness'), value = s.film_thickness; else, value = require_field(require_field(s, 'film'), 'h_m'); end
end

function value = extract_loaded_mask(s)
if isfield(s, 'loaded_mask'), value = s.loaded_mask; else, value = require_field(require_field(s, 'film'), 'loaded_mask'); end
end

function value = extract_angle(s)
if isfield(s, 'element_angle'), value = s.element_angle; else, value = require_field(s, 'theta'); end
end

function value = extract_slice_z(s)
if isfield(s, 'slice_z'), value = s.slice_z; else, value = []; end
end

function value = extract_viscosity(s)
if isfield(s, 'viscosity_Pa_s'), value = s.viscosity_Pa_s; else, value = require_field(require_field(require_field(s, 'microphysics_state'), 'temperature'), 'input').eta; end
end

function value = extract_film_temperature(s)
if isfield(s, 'T_film_C'), value = s.T_film_C; else, value = require_field(require_field(require_field(s, 'microphysics_state'), 'temperature'), 'input').T_film; end
end

function value = extract_geometry(s, direct_name, nested_name)
if isfield(s, direct_name), value = s.(direct_name); else, value = require_field(s, nested_name); end
end
