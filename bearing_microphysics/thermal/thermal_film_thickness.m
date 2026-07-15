function [film_thickness_m, state] = thermal_film_thickness(brg, omega_rad_s, Q_N, eta_Pa_s, alpha_p_Pa_inv)
%THERMAL_FILM_THICKNESS Reduced local EHL film model for current contacts.
%   Reference films are computed solely from the current bearing geometry,
%   speed, local loads, viscosity reference and current alpha_p.  This is a
%   reduced model, not a fixed 20 kN/80 kN operating-case correlation.

validateattributes(omega_rad_s, {'numeric'}, {'real', 'finite', 'scalar'});
validateattributes(eta_Pa_s, {'numeric'}, {'real', 'finite', 'scalar', 'positive'});
validateattributes(alpha_p_Pa_inv, {'numeric'}, {'real', 'finite', 'scalar', 'positive'});
expected_size = contact_grid_size(brg);
validateattributes(Q_N, {'numeric'}, {'real', 'finite', 'nonnegative'});
if ~isequal(size(Q_N), expected_size)
    error('thermal_film_thickness:ContactGridSize', ...
        'Q_N for a %s bearing must have size %s.', lower(brg.type), mat2str(expected_size));
end

eta_reference_Pa_s = 1003.5 * 27.6e-6;
[effective_radius_m, effective_length_m, exponent] = local_geometry(brg);
entrainment_speed_m_s = max(abs(omega_rad_s) * brg.Dm / 4, eps);
elastic_modulus_Pa = 2.06e11;
loaded = Q_N > 0;
reference_film_m = zeros(expected_size);

if any(loaded, 'all')
    if strcmpi(brg.type, 'ball')
        load_group = Q_N(loaded) ./ (elastic_modulus_Pa * effective_radius_m^2);
    else
        load_group = Q_N(loaded) ./ (elastic_modulus_Pa * effective_radius_m * effective_length_m);
    end
    speed_group = eta_reference_Pa_s * entrainment_speed_m_s / ...
        (elastic_modulus_Pa * effective_radius_m);
    pressure_group = alpha_p_Pa_inv * elastic_modulus_Pa;
    reference_film_m(loaded) = effective_radius_m .* 1.6 .* ...
        speed_group.^exponent .* pressure_group.^0.10 .* load_group.^(-0.067);
end

film_thickness_m = reference_film_m .* (eta_Pa_s / eta_reference_Pa_s).^exponent;
if any(~isfinite(film_thickness_m), 'all') || any(film_thickness_m(loaded) <= 0)
    error('thermal_film_thickness:InvalidFilm', ...
        'Loaded contacts must produce finite positive reduced-EHL films.');
end
if any(loaded, 'all'), h_min_m = min(film_thickness_m(loaded)); else, h_min_m = NaN; end
state = struct('model', 'reduced_local_ehl', 'loaded', loaded, ...
    'h_min_m', h_min_m, 'reference_film_m', reference_film_m, ...
    'eta_reference_Pa_s', eta_reference_Pa_s, ...
    'entrainment_speed_m_s', entrainment_speed_m_s, ...
    'effective_radius_m', effective_radius_m, ...
    'effective_length_m', effective_length_m, 'film_exponent', exponent);
end

function expected_size = contact_grid_size(brg)
if strcmpi(brg.type, 'ball')
    expected_size = [1 brg.n];
elseif strcmpi(brg.type, 'roller')
    expected_size = [brg.n brg.stage4A_slice_count];
else
    error('thermal_film_thickness:UnknownBearingType', 'Unknown bearing type: %s.', brg.type);
end
end

function [radius_m, length_m, exponent] = local_geometry(brg)
if strcmpi(brg.type, 'ball')
    radius_m = brg.Db / 2;
    length_m = brg.Db;
    exponent = 0.68;
else
    radius_m = brg.Dw / 2;
    length_m = brg.L;
    exponent = 0.70;
end
validateattributes(brg.Dm, {'numeric'}, {'real', 'finite', 'scalar', 'positive'});
validateattributes(radius_m, {'numeric'}, {'real', 'finite', 'scalar', 'positive'});
validateattributes(length_m, {'numeric'}, {'real', 'finite', 'scalar', 'positive'});
end
