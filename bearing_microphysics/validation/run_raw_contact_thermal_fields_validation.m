function run_raw_contact_thermal_fields_validation()
%RUN_RAW_CONTACT_THERMAL_FIELDS_VALIDATION In-memory B0/B1 raw-contact check.

evalc('result = run_microphysics_interface_validation(false);');
assert(result.b0.max_u == 0 && result.b0.max_delta_fb == 0 && result.b0.max_bu == 0);
assert(result.b1.u.max_abs_error == 0 && result.b1.v.max_abs_error == 0 && ...
    result.b1.a.max_abs_error == 0 && result.b1.bearing_force.max_abs_error == 0 && ...
    result.b1.contact_body_count.max_abs_error == 0 && ...
    result.b1.Kb_last.max_abs_error == 0 && result.b1.Cb_last.max_abs_error == 0 && ...
    result.b1.newton_iterations.max_abs_error == 0 && ...
    result.b1.unconverged_step_difference == 0);
assert_raw_contact_film_checks();
fprintf('RAW_CONTACT_THERMAL_FIELDS_PASS\n');
end

function assert_raw_contact_film_checks()
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(root);

ball_shape = [1 8];
roller_shape = [12 5];
assert(isequal(normalize_contact_film_thickness([], ball_shape), zeros(ball_shape)));
assert(isequal(normalize_contact_film_thickness(0, roller_shape), zeros(roller_shape)));
assert_throws(@() normalize_contact_film_thickness(ones(2, 2), ball_shape));
assert_throws(@() normalize_contact_film_thickness(ones(3, 4), roller_shape));

delta_geometry = [3 2 1 0] * 1e-6;
clearance = 0.5e-6;
film = normalize_contact_film_thickness(0.25e-6, size(delta_geometry));
assert(all(delta_geometry - clearance - film <= delta_geometry - clearance));
end

function assert_throws(action)
try
    action();
catch exception
    assert(strcmp(exception.identifier, 'raw_contact:FilmThicknessShape'));
    return;
end
error('raw_contact:ExpectedFilmThicknessShapeError', ...
    'An invalid film_thickness shape did not raise an error.');
end
