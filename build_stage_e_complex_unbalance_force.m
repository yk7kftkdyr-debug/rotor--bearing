function force = build_stage_e_complex_unbalance_force(context)
%BUILD_STAGE_E_COMPLEX_UNBALANCE_FORCE Recover the +i*omega force phasor.

required = {'formal_force_at_time','omega_exc_rad_s','unbalance_mass_kg', ...
    'eccentricity_m','radial_dofs'};
for k = 1:numel(required)
    if ~isfield(context,required{k})
        error('StageE:ForceContext','Missing force context field %s.',required{k});
    end
end
omega = context.omega_exc_rad_s;
if ~isscalar(omega) || ~isfinite(omega) || omega <= 0
    error('StageE:ForceContext','omega_exc_rad_s must be positive and finite.');
end

phases = [0 pi/2 pi 3*pi/2];
times_s = phases/omega;
phase_forces = cell(1,4);
for k = 1:4
    phase_forces{k} = context.formal_force_at_time(times_s(k));
end
F0 = phase_forces{1}(:); n = numel(F0);
if n == 0 || any(cellfun(@(F) ~isnumeric(F) || ~isequal(size(F(:)),[n 1]) || ...
        any(~isfinite(F(:))),phase_forces))
    error('StageE:ForceContext','The four formal force samples must be finite equal-length vectors.');
end
F90 = phase_forces{2}(:); F180 = phase_forces{3}(:); F270 = phase_forces{4}(:);
Fhat = 0.5*(F0-F180) + 1i*0.5*(F270-F90);
reconstructed = [real(Fhat*exp(1i*phases(1))), ...
    real(Fhat*exp(1i*phases(2))),real(Fhat*exp(1i*phases(3))), ...
    real(Fhat*exp(1i*phases(4)))];
formal_phase_forces_N = [F0 F90 F180 F270];
column_errors = vecnorm(reconstructed-formal_phase_forces_N,2,1);
column_norms = vecnorm(formal_phase_forces_N,2,1);
reconstruction_error = max(column_errors)/max(max(column_norms),eps);
if reconstruction_error > 1e-12
    error('StageE:ForcePhaseReconstruction', ...
        'Four formal force samples are not a 1X harmonic to 1e-12.');
end

force = struct();
force.Fhat = Fhat;
force.omega_exc_rad_s = omega;
force.force_amplitude_N = context.unbalance_mass_kg*context.eccentricity_m*omega^2;
force.unbalance_mass_kg = context.unbalance_mass_kg;
force.eccentricity_m = context.eccentricity_m;
force.unbalance_U_kg_m = context.unbalance_mass_kg*context.eccentricity_m;
if isfield(context,'phase_rad'), force.unbalance_phase_rad = context.phase_rad; else, force.unbalance_phase_rad = 0; end
if isfield(context,'unbalance_node'), force.unbalance_node = context.unbalance_node; else, force.unbalance_node = NaN; end
force.phase_rad = phases;
force.sample_times_s = times_s;
force.formal_phase_forces_N = formal_phase_forces_N;
force.four_phase_reconstruction_error = reconstruction_error;
force.nonzero_dofs = reshape(find(Fhat ~= 0),1,[]);
radial_dofs = reshape(context.radial_dofs,1,[]);
if any(~isfinite(radial_dofs)) || any(radial_dofs ~= floor(radial_dofs)) || ...
        any(radial_dofs < 1 | radial_dofs > n) || numel(unique(radial_dofs)) ~= numel(radial_dofs) || ...
        ~isequal(radial_dofs,force.nonzero_dofs)
    error('StageE:ForceMapping', ...
        'The recovered force support must equal the validated radial DOFs.');
end
recovered_amplitude = max(abs(Fhat(radial_dofs)));
force.force_amplitude_relative_error = abs(recovered_amplitude-force.force_amplitude_N)/ ...
    max(force.force_amplitude_N,eps);
force.force_amplitude_gate_passed = force.force_amplitude_relative_error <= 1e-12;
if isfield(context,'mapping_source'), force.mapping_source = context.mapping_source; else, force.mapping_source = ''; end
end
