function validation = run_stage3_validation(write_sentinel)
%RUN_STAGE3_VALIDATION Terminal-only acceptance for the 19/13-node Stage-3 model.
if nargin < 1, write_sentinel = true; end
params = initial_conditions(); [M,C,K,modelInfo] = build_rotor_case_model(params);
validation.mass_inertia = mass_inertia_check(params, M); validation.matrix_size = [size(M); size(C); size(K)];
validation.mass_symmetry_error = norm(M-M','fro'); validation.stiffness_symmetry_error = norm(K-K','fro'); validation.zero_mass_diagonal = sum(abs(diag(M)) < 1e-14); validation.zero_mass_rows = sum(sum(abs(M),2) < 1e-14);
assert(isequal(size(M),[192 192]) && isequal(size(C),[192 192]) && isequal(size(K),[192 192]), 'Stage 3 matrices must be 192x192.');
assert(validation.mass_symmetry_error <= 1e-12 && validation.stiffness_symmetry_error <= 1e-12 && validation.zero_mass_diagonal == 0 && validation.zero_mass_rows == 0, 'Stage 3 matrix integrity check failed.');
fprintf('Stage3 matrix: M/C/K=192x192, sym(M)=%.3e, sym(K)=%.3e, zeroMass=[%d %d].\n', validation.mass_symmetry_error, validation.stiffness_symmetry_error, validation.zero_mass_diagonal, validation.zero_mass_rows);
validation.gyro = gyro_check(params, M, K, modelInfo);
validation.unit = element_unit_check(params);
static_result = solve_static_equilibrium(params, false); params.static_equilibrium_result = static_result;
validation.zero_dynamic = zero_dynamic_check(params, modelInfo);
validation.transverse = transverse_regression_check(params, modelInfo);
validation.pass = true;
if write_sentinel
    sentinel = fullfile(pwd, 'stage3_pass_ready.txt');
    assert(~isfile(sentinel), 'Refusing to overwrite existing stage3_pass_ready.txt.');
    fid = fopen(sentinel, 'wt'); assert(fid >= 0, 'Cannot create stage3_pass_ready.txt.'); fprintf(fid, 'STAGE3_MASS_INERTIA_CALIBRATION_PASS\n'); fclose(fid);
    fprintf('STAGE3_MASS_INERTIA_CALIBRATION_PASS\n');
end
end

function result = mass_inertia_check(params, M)
expected_nodes = [6 8 16]; expected = [251.194 251.194 251.194 15.17 7.8 7.8; 171.18 171.18 171.18 6.18 3.2 3.2; 146.832 146.832 146.832 7.4 3.78 3.78];
base = params; base.concentrated_mass = struct('nodes',[],'kg',[],'inertia_kgm2',zeros(0,3)); base.disk_mass = struct('node',1,'kg',0,'inertia_kgm2',[0 0 0]); [Mbase,~,~,~] = build_rotor_case_model(base);
result.actual = zeros(3,6); result.relative_error = zeros(3,6);
for k = 1:3
    d = 6*expected_nodes(k)+(-5:0); result.actual(k,:) = diag(M(d,d)-Mbase(d,d)).'; result.relative_error(k,:) = abs(result.actual(k,:)-expected(k,:))./expected(k,:);
    fprintf('Mass/inertia R%d: actual=%s, relative_error=%s\n', expected_nodes(k), mat2str(result.actual(k,:),12), mat2str(result.relative_error(k,:),3));
end
assert(all(result.relative_error(:) < 1e-12), 'Mass/inertia calibration does not meet the 1e-12 relative-error requirement.');
end

function result = gyro_check(params, M, K, modelInfo)
d = 6*params.disk_mass.node+[-2 -1]; mr=M(d,d); kr=K(d,d); gr=modelInfo.GG(d,d);
f0=sort(abs(imag(eig([zeros(2) eye(2); -mr\kr zeros(2)])))/(2*pi)); fh=sort(abs(imag(eig([zeros(2) eye(2); -mr\kr -mr\(params.omega*gr)])))/(2*pi));
result.skew_error=norm(modelInfo.GG+modelInfo.GG','fro'); result.zero_rpm_hz=[f0(1) f0(3)]; result.high_rpm_hz=[fh(1) fh(3)]; result.zero_rpm_separation_hz=abs(diff(result.zero_rpm_hz)); result.high_rpm_separation_hz=abs(diff(result.high_rpm_hz));
fprintf('Gyro: skew=%.3e, 0rpm=[%.6f %.6f]Hz, 9900rpm=[%.6f %.6f]Hz, separation=[%.6f %.6f]Hz.\n', result.skew_error, result.zero_rpm_hz, result.high_rpm_hz, result.zero_rpm_separation_hz, result.high_rpm_separation_hz);
assert(result.skew_error <= 1e-12 && all(result.zero_rpm_hz > 0) && all(result.high_rpm_hz > 0) && result.high_rpm_separation_hz > result.zero_rpm_separation_hz, 'Gyro acceptance check failed.');
end

function result = element_unit_check(params)
L=0.2; p=params; p.node_pos=[0 L]; p.concentrated_mass=struct('nodes',[],'kg',[],'inertia_kgm2',zeros(0,3)); p.disk_mass=struct('node',1,'kg',0,'inertia_kgm2',[0 0 0]); [~,~,K,~]=build_rotor_case_model(p); A=pi/4*(p.shaft_od^2-p.shaft_id^2); J=pi/32*(p.shaft_od^4-p.shaft_id^4); F=1234; T=56;
result.axial_error=abs(F/K(9,9)-F*L/(p.E*A))/(F*L/(p.E*A)); result.torsion_error=abs(T/K(12,12)-T*L/(p.G*J))/(T*L/(p.G*J));
fprintf('Element unit: axial_error=%.3e, torsion_error=%.3e.\n', result.axial_error, result.torsion_error);
assert(result.axial_error <= 1e-12 && result.torsion_error <= 1e-12, 'Axial/torsion unit check failed.');
end

function result = zero_dynamic_check(params, modelInfo)
p=params; p.n_Fen=1; sim=newmark_newton_multi(p); iz=[3:6:modelInfo.num_rotor_dof modelInfo.num_rotor_dof+3:6:modelInfo.num_rotor_dof+modelInfo.num_case_dof]; itz=[6:6:modelInfo.num_rotor_dof modelInfo.num_rotor_dof+6:6:modelInfo.num_rotor_dof+modelInfo.num_case_dof];
result.max_u=max(abs(sim.u(:))); result.max_delta_fb=max(abs(sim.Fb_increment_hist(:))); result.max_uz=max(abs(sim.u(iz,:)),[],'all'); result.max_theta_z=max(abs(sim.u(itz,:)),[],'all'); result.unconverged_steps=sim.solver.unconverged_steps;
fprintf('Zero dynamics: maxU=%.3e, maxDeltaFb=%.3e, maxUz=%.3e, maxThetaZ=%.3e, unconverged=%d.\n', result.max_u, result.max_delta_fb, result.max_uz, result.max_theta_z, result.unconverged_steps);
assert(result.max_u <= 1e-12 && result.max_delta_fb <= 1e-12 && result.max_uz <= 1e-12 && result.max_theta_z <= 1e-12 && result.unconverged_steps == 0, 'Zero-load dynamic check failed.');
end

function result = transverse_regression_check(params, modelInfo)
p=params; p.unbalance.nodes=16; p.unbalance.mass_g=1; p.unbalance.ecc_mm=1; p.unbalance.phase=0; p.startup_ramp_cycles=0; p.n_Fen=1; p.Fen=256; coarse=newmark_newton_multi(p); p.Fen=512; fine=newmark_newton_multi(p); ix=6*16-5; iy=6*16-4; a=response_metrics(coarse,ix,iy,p.omega); b=response_metrics(fine,ix,iy,p.omega);
result.coarse=a; result.fine=b; result.relative_error=abs(b(1:3)-a(1:3))./max(abs(b(1:3)),1e-15); result.phase_difference_deg=atan2(sin(b(4)-a(4)),cos(b(4)-a(4)))*180/pi; iz=[3:6:modelInfo.num_rotor_dof modelInfo.num_rotor_dof+3:6:modelInfo.num_rotor_dof+modelInfo.num_case_dof]; itz=[6:6:modelInfo.num_rotor_dof modelInfo.num_rotor_dof+6:6:modelInfo.num_rotor_dof+modelInfo.num_case_dof]; result.max_uz=max(abs(fine.u(iz,:)),[],'all'); result.max_theta_z=max(abs(fine.u(itz,:)),[],'all'); result.mean_u_norm=norm(mean(fine.u,2)); result.unconverged_steps=[coarse.solver.unconverged_steps fine.solver.unconverged_steps];
fprintf('Transverse fallback: peak/RMS/1X errors=[%.3f%% %.3f%% %.3f%%], phase=%.3fdeg, meanU=%.3e, maxUz=%.3e, maxThetaZ=%.3e.\n', 100*result.relative_error, result.phase_difference_deg, result.mean_u_norm, result.max_uz, result.max_theta_z);
assert(all(result.relative_error < 0.03) && abs(result.phase_difference_deg) < 5 && result.mean_u_norm < 1e-6 && result.max_uz <= 1e-12 && result.max_theta_z <= 1e-12 && all(result.unconverged_steps == 0), 'Transverse dynamic regression check failed.');
end

function a=response_metrics(sim,ix,iy,omega)
z=sim.u(ix,:)+1i*sim.u(iy,:); a=[max(abs(z)) sqrt(mean(abs(z).^2)) abs(sum(z.*exp(-1i*omega*sim.time))/numel(z)) angle(sum(z.*exp(-1i*omega*sim.time)))];
end
