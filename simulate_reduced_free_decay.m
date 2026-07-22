function sim = simulate_reduced_free_decay(reference, basis, validation, cfg)
%SIMULATE_REDUCED_FREE_DECAY Fixed-workpoint 22-mode Newmark free decay.

if nargin < 4 || isempty(cfg), cfg = struct(); end
cfg = fixed_config(cfg);
sim = empty_simulation();
validate_inputs(reference, basis, validation);
[M, K, C, G, frequency_tolerance] = reduced_matrices(reference, basis);
assert_matrix_properties(M, K, C, G);

sim.temperature_C = reference.temperature_C;
sim.used_amplitude_m = validation.used_amplitude_m;
sim.gamma = cfg.gamma; sim.beta = cfg.beta;
sim.rotation_speed_rad_s = 2*pi*cfg.rpm/60;
sim.retained_mode_count = basis.retained_mode_count;
sim.selected_dof_labels = {'R2_x','R2_y','R10_x','R10_y','R16_x','R16_y','C2_x','C2_y','C8_x','C8_y'};

G_velocity = sim.rotation_speed_rad_s*G;
C_velocity = C + G_velocity;
[selected_rows, target_row] = find_selected_rows(reference.transverse_dofs);
[eta, eta_dot, eta_ddot, sim.initial_constraint_error_m] = initial_state(M, K, C_velocity, basis.Phi_t, target_row, sim.used_amplitude_m);
sim.initial_acceleration_norm = norm(eta_ddot);
sim.maximum_state_real_part = max(real(eig([zeros(size(M)), eye(size(M)); -M\K, -M\C_velocity])));

f1 = basis.frequency_Hz(1); fmax = basis.frequency_Hz(end);
sim.dt = min(1/(50*fmax), 1/(256*cfg.frot_Hz));
sim.requested_end_time_s = max(20/f1, 3/(2*pi*cfg.target_damping_ratio*f1));
sim.n_internal_steps = ceil(sim.requested_end_time_s/sim.dt);
if sim.n_internal_steps > cfg.maximum_internal_steps
    error('Stage8B:StepBudget', 'Stage8B requires %d internal steps, exceeding the fixed safety limit %d.', sim.n_internal_steps, cfg.maximum_internal_steps);
end
sim.output_stride = max(1, ceil(sim.n_internal_steps/cfg.maximum_saved_steps));
sim.n_saved_steps = floor(sim.n_internal_steps/sim.output_stride) + 1 + (mod(sim.n_internal_steps, sim.output_stride) ~= 0);
if sim.n_saved_steps > cfg.maximum_saved_steps + 2
    error('Stage8B:SaveBudget', 'The downsampled output exceeds the fixed storage limit.');
end

[D_eff, coefficients] = effective_solver(M, K, C_velocity, sim.dt, cfg.gamma, cfg.beta);
sim.factorization_count = 1; sim.newmark_force_call_count = 0; sim.thermal_update_count = 0;
[time, displacement, velocity, acceleration, energy, saved] = allocate_output(sim.n_saved_steps, numel(selected_rows));
[time, displacement, velocity, acceleration, energy, saved] = save_state(time, displacement, velocity, acceleration, energy, saved, 0, eta, eta_dot, eta_ddot, basis.Phi_t(selected_rows,:), M, K);
sim.initial_energy_J = energy(1); previous_energy = sim.initial_energy_J; maximum_internal_energy_growth = 0;

for step = 1:sim.n_internal_steps
    rhs = M*(coefficients.a0*eta + coefficients.a2*eta_dot + coefficients.a3*eta_ddot) + ...
        C_velocity*(coefficients.a1*eta + coefficients.a4*eta_dot + coefficients.a5*eta_ddot);
    eta_next = D_eff\rhs;
    eta_ddot_next = coefficients.a0*(eta_next-eta) - coefficients.a2*eta_dot - coefficients.a3*eta_ddot;
    eta_dot_next = eta_dot + sim.dt*((1-cfg.gamma)*eta_ddot + cfg.gamma*eta_ddot_next);
    current_energy = 0.5*(eta_dot_next.'*M*eta_dot_next + eta_next.'*K*eta_next);
    maximum_internal_energy_growth = max(maximum_internal_energy_growth, max(current_energy-previous_energy, 0));
    previous_energy = current_energy; eta = eta_next; eta_dot = eta_dot_next; eta_ddot = eta_ddot_next;
    if mod(step, sim.output_stride) == 0 || step == sim.n_internal_steps
        [time, displacement, velocity, acceleration, energy, saved] = save_state(time, displacement, velocity, acceleration, energy, saved, step*sim.dt, eta, eta_dot, eta_ddot, basis.Phi_t(selected_rows,:), M, K);
    end
end

sim.time = time(1:saved); sim.displacement = displacement(1:saved,:); sim.velocity = velocity(1:saved,:); sim.acceleration = acceleration(1:saved,:); sim.energy_J = energy(1:saved);
sim.n_saved_steps = saved; sim.final_energy_J = sim.energy_J(end);
sim.maximum_energy_growth_ratio = maximum_internal_energy_growth/max(sim.initial_energy_J, realmin);
sim.maximum_state_real_part = real(sim.maximum_state_real_part);
sim.finite_pass = all(isfinite([sim.time; sim.displacement(:); sim.velocity(:); sim.acceleration(:); sim.energy_J(:)]));
sim.stability_pass = sim.maximum_state_real_part <= 1e-6;
sim.energy_pass = sim.maximum_energy_growth_ratio <= 1e-4 && sim.final_energy_J < sim.initial_energy_J;
sim.pass = sim.finite_pass && sim.stability_pass && sim.energy_pass && sim.initial_constraint_error_m <= 1e-12 && ...
    sim.factorization_count == 1 && sim.newmark_force_call_count == 0 && sim.thermal_update_count == 0 && ...
    sim.n_internal_steps <= cfg.maximum_internal_steps && sim.n_saved_steps <= cfg.maximum_saved_steps + 2 && ...
    basis.frequency_consistency_error <= frequency_tolerance;
end

function cfg = fixed_config(cfg)
defaults = struct('gamma',0.5,'beta',0.25,'rpm',9900,'frot_Hz',165,'target_damping_ratio',0.01,'maximum_internal_steps',500000,'maximum_saved_steps',30000);
names = fieldnames(defaults);
for k = 1:numel(names)
    name = names{k}; if ~isfield(cfg,name) || isempty(cfg.(name)), cfg.(name) = defaults.(name); end
    if ~isequal(cfg.(name),defaults.(name)), error('Stage8B:FixedConfiguration','Stage8B requires cfg.%s = %.16g.',name,defaults.(name)); end
end
end

function sim = empty_simulation()
sim = struct('temperature_C',NaN,'used_amplitude_m',NaN,'gamma',NaN,'beta',NaN,'rotation_speed_rad_s',NaN,'dt',NaN,'requested_end_time_s',NaN,'n_internal_steps',0,'output_stride',0,'n_saved_steps',0,'time',zeros(0,1),'retained_mode_count',0,'selected_dof_labels',{{}},'displacement',zeros(0,0),'velocity',zeros(0,0),'acceleration',zeros(0,0),'initial_constraint_error_m',NaN,'initial_acceleration_norm',NaN,'maximum_state_real_part',NaN,'initial_energy_J',NaN,'final_energy_J',NaN,'maximum_energy_growth_ratio',NaN,'factorization_count',0,'newmark_force_call_count',0,'thermal_update_count',0,'finite_pass',false,'stability_pass',false,'energy_pass',false,'pass',false,'energy_J',zeros(0,1));
end

function validate_inputs(reference,basis,validation)
if ~reference.pass || ~basis.pass || ~validation.pass || basis.retained_mode_count ~= 22 || basis.final_ratio_x < .95 || basis.final_ratio_y < .95 || basis.mass_orthogonality_error > 1e-10 || basis.stiffness_diagonalization_error > 1e-8 || basis.maximum_ritz_backward_error > 1e-10
    error('Stage8B:InputGate','Stage8B requires the passing 20 C reference, basis and Stage7 validation.');
end
if ~isequal(size(basis.M_r),[22 22]) || ~isequal(size(basis.K_r),[22 22]) || ~isequal(size(basis.Phi_t),[128 22])
    error('Stage8B:BasisDimensions','Stage8B requires the fixed 22-mode basis dimensions.');
end
end

function [M,K,C,G,tolerance] = reduced_matrices(reference,basis)
M=basis.M_r; K=basis.K_r; C=basis.C_rayleigh_r+basis.C_foundation_r+basis.C_ehl_r; G=basis.G_r;
if isfield(basis,'frequency_tolerance'), tolerance=basis.frequency_tolerance; elseif isfield(basis,'frequency_consistency_tolerance'), tolerance=basis.frequency_consistency_tolerance; else, error('Stage8B:FrequencyTolerance','The Stage8A-2 residual-aware frequency tolerance is missing.'); end
if abs(reference.rotation_speed_rad_s-2*pi*9900/60) > 100*eps(max(1,abs(reference.rotation_speed_rad_s)))
    error('Stage8B:ReferenceSpeed','The Stage8A reference rotation speed is inconsistent with 9900 rpm.');
end
end

function assert_matrix_properties(M,K,C,G)
if any(~isfinite([M(:);K(:);C(:);G(:)])) || symmetry_error(M)>1e-10 || symmetry_error(K)>1e-10 || symmetry_error(C)>1e-10 || norm(G+G.','fro')/max(norm(G,'fro'),1)>1e-10
    error('Stage8B:MatrixProperties','Reduced matrices must be finite, symmetric where applicable, and G must be skew-symmetric.');
end
[~,pM]=chol(0.5*(M+M.'),'lower'); [~,pK]=chol(0.5*(K+K.'),'lower');
if pM ~= 0 || pK ~= 0
    error('Stage8B:PositiveDefinite','M_r and K_r must be positive definite without regularization.');
end
e=eig(0.5*(C+C.')); if min(e)<-1e-10*max(1,max(e)), error('Stage8B:DampingPSD','C_dissipative_r is not positive semidefinite.'); end
end

function [rows,target] = find_selected_rows(dofs)
global_dofs=[7 8 55 56 91 92 121 122 157 158];
rows=zeros(1,numel(global_dofs)); for k=1:numel(global_dofs), rows(k)=find(dofs==global_dofs(k),1); end
if any(rows==0), error('Stage8B:SelectedDofs','Required R2/R10/R16/C2/C8 transverse DOFs are absent.'); end
target=rows(5);
end

function [eta,v,a,constraint_error] = initial_state(M,K,C,Phi,target,amplitude)
shape=Phi(target,:); A=[K,shape.';shape,0]; solution=A\[zeros(size(K,1),1);amplitude]; eta=solution(1:end-1); v=zeros(size(eta)); constraint_error=abs(shape*eta-amplitude); a=M\(-C*v-K*eta);
end

function [D,coeff] = effective_solver(M,K,C,dt,gamma,beta)
coeff=struct('a0',1/(beta*dt^2),'a1',gamma/(beta*dt),'a2',1/(beta*dt),'a3',1/(2*beta)-1,'a4',gamma/beta-1,'a5',dt*(gamma/(2*beta)-1));
D=decomposition(K+coeff.a0*M+coeff.a1*C,'lu');
end

function [time,x,v,a,E,saved] = allocate_output(n,m)
time=zeros(n,1); x=zeros(n,m); v=zeros(n,m); a=zeros(n,m); E=zeros(n,1); saved=0;
end

function [time,x,v,a,E,saved] = save_state(time,x,v,a,E,saved,t,eta,eta_dot,eta_ddot,Phi,M,K)
saved=saved+1; time(saved)=t; x(saved,:)= (Phi*eta).'; v(saved,:)=(Phi*eta_dot).'; a(saved,:)=(Phi*eta_ddot).'; E(saved)=.5*(eta_dot.'*M*eta_dot+eta.'*K*eta);
end

function value = symmetry_error(A)
value=norm(A-A.','fro')/max(norm(A,'fro'),1);
end
