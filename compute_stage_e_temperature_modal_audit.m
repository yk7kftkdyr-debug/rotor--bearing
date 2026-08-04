function audit = compute_stage_e_temperature_modal_audit(M,K,damping,frequency_1X_Hz,observation)
%COMPUTE_STAGE_E_TEMPERATURE_MODAL_AUDIT Full-order physical modal audit.
% This deliberately matches Stage C's singular-mass condensation procedure.

if ~isscalar(frequency_1X_Hz) || ~isfinite(frequency_1X_Hz) || frequency_1X_Hz <= 0
    error('StageE:ModalAudit','frequency_1X_Hz must be positive and finite.');
end
Phi_data = physical_modes_stage_c(M,K);
frequencies = Phi_data.natural_frequencies_Hz;
Phi = Phi_data.mass_normalized_modes;
observation_dofs = observation_dof_indices(observation);
if any(observation_dofs < 1) || any(observation_dofs > size(Phi,1))
    error('StageE:ModalAudit','Observation DOFs are out of range.');
end
participation = vecnorm(Phi(observation_dofs,:),2,1)./ ...
    max(vecnorm(Phi,2,1),eps);
strict_tolerance_Hz = 100*eps(max(frequency_1X_Hz,1));
below = find(frequencies < frequency_1X_Hz-strict_tolerance_Hz);
above = find(frequencies > frequency_1X_Hz+strict_tolerance_Hz);
if isempty(below) || isempty(above)
    error('StageE:ModalAudit','Physical modes must strictly bracket the 1X frequency.');
end
[~,i_below] = min(frequency_1X_Hz-frequencies(below)); below_index = below(i_below);
[~,i_above] = min(frequencies(above)-frequency_1X_Hz); above_index = above(i_above);
omega = 2*pi*frequencies;
names = {'LOW','NOMINAL','HIGH'};
actual_zeta = struct();
for k = 1:numel(names)
    name = names{k};
    if ~isfield(damping,name) || ~isequal(size(damping.(name)),size(M))
        error('StageE:ModalAudit','Damping must contain full-order LOW, NOMINAL and HIGH matrices.');
    end
    C = damping.(name);
    if any(~isfinite(C),'all')
        error('StageE:ModalAudit','Damping matrices must be finite.');
    end
    actual_zeta.(name) = reshape(diag(Phi.'*C*Phi),1,[])./(2*omega);
end

audit = Phi_data;
audit.first_six_frequency_Hz = frequencies(1:min(6,end));
audit.observation_dofs = observation_dofs;
audit.observation_participation = participation;
audit.actual_zeta = actual_zeta;
audit.nearest_below_165_Hz = modal_endpoint(below_index,frequencies,frequency_1X_Hz,actual_zeta,names);
audit.nearest_above_165_Hz = modal_endpoint(above_index,frequencies,frequency_1X_Hz,actual_zeta,names);
audit.frequency_1X_Hz = frequency_1X_Hz;
end

function endpoint = modal_endpoint(index,frequencies,target,zeta,names)
endpoint = struct('mode_index',index,'frequency_Hz',frequencies(index), ...
    'distance_Hz',abs(frequencies(index)-target),'actual_zeta',struct());
for k = 1:numel(names), endpoint.actual_zeta.(names{k}) = zeta.(names{k})(index); end
end

function dofs = observation_dof_indices(observation)
if isnumeric(observation)
    dofs = unique(reshape(observation,1,[]));
elseif isstruct(observation)
    values = struct2cell(observation);
    dofs = [];
    for k = 1:numel(values)
        if isnumeric(values{k}), dofs = [dofs reshape(values{k},1,[])]; end %#ok<AGROW>
    end
    dofs = unique(dofs);
else
    error('StageE:ModalAudit','Observation mapping must be numeric DOFs or a mapping struct.');
end
if isempty(dofs) || any(dofs ~= floor(dofs)), error('StageE:ModalAudit','Observation DOFs must be integer indices.'); end
end

function modal = physical_modes_stage_c(M,K)
if ~isnumeric(M) || ~isnumeric(K) || ~ismatrix(M) || ~isequal(size(M),size(K)) || ...
        size(M,1) == 0 || any(~isfinite(M),'all') || any(~isfinite(K),'all')
    error('StageE:ModalAudit','M and K must be finite equal-size matrices.');
end
M_s = 0.5*(M+M.'); K_s = 0.5*(K+K.');
[U,mu] = eig(full(M_s),'vector');
if any(~isfinite(U),'all') || any(~isfinite(mu)) || max(abs(imag(U)),[],'all') > 1e-10 || max(abs(imag(mu))) > 1e-10
    error('StageE:ModalAuditBasis','Mass eigensystem must be finite and real.');
end
[mu,order] = sort(real(mu),'ascend'); U = real(U(:,order));
mass_tolerance = max(size(M_s))*eps(max(abs(mu)));
if any(mu < -mass_tolerance), error('StageE:ModalAuditBasis','Mass has a negative eigenvalue.'); end
U0 = U(:,abs(mu) <= mass_tolerance); Up = U(:,mu > mass_tolerance);
mass_rank = size(Up,2); mass_nullity = size(U0,2);
if mass_rank == 0 || mass_rank+mass_nullity ~= size(M,1)
    error('StageE:ModalAuditBasis','Mass subspaces do not support physical modes.');
end
condensation_residual = 0;
if mass_nullity == 0
    X0 = zeros(0,mass_rank);
else
    K00 = 0.5*(U0.'*K_s*U0+(U0.'*K_s*U0).'); K0p = U0.'*K_s*Up;
    [L0,p0] = chol(full(K00),'lower');
    if p0 ~= 0, error('StageE:ModalAuditBasis','Zero-mass stiffness block is not positive definite.'); end
    X0 = -(L0.'\(L0\K0p));
    condensation_residual = norm(K00*X0+K0p,'fro')/max(norm(K0p,'fro'),1);
    if condensation_residual > 1e-10, error('StageE:ModalAuditBasis','Zero-mass condensation residual exceeds 1e-10.'); end
end
T = Up+U0*X0; M_c = 0.5*(T.'*M_s*T+(T.'*M_s*T).'); K_c = 0.5*(T.'*K_s*T+(T.'*K_s*T).');
[Lm,pm] = chol(full(M_c),'lower'); if pm ~= 0, error('StageE:ModalAuditBasis','Condensed mass is not positive definite.'); end
A = Lm\K_c/Lm.'; A = 0.5*(A+A.'); [V,lambda] = eig(full(A),'vector');
if any(~isfinite(V),'all') || any(~isfinite(lambda)) || max(abs(imag(V)),[],'all') > 1e-10 || max(abs(imag(lambda))) > 1e-10
    error('StageE:ModalAuditBasis','Condensed eigensystem must be finite and real.');
end
[lambda,order] = sort(real(lambda),'ascend'); V = real(V(:,order)); Phi = T*(Lm.'\V);
minimum_lambda = max((2*pi*1e-6)^2,max(size(A))*eps(max(1,max(abs(lambda)))));
retained = lambda > minimum_lambda; lambda = lambda(retained); Phi = Phi(:,retained);
for j = 1:size(Phi,2), [~,pivot] = max(abs(Phi(:,j))); if Phi(pivot,j) < 0, Phi(:,j) = -Phi(:,j); end, end
mass_error = norm(Phi.'*M*Phi-eye(size(Phi,2)),inf);
if ~isfinite(mass_error) || mass_error > 1e-8, error('StageE:ModalAuditMassNormalization','Mass normalization failed.'); end
algebraic_residual = norm(U0.'*K*Phi,'fro')/max(norm(K*Phi,'fro'),1);
modal = struct('natural_frequencies_Hz',reshape(sqrt(lambda)/(2*pi),1,[]), ...
    'physical_mode_indices',1:numel(lambda),'mass_normalized_modes',Phi, ...
    'mass_normalization_error',mass_error,'rigid_body_frequency_threshold_Hz',1e-6, ...
    'mass_rank',mass_rank,'mass_nullity',mass_nullity,'mass_tolerance',mass_tolerance, ...
    'zero_mass_condensation_residual',condensation_residual, ...
    'zero_mass_algebraic_residual',algebraic_residual);
end
