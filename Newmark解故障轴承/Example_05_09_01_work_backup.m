% Example 5.9.1
% Modal analysis for the current single-rotor / dual-bearing / casing model.

clear
format short e
close all

set(0,'defaultaxesfontsize',12)
set(0,'defaultaxesfontname','Times New Roman')
set(0,'defaulttextfontsize',12)
set(0,'defaulttextfontname','Times New Roman')

example_dir = fileparts(mfilename('fullpath'));
parent_dir = fileparts(example_dir);
model_dirs = dir(fullfile(parent_dir,'Newmark*'));
model_dirs = model_dirs([model_dirs.isdir]);
assert(~isempty(model_dirs),'Cannot find the current Newmark model directory.');
current_model_dir = fullfile(parent_dir,model_dirs(1).name);
addpath(current_model_dir)

modal_config = default_modal_config;
[model,modal] = build_modal_model_from_current_system(current_model_dir,modal_config);

%% Rotor picture
figure(1), clf
try
    picrotor(model)
catch ME
    warning('picrotor(model) failed: %s',ME.message);
    plot_simplified_rotor_layout(model,modal);
end

%% Campbell chart and root locus
Rotor_Spd_rpm = 0:500:150000.0;
Rotor_Spd_rpm_all = Rotor_Spd_rpm;
Rotor_Spd = 2*pi*Rotor_Spd_rpm/60;
[eigenvalues,eigenvectors,kappa,full_eigenvectors] = chr_root_current_system(modal,Rotor_Spd);

camp_opts = modal_config.camp_opts;
diag_0 = build_modal_diagnostics(eigenvalues(:,1),full_eigenvectors(:,:,1),modal,camp_opts);
write_modal_diagnostics_csv(diag_0,fullfile(current_model_dir,'results','modal','modal_diagnostics_0rpm.csv'));
candidate_file = fullfile(current_model_dir,'results','modal','campbell_candidate_diagnostics.csv');
export_campbell_candidate_diagnostics(eigenvalues,full_eigenvectors,modal,camp_opts,Rotor_Spd_rpm,candidate_file);
[camp_fb,camp_opts] = track_campbell_forward_backward(eigenvalues,full_eigenvectors,modal,camp_opts,Rotor_Spd_rpm);
branch_file = fullfile(current_model_dir,'results','modal','campbell_branches_tracked.csv');
export_campbell_branches_tracked(camp_fb,branch_file);
crit = empty_report_critical_speeds();
[camp_eigs,camp_kappa,camp_info] = campbell_fb_to_legacy(camp_fb,modal);

figure(2)
NX = 2;
damped_NF = 1;
plotcamp_forward_backward_separated(Rotor_Spd_rpm,camp_fb,camp_opts,crit,modal.run_speed_rpm)
hold on
yl = ylim;
plot([modal.run_speed_rpm modal.run_speed_rpm],yl,'k--','LineWidth',1.0,'HandleVisibility','off')
text(modal.run_speed_rpm,0.04*yl(2),'9900 rpm', ...
    'HorizontalAlignment','left','VerticalAlignment','bottom')
ylim(yl)
hold off
title('Campbell chart - coupled rotor bearing casing system')
saveas(gcf,fullfile(current_model_dir,'results','modal','campbell_corrected.png'))
saveas(gcf,fullfile(current_model_dir,'results','modal','campbell_corrected.svg'))
saveas(gcf,fullfile(current_model_dir,'results','modal','campbell_report.png'))
saveas(gcf,fullfile(current_model_dir,'results','modal','campbell_report.svg'))

figure(3)
plotloci(Rotor_Spd,eigenvalues,NX)
title('Raw root locus - all coupled-system eigenvalues')

figure(7)
plotloci_filtered(Rotor_Spd,camp_eigs,NX)

%% Modes at running speed
Rotor_Spd_rpm = modal.run_speed_rpm;
Rotor_Spd = 2*pi*Rotor_Spd_rpm/60;
[eigenvalues_9900,eigenvectors_9900,kappa_9900,full_eigenvectors_9900] = chr_root_current_system(modal,Rotor_Spd);
mode_diag_9900 = build_modal_diagnostics(eigenvalues_9900,full_eigenvectors_9900,modal,camp_opts);
mode_idx = modal_display_mode_indices(mode_diag_9900,eigenvalues_9900,full_eigenvectors_9900,modal,4,camp_opts);
mode_diag_9900 = mark_selected_modes(mode_diag_9900,camp_info,camp_eigs,Rotor_Spd_rpm_all,modal.run_speed_rpm,mode_idx);
write_modal_diagnostics_csv(mode_diag_9900,fullfile(current_model_dir,'results','modal','modal_diagnostics_9900rpm.csv'));
write_modal_diagnostics_csv(mode_diag_9900,fullfile(current_model_dir,'results','modal','modal_candidate_diagnostics_9900.csv'));
export_modal_frequency_summary(diag_0,mode_diag_9900,camp_fb,fullfile(current_model_dir,'results','modal','modal_frequency_summary.csv'));
report_modes = extract_report_natural_frequencies(diag_0,mode_diag_9900, ...
    fullfile(current_model_dir,'results','modal','natural_frequency_summary.csv'));
undamped_report_modes = extract_undamped_report_natural_frequencies(modal,camp_opts, ...
    fullfile(current_model_dir,'results','modal','undamped_natural_frequency_summary.csv'));
if ~isempty(undamped_report_modes)
    report_modes = undamped_report_modes;
    export_report_modes_to_natural_summary(report_modes, ...
        fullfile(current_model_dir,'results','modal','natural_frequency_summary.csv'));
end
crit = identify_report_critical_speeds(report_modes,camp_fb,camp_opts, ...
    fullfile(current_model_dir,'results','modal','critical_speed_summary.csv'));
validation = validate_campbell_result(camp_fb,crit,camp_opts);
export_campbell_debug_report(validation,camp_fb,camp_opts,current_model_dir);
figure(2)
plotcamp_forward_backward_separated(Rotor_Spd_rpm_all,camp_fb,camp_opts,crit,modal.run_speed_rpm)
hold on
yl = ylim;
plot([modal.run_speed_rpm modal.run_speed_rpm],yl,'k--','LineWidth',1.0,'HandleVisibility','off')
text(modal.run_speed_rpm,0.04*yl(2),'9900 rpm', ...
    'HorizontalAlignment','left','VerticalAlignment','bottom')
ylim(yl)
hold off
title('Campbell chart and critical speed reference of the rotor-bearing-casing system')
saveas(gcf,fullfile(current_model_dir,'results','modal','campbell_report.png'))
saveas(gcf,fullfile(current_model_dir,'results','modal','campbell_report.svg'))

save_report_mode_shapes(model,report_modes,current_model_dir)

figure(5)
clf
outputnode = modal.loc_rub;
axes('position',[0.18 0.56 0.26 0.26])
plotorbit(eigenvectors_9900(:,mode_idx(1)),outputnode,'Mode 1',damped_lambda_for_plot(eigenvalues_9900(mode_idx(1))))
axes('position',[0.56 0.56 0.26 0.26])
plotorbit(eigenvectors_9900(:,mode_idx(2)),outputnode,'Mode 2',damped_lambda_for_plot(eigenvalues_9900(mode_idx(2))))
axes('position',[0.18 0.20 0.26 0.26])
plotorbit(eigenvectors_9900(:,mode_idx(3)),outputnode,'Mode 3',damped_lambda_for_plot(eigenvalues_9900(mode_idx(3))))
axes('position',[0.56 0.20 0.26 0.26])
plotorbit(eigenvectors_9900(:,mode_idx(4)),outputnode,'Mode 4',damped_lambda_for_plot(eigenvalues_9900(mode_idx(4))))

%% Result check
modal_result_check(modal,Rotor_Spd_rpm,Rotor_Spd_rpm_all,eigenvalues_9900,eigenvalues,camp_eigs,camp_info,camp_opts,mode_diag_9900)
compare_modal_model_levels(modal,camp_opts)
export_modal_comparison_summaries(modal,camp_opts,current_model_dir)
export_modal_model_audit(modal,current_model_dir)
export_modal_critical_speed_report(modal,diag_0,mode_diag_9900,camp_fb,crit,validation,current_model_dir)
print_final_modal_critical_summary(mode_diag_9900,crit,validation)


function plot_simplified_rotor_layout(model,modal)
z = model.node(:,end);
plot(z,zeros(size(z)),'k-','LineWidth',1.2)
hold on
plot(z,zeros(size(z)),'ko','MarkerFaceColor','w')
for i = 1:numel(z)
    text(z(i),-0.08,['N' num2str(i)],'HorizontalAlignment','center','Rotation',90)
end
for i = 1:numel(modal.loc_rub)
    node = modal.loc_rub(i);
    plot(z(node),0,'rv','MarkerFaceColor','r','MarkerSize',8)
    text(z(node),0.10,['B' num2str(i)],'Color','r','HorizontalAlignment','center')
end
for i = 1:numel(modal.loca)
    node = modal.loca(i);
    plot(z(node),0,'bs','MarkerFaceColor','y','MarkerSize',8)
    text(z(node),0.18,['D' num2str(i)],'Color','b','HorizontalAlignment','center')
end
ylim([-0.25 0.30])
xlabel('Axial position / m')
title('Simplified rotor layout')
grid on
axis tight
hold off
end


function config = default_modal_config
config.include_disk_inertia = true;
config.case_bearing_nodes = [3 10];
config.bearing_kx = [2.0e8 2.6e8];
config.bearing_ky = [2.0e8 2.6e8];
config.bearing_cx = [1.5e3 1.5e3];
config.bearing_cy = [1.5e3 1.5e3];
config.camp_opts.min_freq_hz = 20;
config.camp_opts.max_freq_hz = 2500;
config.camp_opts.nbranch = 6;
config.camp_opts.norder = 3;
config.camp_opts.reference_rpm = 1000;
config.camp_opts.reference_rpm_fallback = 2000;
config.camp_opts.mac_weight = 0.8;
config.camp_opts.freq_weight = 0.20;
config.camp_opts.rotor_ratio_min = 0.50;
config.camp_opts.rotor_ratio_relaxed = 0.40;
config.camp_opts.zeta_max = 0.60;
config.camp_opts.mode_shape_zeta_max = 0.60;
config.camp_opts.shape_complexity_max = 0.80;
config.camp_opts.shape_complexity_min = 0.03;
config.camp_opts.max_freq_jump_ratio = 0.10;
config.camp_opts.max_freq_jump_hz = 80;
config.camp_opts.min_branch_coverage = 0.80;
config.camp_opts.min_branch_contiguous_coverage = 0.80;
config.camp_opts.min_critical_rpm = 1000;
end


function [model,modal] = build_modal_model_from_current_system(current_model_dir,config)
addpath(current_model_dir)

[~,loca,loc_rub,wi,~,~,data] = initial_conditions;
[N,density,Ef,L,R,RO,miu] = rotor_parameters;
[N_C,density_C,Ef_C,L_C,R_C,RO_C,miu_C] = case_parameters;

[Mst,Msr,Ks,Ge] = Mst_Msr_Ks_Ge(N,density,R,RO,L,Ef,miu);
evalc('[Mr,Gr,Kr0] = M_G_K(N,Ef,R,RO,Mst,Msr,Ge,Ks,miu,L);');

disk_nodes = loca(:).';
disk_masses = [1.4768 2.1790 1.8396 5.6658];
disk_info = estimate_disk_properties(disk_nodes,disk_masses,R,L);
[Mr,Gr] = add_lumped_disk_inertia_and_gyro(Mr,Gr,disk_info,config.include_disk_inertia);
[Mr,mass_fix_rotor] = regularize_mass_for_modal(Mr,'rotor');

is_coupled = data(8) ~= 1;
evalc('[Kr,Cr,~,~,~] = K_D(N,Kr0,Mr,Gr,0,is_coupled);');

[Mst_C,Msr_C,Ks_C,Ge_C] = Mst_Msr_Ks_Ge(N_C,density_C,R_C,RO_C,L_C,Ef_C,miu_C);
evalc('[Mc,Gc,Kc0] = M_G_K(N_C,Ef_C,R_C,RO_C,Mst_C,Msr_C,Ge_C,Ks_C,miu_C,L_C);');
[Mc,mass_fix_case] = regularize_mass_for_modal(Mc,'casing');
evalc('[Kc,Cc,~,~,~] = K_D_case(N_C,Kc0,Mc,zeros(size(Gc)),is_coupled);');

nrotor = size(Mr,1);
ncase = size(Mc,1);
M = blkdiag(Mr,Mc);
K = blkdiag(Kr,Kc);
C = blkdiag(Cr,Cc);
G = blkdiag(Gr,zeros(ncase));

bearing.rotor_nodes = loc_rub(:).';
bearing.case_nodes = config.case_bearing_nodes;
bearing.kx = config.bearing_kx;
bearing.ky = config.bearing_ky;
bearing.cx = config.bearing_cx;
bearing.cy = config.bearing_cy;

[Kbc,Cbc] = assemble_bearing_coupling_modal(nrotor,ncase,bearing);
K = K + Kbc;
C = C + Cbc;

modal.M = M;
modal.K = K;
modal.C = C;
modal.G = G;
modal.Mr = Mr;
modal.Kr = Kr;
modal.Cr = Cr;
modal.Gr = Gr;
modal.Mc = Mc;
modal.Kc = Kc;
modal.Cc = Cc;
modal.Kbc = Kbc;
modal.Cbc = Cbc;
modal.N = N;
modal.N_C = N_C;
modal.ndof_rotor = nrotor;
modal.ndof_case = ncase;
modal.node_count_rotor = N + 1;
modal.loca = loca;
modal.loc_rub = loc_rub;
modal.case_bearing_nodes = bearing.case_nodes;
modal.bearing = bearing;
modal.data = data;
modal.run_speed_rpm = data(7);
modal.run_speed_rad = wi;
modal.disk_nodes = disk_nodes;
modal.disk_masses = disk_masses;
modal.disk_info = disk_info;
modal.include_disk_inertia = config.include_disk_inertia;
modal.mass_fix_rotor = mass_fix_rotor;
modal.mass_fix_case = mass_fix_case;
modal.matrix_check.Mr_min_eig = min(eig((Mr+Mr.')/2));
modal.matrix_check.Mc_min_eig = min(eig((Mc+Mc.')/2));
modal.matrix_check.M_min_eig = min(eig((M+M.')/2));
modal.matrix_check.Kc_min_eig = min(eig((Kc+Kc.')/2));
modal.matrix_check.Mr_zero_dof = find(abs(diag(Mr)) < 1e-12);
modal.matrix_check.Mc_zero_dof = find(abs(diag(Mc)) < 1e-12);
modal.matrix_check.M_zero_dof = find(abs(diag(M)) < 1e-12);

z = [0 cumsum(L)];
modal.z = z(:);
model.node = [(1:N+1).' z(:)];
model.shaft = zeros(N,9);
for i = 1:N
    model.shaft(i,:) = [2 i i+1 2*R(i) 2*RO(i) density Ef(i) 79.3e9 0];
end
model.disc = zeros(numel(disk_nodes),6);
for i = 1:numel(disk_nodes)
    node = disk_nodes(i);
    shaft_od = local_shaft_outer_diameter(node,R);
    disk_od = 1.25*shaft_od;
    local_L = L(max(1,node-1):min(N,node));
    disk_thick = max(0.02,min(0.08,0.5*sum(local_L)/numel(local_L)));
    model.disc(i,:) = [1 node density disk_thick disk_od shaft_od];
end
model.bearing = [3 loc_rub(1) bearing.kx(1) bearing.ky(1) bearing.cx(1) bearing.cy(1); ...
                 3 loc_rub(2) bearing.kx(2) bearing.ky(2) bearing.cx(2) bearing.cy(2)];

assert(size(M,1)==size(K,1) && size(M,1)==size(C,1) && size(M,1)==size(G,1), ...
    'Modal matrices have inconsistent dimensions.');
assert(all(bearing.rotor_nodes==[2 10]), 'Bearing rotor nodes must be [2 10].');
assert(all(bearing.case_nodes==[3 10]), 'Bearing casing nodes must be [3 10].');
end


function disk_info = estimate_disk_properties(disk_nodes,disk_masses,R,L)
disk_info = struct('node',{},'mass',{},'Jd',{},'Jp',{},'outer_radius',{},'inner_radius',{},'thickness',{});
for i = 1:numel(disk_nodes)
    node = disk_nodes(i);
    shaft_ro = local_shaft_outer_diameter(node,R)/2;
    outer_radius = 1.25*shaft_ro;
    inner_radius = shaft_ro;
    local_L = L(max(1,node-1):min(numel(L),node));
    thickness = max(0.02,min(0.08,0.5*sum(local_L)/numel(local_L)));
    mass = disk_masses(i);
    Jp = 0.5*mass*(outer_radius^2 + inner_radius^2);
    Jd = 0.25*mass*(outer_radius^2 + inner_radius^2) + mass*thickness^2/12;
    disk_info(i).node = node;
    disk_info(i).mass = mass;
    disk_info(i).Jd = Jd;
    disk_info(i).Jp = Jp;
    disk_info(i).outer_radius = outer_radius;
    disk_info(i).inner_radius = inner_radius;
    disk_info(i).thickness = thickness;
end
end


function [M,G] = add_lumped_disk_inertia_and_gyro(M,G,disk_info,include_disk_inertia)
for i = 1:numel(disk_info)
    node = disk_info(i).node;
    ix = 4*node - 3;
    iy = 4*node - 2;
    itx = 4*node - 1;
    ity = 4*node;
    M(ix,ix) = M(ix,ix) + disk_info(i).mass;
    M(iy,iy) = M(iy,iy) + disk_info(i).mass;
    if include_disk_inertia
        M(itx,itx) = M(itx,itx) + disk_info(i).Jd;
        M(ity,ity) = M(ity,ity) + disk_info(i).Jd;
        G(itx,ity) = G(itx,ity) + disk_info(i).Jp;
        G(ity,itx) = G(ity,itx) - disk_info(i).Jp;
    end
end
end


function [M,fix_info] = regularize_mass_for_modal(M,name)
M = (M + M.')/2;
[V,D] = eig(M);
d = real(diag(D));
max_d = max(d);
floor_d = 1e-8*max(max_d,1);
fix_info.name = name;
fix_info.min_eig_before = min(d);
fix_info.shift = 0;
if min(d) <= floor_d
    d2 = max(d,floor_d);
    M = V*diag(d2)*V.';
    M = (M + M.')/2;
    fix_info.shift = max(d2-d);
end
end


function shaft_od = local_shaft_outer_diameter(node,R)
n = numel(R);
if node <= 1
    rr = R(1);
elseif node > n
    rr = R(n);
else
    rr = max(R(node-1),R(node));
end
shaft_od = 2*rr;
end


function [Kbc,Cbc] = assemble_bearing_coupling_modal(nrotor,ncase,bearing)
ntotal = nrotor + ncase;
Kbc = zeros(ntotal);
Cbc = zeros(ntotal);

for ib = 1:numel(bearing.rotor_nodes)
    rnode = bearing.rotor_nodes(ib);
    cnode = bearing.case_nodes(ib);

    rdof = [4*rnode-3 4*rnode-2];
    cdof = nrotor + [4*cnode-3 4*cnode-2];

    Kbi = [ bearing.kx(ib) 0 -bearing.kx(ib) 0; ...
            0 bearing.ky(ib) 0 -bearing.ky(ib); ...
           -bearing.kx(ib) 0 bearing.kx(ib) 0; ...
            0 -bearing.ky(ib) 0 bearing.ky(ib)];
    Cbi = [ bearing.cx(ib) 0 -bearing.cx(ib) 0; ...
            0 bearing.cy(ib) 0 -bearing.cy(ib); ...
           -bearing.cx(ib) 0 bearing.cx(ib) 0; ...
            0 -bearing.cy(ib) 0 bearing.cy(ib)];

    dof = [rdof cdof];
    Kbc(dof,dof) = Kbc(dof,dof) + Kbi;
    Cbc(dof,dof) = Cbc(dof,dof) + Cbi;
end
end


function [eigenvalues,eigenvectors,kappa,full_eigenvectors] = chr_root_current_system(modal,Rotor_Spd)
M = modal.M;
K = modal.K;
C = modal.C;
G = modal.G;
n = size(M,1);
nrotor = modal.ndof_rotor;
nspeed = numel(Rotor_Spd);
neig = 2*n;

eigenvalues = zeros(neig,nspeed);
if nspeed == 1
    eigenvectors = zeros(nrotor,neig);
    full_eigenvectors = zeros(n,neig);
    kappa = zeros(nrotor,neig);
else
    eigenvectors = zeros(nrotor,neig,nspeed);
    full_eigenvectors = zeros(n,neig,nspeed);
    kappa = zeros(nrotor,neig,nspeed);
end

Z = zeros(n);
I = eye(n);
for ispd = 1:nspeed
    D = C + Rotor_Spd(ispd)*G;
    A = [Z I; -(M\K) -(M\D)];
    [V,Lam] = eig(A);
    lam = diag(Lam);
    order = modal_eigen_order(lam);
    lam = lam(order);
    V = V(:,order);
    q = V(1:n,:);

    eigenvalues(:,ispd) = lam;
    if nspeed == 1
        eigenvectors(:,:) = q(1:nrotor,:);
        full_eigenvectors(:,:) = q;
        kappa(:,:) = rotor_kappa(q(1:nrotor,:),lam,modal.node_count_rotor);
    else
        eigenvectors(:,:,ispd) = q(1:nrotor,:);
        full_eigenvectors(:,:,ispd) = q;
        kappa(:,:,ispd) = rotor_kappa(q(1:nrotor,:),lam,modal.node_count_rotor);
    end
end
end


function order = modal_eigen_order(lam)
tol = 1e-7;
pos = find(imag(lam) > tol);
neg = find(imag(lam) < -tol);
[~,ipos] = sort(abs(imag(lam(pos))));
pos = pos(ipos);
used = false(size(lam));
order = [];
for ii = 1:numel(pos)
    ip = pos(ii);
    if used(ip)
        continue
    end
    free_neg = neg(~used(neg));
    if isempty(free_neg)
        in = [];
    else
        [~,jj] = min(abs(lam(free_neg) - conj(lam(ip))));
        in = free_neg(jj);
    end
    order = [order; ip]; %#ok<AGROW>
    used(ip) = true;
    if ~isempty(in)
        order = [order; in]; %#ok<AGROW>
        used(in) = true;
    end
end
rest = find(~used);
[~,irest] = sort(abs(lam(rest)));
order = [order; rest(irest)];
end


function kappa = rotor_kappa(qrotor,lam,nnode)
nrotor = 4*nnode;
neig = size(qrotor,2);
kappa = zeros(nrotor,neig);
for imode = 1:neig
    ev = qrotor(:,imode);
    kk_node = whirl(ev(1:4:nrotor),ev(2:4:nrotor));
    if imag(lam(imode)) < 0
        kk_node = -kk_node;
    end
    for inode = 1:nnode
        kappa(4*inode-3:4*inode,imode) = kk_node(inode);
    end
end
end


function [camp_eigs,camp_kappa,camp_info,opts] = track_campbell_branches_mac(eigenvalues,kappa,full_eigenvectors,modal,opts,Rotor_Spd_rpm)
[~,nspeed] = size(eigenvalues);
ndof = size(kappa,1);
camp_eigs = NaN(opts.nbranch,nspeed);
camp_kappa = zeros(ndof,opts.nbranch,nspeed);
camp_info.rotor_ratio = NaN(opts.nbranch,nspeed);
camp_info.damping_ratio = NaN(opts.nbranch,nspeed);
camp_info.selected_idx = NaN(opts.nbranch,nspeed);
camp_info.rejected_low_freq = zeros(1,nspeed);
camp_info.rejected_casing = zeros(1,nspeed);
camp_info.rejected_numeric = zeros(1,nspeed);
camp_info.ntrack = 0;

[~,iref] = min(abs(Rotor_Spd_rpm - opts.reference_rpm));
fprintf('Campbell reference speed = %.0f rpm\n',Rotor_Spd_rpm(iref));
ref_candidates = campbell_candidates(eigenvalues(:,iref),full_eigenvectors(:,:,iref),modal,opts);
if Rotor_Spd_rpm(iref) == 0 && numel(ref_candidates.idx) < min(opts.nbranch,4)
    opts.reference_rpm = 1000;
    [~,iref] = min(abs(Rotor_Spd_rpm - opts.reference_rpm));
    fprintf('Campbell reference speed changed to %.0f rpm because 0 rpm has too few usable split modes.\n',Rotor_Spd_rpm(iref));
    ref_candidates = campbell_candidates(eigenvalues(:,iref),full_eigenvectors(:,:,iref),modal,opts);
end
if numel(ref_candidates.idx) < min(opts.nbranch,4)
    opts_relaxed = opts;
    opts_relaxed.rotor_ratio_min = opts.rotor_ratio_relaxed;
    fprintf('Campbell warning: rotor_ratio_min relaxed from %.2f to %.2f at reference speed.\n', ...
        opts.rotor_ratio_min,opts_relaxed.rotor_ratio_min);
    ref_candidates = campbell_candidates(eigenvalues(:,iref),full_eigenvectors(:,:,iref),modal,opts_relaxed);
    opts = opts_relaxed;
end
ntrack = min(opts.nbranch,numel(ref_candidates.idx));
if ntrack < opts.nbranch
    fprintf('Campbell warning: only %d rotor-dominant branches satisfy the filter at %.0f rpm.\n', ...
        ntrack,Rotor_Spd_rpm(iref));
end
assert(ntrack >= 1,'No rotor-dominant Campbell candidate at the reference speed.');
camp_info.ntrack = ntrack;

ref_idx = ref_candidates.idx(1:ntrack);
camp_eigs(1:ntrack,iref) = eigenvalues(ref_idx,iref);
camp_kappa(:,1:ntrack,iref) = kappa(:,ref_idx,iref);
camp_info.rotor_ratio(1:ntrack,iref) = ref_candidates.rotor_ratio(1:ntrack);
camp_info.damping_ratio(1:ntrack,iref) = damping_ratio_from_lambda(eigenvalues(ref_idx,iref));
camp_info.selected_idx(1:ntrack,iref) = ref_idx(:);
camp_info.rejected_low_freq(iref) = ref_candidates.rejected_low_freq;
camp_info.rejected_casing(iref) = ref_candidates.rejected_casing;
camp_info.rejected_numeric(iref) = ref_candidates.rejected_numeric;
prev_q = ref_candidates.qtrack(:,1:ntrack);
prev_f = ref_candidates.freq(1:ntrack);

for ispd = iref+1:nspeed
    [sel_idx,prev_q,prev_f,sel_ratio] = track_one_speed(prev_q,prev_f, ...
        eigenvalues(:,ispd),full_eigenvectors(:,:,ispd),modal,opts);
    ok = ~isnan(sel_idx);
    camp_eigs(ok,ispd) = eigenvalues(sel_idx(ok),ispd);
    camp_kappa(:,ok,ispd) = kappa(:,sel_idx(ok),ispd);
    camp_info.rotor_ratio(ok,ispd) = sel_ratio(ok);
    camp_info.damping_ratio(ok,ispd) = damping_ratio_from_lambda(eigenvalues(sel_idx(ok),ispd));
    camp_info.selected_idx(ok,ispd) = sel_idx(ok);
    diag_cand = campbell_candidates(eigenvalues(:,ispd),full_eigenvectors(:,:,ispd),modal,opts);
    camp_info.rejected_low_freq(ispd) = diag_cand.rejected_low_freq;
    camp_info.rejected_casing(ispd) = diag_cand.rejected_casing;
    camp_info.rejected_numeric(ispd) = diag_cand.rejected_numeric;
end

prev_q = ref_candidates.qtrack(:,1:ntrack);
prev_f = ref_candidates.freq(1:ntrack);
for ispd = iref-1:-1:1
    [sel_idx,prev_q,prev_f,sel_ratio] = track_one_speed(prev_q,prev_f, ...
        eigenvalues(:,ispd),full_eigenvectors(:,:,ispd),modal,opts);
    ok = ~isnan(sel_idx);
    camp_eigs(ok,ispd) = eigenvalues(sel_idx(ok),ispd);
    camp_kappa(:,ok,ispd) = kappa(:,sel_idx(ok),ispd);
    camp_info.rotor_ratio(ok,ispd) = sel_ratio(ok);
    camp_info.damping_ratio(ok,ispd) = damping_ratio_from_lambda(eigenvalues(sel_idx(ok),ispd));
    camp_info.selected_idx(ok,ispd) = sel_idx(ok);
    diag_cand = campbell_candidates(eigenvalues(:,ispd),full_eigenvectors(:,:,ispd),modal,opts);
    camp_info.rejected_low_freq(ispd) = diag_cand.rejected_low_freq;
    camp_info.rejected_casing(ispd) = diag_cand.rejected_casing;
    camp_info.rejected_numeric(ispd) = diag_cand.rejected_numeric;
end
end


function cand = campbell_candidates(lam,q_all,modal,opts)
freq = abs(imag(lam))/(2*pi);
pos_idx = find(imag(lam) > 1e-7);
cand.rejected_low_freq = sum(freq(pos_idx) < opts.min_freq_hz);
freq_idx = pos_idx(freq(pos_idx) >= opts.min_freq_hz & freq(pos_idx) <= opts.max_freq_hz);

rotor_ratio = zeros(numel(freq_idx),1);
qtrack = zeros(2*modal.node_count_rotor,numel(freq_idx));
numeric_bad = false(numel(freq_idx),1);
zeta = damping_ratio_from_lambda(lam(freq_idx));
shape_complexity = zeros(numel(freq_idx),1);
for i = 1:numel(freq_idx)
    q = q_all(:,freq_idx(i));
    qrot = q(1:modal.ndof_rotor);
    qcase = q(modal.ndof_rotor+1:end);
    xy_idx = rotor_xy_indices(modal.node_count_rotor);
    qxy = qrot(xy_idx);
    case_xy_idx = rotor_xy_indices(modal.N_C+1);
    qcxy = qcase(case_xy_idx);
    rotor_ratio(i) = norm(qxy)^2/max(norm(qxy)^2 + norm(qcxy)^2,eps);
    qtrack(:,i) = qxy;
    shape_complexity(i) = rotor_shape_complexity(qxy,modal.node_count_rotor,modal.z);
    numeric_bad(i) = ~isfinite(freq(freq_idx(i))) || norm(qxy) < 1e-12;
end
ratio_bad = rotor_ratio <= opts.rotor_ratio_min;
zeta_bad = zeta(:) > opts.zeta_max;
shape_bad = shape_complexity > opts.shape_complexity_max;
cand.rejected_casing = sum(ratio_bad);
cand.rejected_numeric = sum(numeric_bad | zeta_bad | shape_bad);
keep = ~ratio_bad & ~numeric_bad & ~zeta_bad & ~shape_bad;
idx = freq_idx(keep);
rotor_ratio = rotor_ratio(keep);
qtrack = qtrack(:,keep);
shape_complexity = shape_complexity(keep);
zeta = zeta(keep);
freq_sel = freq(idx);
[~,isort] = sort(freq_sel);
idx = idx(isort);
freq_sel = freq_sel(isort);
rotor_ratio = rotor_ratio(isort);
qtrack = qtrack(:,isort);
shape_complexity = shape_complexity(isort);
zeta = zeta(isort);

cand.idx = idx;
cand.freq = freq_sel(:).';
cand.qtrack = qtrack;
cand.rotor_ratio = rotor_ratio(:).';
cand.shape_complexity = shape_complexity(:).';
cand.zeta = zeta(:).';
end


function [sel_idx,new_q,new_f,sel_ratio] = track_one_speed(prev_q,prev_f,lam,q_all,modal,opts)
nbranch = size(prev_q,2);
sel_idx = NaN(nbranch,1);
new_q = prev_q;
new_f = prev_f;
sel_ratio = NaN(nbranch,1);
cand = campbell_candidates(lam,q_all,modal,opts);
if isempty(cand.idx)
    return
end
used = false(1,numel(cand.idx));
for ib = 1:nbranch
    costs = inf(1,numel(cand.idx));
    for ic = 1:numel(cand.idx)
        if used(ic)
            continue
        end
        mac = complex_mac(prev_q(:,ib),cand.qtrack(:,ic));
        freq_jump = abs(log(max(cand.freq(ic),eps)/max(prev_f(ib),eps)));
        costs(ic) = opts.mac_weight*(1-mac) + opts.freq_weight*freq_jump;
    end
    [best_cost,ic_best] = min(costs);
    df = abs(cand.freq(ic_best)-prev_f(ib));
    max_jump = min(opts.max_freq_jump_hz,opts.max_freq_jump_ratio*max(prev_f(ib),cand.freq(ic_best)));
    if isfinite(best_cost) && df <= max_jump
        sel_idx(ib) = cand.idx(ic_best);
        new_q(:,ib) = cand.qtrack(:,ic_best);
        new_f(ib) = cand.freq(ic_best);
        sel_ratio(ib) = cand.rotor_ratio(ic_best);
        used(ic_best) = true;
    end
end
end


function idx = rotor_xy_indices(nnode)
idx = zeros(2*nnode,1);
idx(1:2:end) = 1:4:(4*nnode);
idx(2:2:end) = 2:4:(4*nnode);
end


function c = rotor_shape_complexity(qxy,nnode,z)
qx = qxy(1:2:end);
qy = qxy(2:2:end);
if numel(qx) < 3
    c = 0;
else
    if nargin < 3 || isempty(z)
        z = (1:nnode).';
    else
        z = z(:);
    end
    comp = [real(qx(:)) imag(qx(:)) real(qy(:)) imag(qy(:))];
    spread = max(comp,[],1) - min(comp,[],1);
    [~,icol] = max(spread);
    def = comp(:,icol);
    p = polyfit(z,def,1);
    fit = polyval(p,z);
    c = norm(def-fit)/max(norm(def),eps);
end
end


function zeta = damping_ratio_from_lambda(lam)
zeta = -real(lam)./max(abs(lam),eps);
end


function mac = complex_mac(a,b)
mac = abs(a'*b)^2/max(real(a'*a)*real(b'*b),eps);
mac = min(max(real(mac),0),1);
end


function export_campbell_candidate_diagnostics(eigenvalues,full_eigenvectors,modal,opts,Rotor_Spd_rpm,outfile)
outdir = fileparts(outfile);
if ~exist(outdir,'dir'), mkdir(outdir); end
fid = fopen(outfile,'w');
assert(fid > 0,'Cannot write Campbell candidate diagnostics.');
fprintf(fid,'rpm,mode_index,frequency_Hz,damping_ratio,rotor_energy_ratio,casing_energy_ratio,shape_complexity,whirl_direction,mode_type,selected_for_campbell\n');
for ispd = 1:numel(Rotor_Spd_rpm)
    cand = campbell_candidates_fb(eigenvalues(:,ispd),full_eigenvectors(:,:,ispd),modal,opts,false);
    for i = 1:numel(cand.idx)
        selected = strcmp(cand.mode_type{i},'rotor_bending') && ...
            (strcmp(cand.direction{i},'forward') || strcmp(cand.direction{i},'backward'));
        fprintf(fid,'%.0f,%d,%.10g,%.10g,%.10g,%.10g,%.10g,%s,%s,%d\n', ...
            Rotor_Spd_rpm(ispd),cand.idx(i),cand.freq(i),cand.zeta(i), ...
            cand.rotor_ratio(i),cand.casing_ratio(i),cand.shape_complexity(i), ...
            cand.direction{i},cand.mode_type{i},selected);
    end
end
fclose(fid);
fprintf('Campbell candidate diagnostics written to: %s\n',outfile);
end


function cand = campbell_candidates_fb(lam,q_all,modal,opts,apply_filter)
if nargin < 5, apply_filter = true; end
freq = abs(imag(lam))/(2*pi);
pos_idx = find(imag(lam) > 1e-7);
base_idx = pos_idx(freq(pos_idx) >= opts.min_freq_hz & freq(pos_idx) <= opts.max_freq_hz);
n = numel(base_idx);
xy_idx = rotor_xy_indices(modal.node_count_rotor);
case_xy_idx = rotor_xy_indices(modal.N_C+1);
cand.idx = base_idx(:).';
cand.freq = zeros(1,n);
cand.zeta = zeros(1,n);
cand.rotor_ratio = zeros(1,n);
cand.casing_ratio = zeros(1,n);
cand.shape_complexity = zeros(1,n);
cand.direction = cell(1,n);
cand.mode_type = cell(1,n);
cand.qtrack = zeros(numel(xy_idx),n);
for i = 1:n
    imode = base_idx(i);
    q = q_all(:,imode);
    qrot = q(1:modal.ndof_rotor);
    qcase = q(modal.ndof_rotor+1:end);
    qxy = qrot(xy_idx);
    qcxy = qcase(case_xy_idx);
    rotor_energy = norm(qxy)^2;
    casing_energy = norm(qcxy)^2;
    rr = rotor_energy/max(rotor_energy+casing_energy,eps);
    cr = casing_energy/max(rotor_energy+casing_energy,eps);
    zeta = -real(lam(imode))/max(abs(lam(imode)),eps);
    sc = rotor_shape_complexity(qxy,modal.node_count_rotor,modal.z);
    cand.freq(i) = freq(imode);
    cand.zeta(i) = zeta;
    cand.rotor_ratio(i) = rr;
    cand.casing_ratio(i) = cr;
    cand.shape_complexity(i) = sc;
    cand.direction{i} = whirl_direction_from_rotor_xy(qxy,modal.node_count_rotor);
    cand.mode_type{i} = classify_mode(freq(imode),zeta,rr,cr,sc,opts);
    cand.qtrack(:,i) = qxy;
end
if apply_filter
    keep = strcmp(cand.mode_type,'rotor_bending') & ...
        (strcmp(cand.direction,'forward') | strcmp(cand.direction,'backward')) & ...
        cand.rotor_ratio >= opts.rotor_ratio_min & cand.casing_ratio <= cand.rotor_ratio & ...
        cand.zeta >= -0.02 & cand.zeta <= opts.zeta_max & ...
        cand.shape_complexity <= opts.shape_complexity_max;
    cand = subset_campbell_candidates(cand,keep);
end
[~,isort] = sort(cand.freq);
cand = subset_campbell_candidates(cand,isort);
end


function cand = subset_campbell_candidates(cand,sel)
cand.idx = cand.idx(sel);
cand.freq = cand.freq(sel);
cand.zeta = cand.zeta(sel);
cand.rotor_ratio = cand.rotor_ratio(sel);
cand.casing_ratio = cand.casing_ratio(sel);
cand.shape_complexity = cand.shape_complexity(sel);
cand.direction = cand.direction(sel);
cand.mode_type = cand.mode_type(sel);
cand.qtrack = cand.qtrack(:,sel);
end


function [camp,opts] = track_campbell_forward_backward(eigenvalues,full_eigenvectors,modal,opts,Rotor_Spd_rpm)
norder = opts.norder;
nspeed = numel(Rotor_Spd_rpm);
directions = {'forward','backward'};
camp.rpm = Rotor_Spd_rpm(:).';
camp.freq = NaN(norder,2,nspeed);
camp.eig = NaN(norder,2,nspeed);
camp.mode_index = NaN(norder,2,nspeed);
camp.rotor_ratio = NaN(norder,2,nspeed);
camp.damping_ratio = NaN(norder,2,nspeed);
camp.mac_to_previous = NaN(norder,2,nspeed);
camp.connected_to_previous = false(norder,2,nspeed);
camp.direction = directions;
camp.qtrack = cell(norder,2,nspeed);

[~,iref] = min(abs(Rotor_Spd_rpm - opts.reference_rpm));
ref_cand = campbell_candidates_fb(eigenvalues(:,iref),full_eigenvectors(:,:,iref),modal,opts,true);
if min(direction_count(ref_cand,'forward'),direction_count(ref_cand,'backward')) < norder
    [~,iref2] = min(abs(Rotor_Spd_rpm - opts.reference_rpm_fallback));
    ref_cand2 = campbell_candidates_fb(eigenvalues(:,iref2),full_eigenvectors(:,:,iref2),modal,opts,true);
    if min(direction_count(ref_cand2,'forward'),direction_count(ref_cand2,'backward')) >= ...
            min(direction_count(ref_cand,'forward'),direction_count(ref_cand,'backward'))
        iref = iref2;
        ref_cand = ref_cand2;
        opts.reference_rpm = Rotor_Spd_rpm(iref);
    end
end
if min(direction_count(ref_cand,'forward'),direction_count(ref_cand,'backward')) < norder
    opts.rotor_ratio_min = opts.rotor_ratio_relaxed;
    ref_cand = campbell_candidates_fb(eigenvalues(:,iref),full_eigenvectors(:,:,iref),modal,opts,true);
    fprintf('Campbell warning: rotor_energy_ratio threshold relaxed to %.2f for separated tracking.\n',opts.rotor_ratio_min);
end
fprintf('Separated Campbell reference speed = %.0f rpm\n',Rotor_Spd_rpm(iref));

for idir = 1:2
    dir = directions{idir};
    idx_dir = find(strcmp(ref_cand.direction,dir));
    [~,isort] = sort(ref_cand.freq(idx_dir));
    idx_dir = idx_dir(isort);
    for io = 1:min(norder,numel(idx_dir))
        ic = idx_dir(io);
        set_camp_point(io,idir,iref,ref_cand,ic,true,1.0);
    end
end

for idir = 1:2
    for step_dir = [1 -1]
        if step_dir > 0
            speed_range = iref+1:nspeed;
        else
            speed_range = iref-1:-1:1;
        end
        prev_q = cell(norder,1);
        prev_f = NaN(norder,1);
        for io = 1:norder
            prev_q{io} = camp.qtrack{io,idir,iref};
            prev_f(io) = camp.freq(io,idir,iref);
        end
        for ispd = speed_range
            cand = campbell_candidates_fb(eigenvalues(:,ispd),full_eigenvectors(:,:,ispd),modal,opts,true);
            cand = subset_campbell_candidates(cand,strcmp(cand.direction,directions{idir}));
            [sel,macv] = match_campbell_speed(prev_q,prev_f,cand,opts);
            for io = 1:norder
                if ~isnan(sel(io))
                    set_camp_point(io,idir,ispd,cand,sel(io),true,macv(io));
                    prev_q{io} = cand.qtrack(:,sel(io));
                    prev_f(io) = cand.freq(sel(io));
                else
                    prev_q{io} = [];
                    prev_f(io) = NaN;
                end
            end
        end
    end
end

    function set_camp_point(io,idir,ispd,cand,ic,connected,macv)
        camp.freq(io,idir,ispd) = cand.freq(ic);
        camp.eig(io,idir,ispd) = eigenvalues(cand.idx(ic),ispd);
        camp.mode_index(io,idir,ispd) = cand.idx(ic);
        camp.rotor_ratio(io,idir,ispd) = cand.rotor_ratio(ic);
        camp.damping_ratio(io,idir,ispd) = cand.zeta(ic);
        camp.mac_to_previous(io,idir,ispd) = macv;
        camp.connected_to_previous(io,idir,ispd) = connected;
        camp.qtrack{io,idir,ispd} = cand.qtrack(:,ic);
    end
end


function n = direction_count(cand,dir)
n = sum(strcmp(cand.direction,dir));
end


function [sel,macv] = match_campbell_speed(prev_q,prev_f,cand,opts)
norder = numel(prev_q);
sel = NaN(norder,1);
macv = NaN(norder,1);
if isempty(cand.idx), return, end
pairs = [];
for io = 1:norder
    if isempty(prev_q{io}) || ~isfinite(prev_f(io)), continue, end
    for ic = 1:numel(cand.idx)
        mac = complex_mac(prev_q{io},cand.qtrack(:,ic));
        df = abs(cand.freq(ic)-prev_f(io));
        max_jump = min(opts.max_freq_jump_hz,opts.max_freq_jump_ratio*max(prev_f(io),cand.freq(ic)));
        if df > max_jump, continue, end
        cost = opts.mac_weight*(1-mac) + opts.freq_weight*abs(log(max(cand.freq(ic),eps)/max(prev_f(io),eps)));
        pairs = [pairs; cost io ic mac]; %#ok<AGROW>
    end
end
if isempty(pairs), return, end
[~,isort] = sort(pairs(:,1));
pairs = pairs(isort,:);
used_order = false(norder,1);
used_candidate = false(numel(cand.idx),1);
for ip = 1:size(pairs,1)
    io = pairs(ip,2);
    ic = pairs(ip,3);
    if ~used_order(io) && ~used_candidate(ic)
        sel(io) = ic;
        macv(io) = pairs(ip,4);
        used_order(io) = true;
        used_candidate(ic) = true;
    end
end
end


function export_campbell_branches_tracked(camp,outfile)
outdir = fileparts(outfile);
if ~exist(outdir,'dir'), mkdir(outdir); end
fid = fopen(outfile,'w');
assert(fid > 0,'Cannot write tracked Campbell branches.');
fprintf(fid,'rpm,order,direction,frequency_Hz,mode_index,rotor_energy_ratio,damping_ratio,MAC_to_previous,connected_to_previous\n');
for ispd = 1:numel(camp.rpm)
    for idir = 1:2
        for io = 1:size(camp.freq,1)
            fprintf(fid,'%.0f,%d,%s,%.10g,%.0f,%.10g,%.10g,%.10g,%d\n', ...
                camp.rpm(ispd),io,camp.direction{idir},camp.freq(io,idir,ispd), ...
                camp.mode_index(io,idir,ispd),camp.rotor_ratio(io,idir,ispd), ...
                camp.damping_ratio(io,idir,ispd),camp.mac_to_previous(io,idir,ispd), ...
                camp.connected_to_previous(io,idir,ispd));
        end
    end
end
fclose(fid);
fprintf('Tracked Campbell branches written to: %s\n',outfile);
end


function crit = identify_critical_speeds_from_campbell_fb(camp,opts,outfile)
crit.points = repmat(struct('critical_order',NaN,'branch_order',NaN,'excitation_order',NaN, ...
    'rpm',NaN,'freq',NaN,'branch_direction','forward','reliable',false),3,1);
found = 0;
for exc_order = 1:2
    exc = exc_order*camp.rpm/60;
    for io = 1:size(camp.freq,1)
        f = squeeze(camp.freq(io,1,:)).';
        valid = isfinite(f);
        if sum(valid)/numel(valid) < opts.min_branch_coverage, continue, end
        d = f - exc;
        for k = 1:numel(camp.rpm)-1
            if ~valid(k) || ~valid(k+1), continue, end
            if d(k) == 0 || d(k)*d(k+1) <= 0
                rpm = interp1(d(k:k+1),camp.rpm(k:k+1),0,'linear','extrap');
                if isfinite(rpm) && rpm >= opts.min_critical_rpm
                    found = found + 1;
                    if found <= 3
                        crit.points(found).critical_order = found;
                        crit.points(found).branch_order = io;
                        crit.points(found).excitation_order = exc_order;
                        crit.points(found).rpm = rpm;
                        crit.points(found).freq = exc_order*rpm/60;
                        crit.points(found).reliable = true;
                    end
                    break
                end
            end
        end
    end
end
export_campbell_critical_speeds(crit,outfile);
fprintf('\n===== 临界转速识别结果 =====\n');
for i = 1:numel(crit.points)
    if isfinite(crit.points(i).rpm)
        fprintf('临界/交点 %d：branch %d, %.0fX, %.1f rpm, %.3f Hz\n', ...
            i,crit.points(i).branch_order,crit.points(i).excitation_order, ...
            crit.points(i).rpm,crit.points(i).freq);
    else
        fprintf('临界/交点 %d：无法可靠识别\n',i);
    end
end
primary = find([crit.points.excitation_order] == 1 & isfinite([crit.points.rpm]),1,'first');
if isempty(primary)
    fprintf('当前工作转速是否低于一阶1X临界转速：无法判断\n');
else
    fprintf('当前工作转速是否低于一阶1X临界转速：%s\n',yes_no(9900 < crit.points(primary).rpm));
end
fprintf('================================\n\n');
end


function export_campbell_critical_speeds(crit,outfile)
outdir = fileparts(outfile);
if ~exist(outdir,'dir'), mkdir(outdir); end
fid = fopen(outfile,'w');
assert(fid > 0,'Cannot write Campbell critical speeds.');
fprintf(fid,'critical_order,branch_order,excitation_order,critical_rpm,critical_frequency_Hz,branch_direction,reliable\n');
for i = 1:numel(crit.points)
    fprintf(fid,'%.0f,%.0f,%.0f,%.10g,%.10g,%s,%d\n', ...
        crit.points(i).critical_order,crit.points(i).branch_order,crit.points(i).excitation_order, ...
        crit.points(i).rpm,crit.points(i).freq,crit.points(i).branch_direction,crit.points(i).reliable);
end
fclose(fid);
fprintf('Campbell critical speeds written to: %s\n',outfile);
end


function export_critical_speed_summary(crit,outfile)
outdir = fileparts(outfile);
if ~exist(outdir,'dir'), mkdir(outdir); end
fid = fopen(outfile,'w');
assert(fid > 0,'Cannot write critical speed summary.');
fprintf(fid,'critical_order,branch_order,mode_family,critical_rpm,critical_frequency_Hz,excitation_order,branch_direction,mode_type,rotor_energy_ratio_at_crossing,reliable,reason\n');
for i = 1:numel(crit.points)
    if isfinite(crit.points(i).rpm)
        reason = 'reliable forward branch 1X/2X intersection from coupled model';
        mode_family = sprintf('forward_branch_%d',crit.points(i).branch_order);
        mode_type = 'rotor_bending';
        rr = NaN;
    else
        reason = 'not identified within tracked reliable forward branches';
        mode_family = '';
        mode_type = '';
        rr = NaN;
    end
    fprintf(fid,'%.0f,%.0f,%s,%.10g,%.10g,%.0f,%s,%s,%.10g,%d,%s\n', ...
        i,crit.points(i).branch_order,mode_family,crit.points(i).rpm,crit.points(i).freq, ...
        crit.points(i).excitation_order,crit.points(i).branch_direction,mode_type,rr, ...
        crit.points(i).reliable,reason);
end
fclose(fid);
fprintf('Critical speed summary written to: %s\n',outfile);
end


function crit = empty_report_critical_speeds()
crit.points = repmat(struct('critical_order',NaN,'branch_order',NaN,'excitation_order',1, ...
    'rpm',NaN,'freq',NaN,'branch_direction','forward','reliable',false, ...
    'identification_method','','natural_frequency_Hz',NaN,'mode_index',NaN, ...
    'mode_type','','rotor_energy_ratio',NaN,'casing_energy_ratio',NaN, ...
    'MAC_to_natural_mode',NaN,'reason',''),3,1);
end


function export_modal_frequency_summary(diag0,diag9900,camp,outfile)
outdir = fileparts(outfile);
if ~exist(outdir,'dir'), mkdir(outdir); end
fid = fopen(outfile,'w');
assert(fid > 0,'Cannot write modal frequency summary.');
fprintf(fid,'mode_order,mode_index,frequency_Hz,frequency_type,whirl_direction,rotor_energy_ratio,casing_energy_ratio,bearing_relative_motion_ratio,shape_complexity,mode_type,used_as_natural_frequency,reason\n');

elastic = diag0([diag0.frequency_Hz] > 1 & ~strcmp({diag0.mode_type},'numerical_or_unstable'));
[~,isort] = sort([elastic.frequency_Hz]);
elastic = elastic(isort);
for i = 1:min(3,numel(elastic))
    write_summary_row(fid,i,elastic(i),'coupled_system_low_order',0,'low-order coupled-system positive-frequency mode');
end

rotor = diag9900(strcmp({diag9900.mode_type},'rotor_bending'));
[~,isort] = sort([rotor.frequency_Hz]);
rotor = rotor(isort);
for i = 1:min(3,numel(rotor))
    write_summary_row(fid,i,rotor(i),'rotor_dominant_bending',1,'selected as rotor-dominant bending natural frequency at running speed');
end

for io = 1:min(3,size(camp.freq,1))
    f = squeeze(camp.freq(io,1,:));
    valid = find(isfinite(f),1,'first');
    if ~isempty(valid)
        dummy = struct('mode_index',camp.mode_index(io,1,valid),'frequency_Hz',f(valid), ...
            'whirl_direction','forward','rotor_energy_ratio',camp.rotor_ratio(io,1,valid), ...
            'casing_energy_ratio',NaN,'bearing_relative_motion_ratio',NaN, ...
            'shape_complexity',NaN,'mode_type','rotor_bending');
        write_summary_row(fid,io,dummy,'campbell_forward_branch',0,'forward branch used for critical-speed search');
    end
end
fclose(fid);
fprintf('Modal frequency summary written to: %s\n',outfile);
end


function write_summary_row(fid,order,row,ftype,used,reason)
fprintf(fid,'%d,%.0f,%.10g,%s,%s,%.10g,%.10g,%.10g,%.10g,%s,%d,%s\n', ...
    order,row.mode_index,row.frequency_Hz,ftype,row.whirl_direction, ...
    row.rotor_energy_ratio,row.casing_energy_ratio,row.bearing_relative_motion_ratio, ...
    row.shape_complexity,row.mode_type,used,reason);
end


function report_modes = extract_report_natural_frequencies(diag0,diag9900,outfile)
selected = [];
used_idx = [];
priority_sets = {
    'rotor_dominant_bending', diag9900(strcmp({diag9900.mode_type},'rotor_bending') & [diag9900.rotor_energy_ratio] >= 0.50), ...
        'rotor_bending with rotor_energy_ratio >= 0.50';
    'coupled_rotor_bending', diag9900(strcmp({diag9900.mode_type},'coupled_rotor_bending') & [diag9900.rotor_energy_ratio] >= 0.30), ...
        'coupled_rotor_bending with rotor participation and bearing relative motion';
    'coupled_system_low_order', diag0(~strcmp({diag0.mode_type},'numerical_or_unstable') & ...
        ~strcmp({diag0.mode_type},'casing_local') & [diag0.frequency_Hz] >= 1.0), ...
        'low-order coupled-system elastic mode, not pure rotor bending'
    };
for ip = 1:size(priority_sets,1)
    rows = priority_sets{ip,2};
    [~,isort] = sort([rows.frequency_Hz]);
    rows = rows(isort);
    for i = 1:numel(rows)
        if numel(selected) >= 3, break, end
        if any(used_idx == rows(i).mode_index), continue, end
        rows(i).frequency_source = priority_sets{ip,1};
        rows(i).reason = priority_sets{ip,3};
        rows(i).used_for_report = 1;
        selected = [selected rows(i)]; %#ok<AGROW>
        used_idx = [used_idx rows(i).mode_index]; %#ok<AGROW>
    end
end
report_modes = selected;
outdir = fileparts(outfile);
if ~exist(outdir,'dir'), mkdir(outdir); end
fid = fopen(outfile,'w');
assert(fid > 0,'Cannot write natural frequency summary.');
fprintf(fid,'report_mode_order,mode_index,frequency_Hz,frequency_source,mode_type,rotor_energy_ratio,casing_energy_ratio,bearing_relative_motion_ratio,shape_complexity,whirl_direction,used_for_report,reason\n');
for i = 1:numel(report_modes)
    fprintf(fid,'%d,%.0f,%.10g,%s,%s,%.10g,%.10g,%.10g,%.10g,%s,%d,%s\n', ...
        i,report_modes(i).mode_index,report_modes(i).frequency_Hz, ...
        report_modes(i).frequency_source,report_modes(i).mode_type, ...
        report_modes(i).rotor_energy_ratio,report_modes(i).casing_energy_ratio, ...
        report_modes(i).bearing_relative_motion_ratio,report_modes(i).shape_complexity, ...
        report_modes(i).whirl_direction,report_modes(i).used_for_report,report_modes(i).reason);
end
fclose(fid);
fprintf('Natural frequency summary written to: %s\n',outfile);
end


function crit = identify_report_critical_speeds(report_modes,camp,opts,outfile)
crit.points = repmat(struct('critical_order',NaN,'branch_order',NaN,'excitation_order',1, ...
    'rpm',NaN,'freq',NaN,'branch_direction','forward','reliable',false, ...
    'identification_method','','natural_frequency_Hz',NaN,'mode_index',NaN, ...
    'mode_type','','rotor_energy_ratio',NaN,'casing_energy_ratio',NaN, ...
    'MAC_to_natural_mode',NaN,'reason',''),3,1);
global_candidates = global_forward_1x_intersections(camp,opts);
for i = 1:min(3,numel(report_modes))
    f_nat = report_modes(i).frequency_Hz;
    rpm_guess = 60*f_nat;
    crit.points(i).critical_order = i;
    crit.points(i).natural_frequency_Hz = f_nat;
    crit.points(i).mode_index = report_modes(i).mode_index;
    crit.points(i).mode_type = report_modes(i).mode_type;
    crit.points(i).rotor_energy_ratio = report_modes(i).rotor_energy_ratio;
    crit.points(i).casing_energy_ratio = report_modes(i).casing_energy_ratio;
    match = identify_local_critical_speed_near_mode(report_modes(i),camp,opts);
    if ~isempty(match)
        crit.points(i).branch_order = match.branch_order;
        crit.points(i).rpm = match.rpm;
        crit.points(i).freq = match.freq;
        crit.points(i).reliable = match.reliable;
        crit.points(i).identification_method = match.method;
        crit.points(i).MAC_to_natural_mode = match.mac;
        crit.points(i).reason = match.reason;
        continue
    end
    match = [];
    if ~isempty(global_candidates)
        [dmin,im] = min(abs([global_candidates.rpm] - rpm_guess));
        if dmin <= max(1500,0.35*rpm_guess)
            match = global_candidates(im);
        end
    end
    if ~isempty(match)
        crit.points(i).branch_order = match.branch_order;
        crit.points(i).rpm = match.rpm;
        crit.points(i).freq = match.freq;
        crit.points(i).reliable = true;
        crit.points(i).identification_method = 'global_campbell_intersection';
        crit.points(i).reason = 'reliable forward branch and 1X excitation intersection';
    else
        crit.points(i).branch_order = i;
        crit.points(i).rpm = rpm_guess;
        crit.points(i).freq = f_nat;
        crit.points(i).reliable = false;
        crit.points(i).identification_method = 'modal_frequency_estimate';
        crit.points(i).reason = 'estimated from report natural frequency because continuous forward branch was not available';
    end
end
write_report_critical_speed_summary(crit,outfile);
end


function match = identify_local_critical_speed_near_mode(report_mode,camp,opts)
match = [];
rpm_guess = 60*report_mode.frequency_Hz;
rpm1 = 0.5*rpm_guess;
rpm2 = 1.5*rpm_guess;
if rpm_guess < 3000
    rpm1 = 1000;
    rpm2 = 1.8*rpm_guess;
end
rpm1 = max(min(camp.rpm),rpm1);
rpm2 = min(max(camp.rpm),rpm2);
best = [];
for io = 1:size(camp.freq,1)
    f = squeeze(camp.freq(io,1,:)).';
    inwin = camp.rpm >= rpm1 & camp.rpm <= rpm2 & isfinite(f);
    if sum(inwin) < 3, continue, end
    d = f - camp.rpm/60;
    idx = find(inwin);
    for kk = 1:numel(idx)-1
        k = idx(kk);
        if ~inwin(k+1), continue, end
        if d(k) == 0 || d(k)*d(k+1) <= 0
            rpm = interp1(d(k:k+1),camp.rpm(k:k+1),0,'linear','extrap');
            if isfinite(rpm) && rpm >= opts.min_critical_rpm
                match = struct('branch_order',io,'rpm',rpm,'freq',rpm/60, ...
                    'method','local_forward_branch_intersection','reliable',true, ...
                    'mac',NaN,'reason','local forward branch crossed 1X near report natural frequency');
                return
            end
        end
    end
    [err,imin] = min(abs(d(inwin)));
    local_rpm = camp.rpm(idx(imin));
    if isempty(best) || err < best.err
        best = struct('branch_order',io,'rpm',local_rpm,'freq',f(idx(imin)), ...
            'method','local_minimum_error','reliable',false,'mac',NaN, ...
            'reason','nearest local forward branch point to 1X without sign crossing', ...
            'err',err);
    end
end
if ~isempty(best) && best.err <= max(5,0.10*report_mode.frequency_Hz)
    match = rmfield(best,'err');
end
end


function candidates = global_forward_1x_intersections(camp,opts)
candidates = repmat(struct('branch_order',NaN,'rpm',NaN,'freq',NaN),0,1);
exc = camp.rpm/60;
for io = 1:size(camp.freq,1)
    f = squeeze(camp.freq(io,1,:)).';
    valid = isfinite(f);
    if sum(valid) < 3, continue, end
    d = f - exc;
    for k = 1:numel(camp.rpm)-1
        if ~valid(k) || ~valid(k+1), continue, end
        if d(k) == 0 || d(k)*d(k+1) <= 0
            rpm = interp1(d(k:k+1),camp.rpm(k:k+1),0,'linear','extrap');
            if isfinite(rpm) && rpm >= opts.min_critical_rpm
                candidates(end+1).branch_order = io; %#ok<AGROW>
                candidates(end).rpm = rpm;
                candidates(end).freq = rpm/60;
                break
            end
        end
    end
end
end


function write_report_critical_speed_summary(crit,outfile)
outdir = fileparts(outfile);
if ~exist(outdir,'dir'), mkdir(outdir); end
fid = fopen(outfile,'w');
assert(fid > 0,'Cannot write report critical speed summary.');
fprintf(fid,'critical_order,natural_frequency_Hz,rpm_initial_guess,critical_rpm,critical_frequency_Hz,identification_method,mode_index,mode_type,branch_direction,MAC_to_natural_mode,rotor_energy_ratio,casing_energy_ratio,reliable,reason\n');
for i = 1:numel(crit.points)
    rpm_guess = 60*crit.points(i).natural_frequency_Hz;
    fprintf(fid,'%.0f,%.10g,%.10g,%.10g,%.10g,%s,%.0f,%s,%s,%.10g,%.10g,%.10g,%s,%s\n', ...
        crit.points(i).critical_order,crit.points(i).natural_frequency_Hz,rpm_guess, ...
        crit.points(i).rpm,crit.points(i).freq,crit.points(i).identification_method, ...
        crit.points(i).mode_index,crit.points(i).mode_type,crit.points(i).branch_direction, ...
        crit.points(i).MAC_to_natural_mode,crit.points(i).rotor_energy_ratio, ...
        crit.points(i).casing_energy_ratio,reliable_label(crit.points(i).reliable),crit.points(i).reason);
end
fclose(fid);
fprintf('Report critical speed summary written to: %s\n',outfile);
end


function s = reliable_label(flag)
if islogical(flag)
    if flag, s = 'true'; else, s = 'false'; end
elseif isnumeric(flag)
    if flag == 1, s = 'true'; elseif flag == 0, s = 'false'; else, s = 'approximate'; end
else
    s = char(flag);
end
end


function report_modes = extract_undamped_report_natural_frequencies(modal,opts,outfile)
M = (modal.M + modal.M.')/2;
K = (modal.K + modal.K.')/2;
[V,D] = eig(K,M);
w2 = real(diag(D));
valid = isfinite(w2) & w2 > (2*pi*opts.min_freq_hz)^2;
w = sqrt(w2(valid));
idx_all = find(valid);
[freq,isort] = sort(w/(2*pi));
idx_all = idx_all(isort);
V = V(:,valid);
V = V(:,isort);
nmode = numel(freq);
diag_rows = repmat(struct('mode_index',0,'frequency_Hz',0,'frequency_source','undamped_generalized_eigenvalue', ...
    'mode_type','','rotor_energy_ratio',0,'casing_energy_ratio',0,'bearing_relative_motion_ratio',0, ...
    'shape_complexity',0,'whirl_direction','undamped','used_for_report',0,'reason','', ...
    'phi_rotor',[]),nmode,1);
for i = 1:nmode
    q = V(:,i);
    qrot = q(1:modal.ndof_rotor);
    qcase = q(modal.ndof_rotor+1:end);
    qxy = qrot(rotor_xy_indices(modal.node_count_rotor));
    qcxy = qcase(rotor_xy_indices(modal.N_C+1));
    re = norm(qxy)^2;
    ce = norm(qcxy)^2;
    rr = re/max(re+ce,eps);
    cr = ce/max(re+ce,eps);
    br = bearing_relative_motion_ratio(qrot,qcase,modal);
    sc = rotor_shape_complexity(qxy,modal.node_count_rotor,modal.z);
    mt = classify_mode(freq(i),0.01,rr,cr,sc,opts);
    if ~strcmp(mt,'rotor_bending') && ~strcmp(mt,'rotor_support_coupled') && ...
            ~strcmp(mt,'numerical_or_unstable') && ...
            ~strcmp(mt,'casing_local') && rr >= 0.30 && br >= 0.10 && cr < 0.70 && ...
            sc >= 0.01 && sc <= opts.shape_complexity_max
        mt = 'coupled_rotor_bending';
    end
    diag_rows(i).mode_index = idx_all(i);
    diag_rows(i).frequency_Hz = freq(i);
    diag_rows(i).mode_type = mt;
    diag_rows(i).rotor_energy_ratio = rr;
    diag_rows(i).casing_energy_ratio = cr;
    diag_rows(i).bearing_relative_motion_ratio = br;
    diag_rows(i).shape_complexity = sc;
    diag_rows(i).phi_rotor = qrot;
end
report_modes = select_report_modes_from_rows(diag_rows);
write_report_modes_csv(report_modes,outfile);
fprintf('Undamped natural frequency summary written to: %s\n',outfile);
end


function report_modes = select_report_modes_from_rows(rows)
selected = [];
priority = {'rotor_bending','coupled_rotor_bending','rotor_support_coupled','rigid_support_coupled'};
reasons = {'undamped rotor_bending mode selected for report', ...
    'undamped coupled_rotor_bending mode selected for report', ...
    'undamped rotor-support coupled mode; reference for coupled-system low-order characteristic, not pure rotor bending', ...
    'undamped rigid/support-coupled mode; reference for coupled-system low-order characteristic, not pure rotor bending'};
used = [];
for ip = 1:numel(priority)
    cand = rows(strcmp({rows.mode_type},priority{ip}));
    [~,isort] = sort([cand.frequency_Hz]);
    cand = cand(isort);
    for i = 1:numel(cand)
        if numel(selected) >= 3, break, end
        if any(used == cand(i).mode_index), continue, end
        cand(i).reason = reasons{ip};
        cand(i).used_for_report = 1;
        selected = [selected cand(i)]; %#ok<AGROW>
        used = [used cand(i).mode_index]; %#ok<AGROW>
    end
end
if ~isempty(selected)
    [~,isort] = sort([selected.frequency_Hz]);
    report_modes = selected(isort);
else
    report_modes = selected;
end
end


function export_report_modes_to_natural_summary(report_modes,outfile)
write_report_modes_csv(report_modes,outfile);
fprintf('Natural frequency summary overwritten with undamped report frequencies: %s\n',outfile);
end


function write_report_modes_csv(report_modes,outfile)
outdir = fileparts(outfile);
if ~exist(outdir,'dir'), mkdir(outdir); end
fid = fopen(outfile,'w');
assert(fid > 0,'Cannot write report natural frequency summary.');
fprintf(fid,'report_mode_order,mode_index,frequency_Hz,frequency_source,mode_type,rotor_energy_ratio,casing_energy_ratio,bearing_relative_motion_ratio,shape_complexity,whirl_direction,used_for_report,reason\n');
for i = 1:numel(report_modes)
    fprintf(fid,'%d,%.0f,%.10g,%s,%s,%.10g,%.10g,%.10g,%.10g,%s,%d,%s\n', ...
        i,report_modes(i).mode_index,report_modes(i).frequency_Hz,report_modes(i).frequency_source, ...
        report_modes(i).mode_type,report_modes(i).rotor_energy_ratio,report_modes(i).casing_energy_ratio, ...
        report_modes(i).bearing_relative_motion_ratio,report_modes(i).shape_complexity, ...
        report_modes(i).whirl_direction,report_modes(i).used_for_report,report_modes(i).reason);
end
fclose(fid);
end


function save_report_mode_shapes(model,report_modes,current_model_dir)
outdir = fullfile(current_model_dir,'results','modal');
if ~exist(outdir,'dir'), mkdir(outdir); end
valid = [];
for i = 1:numel(report_modes)
    if isfield(report_modes,'phi_rotor') && ~isempty(report_modes(i).phi_rotor) && ...
            isfinite(report_modes(i).frequency_Hz) && ...
            ~strcmp(report_modes(i).mode_type,'numerical_or_unstable') && ...
            ~strcmp(report_modes(i).mode_type,'casing_local')
        valid(end+1) = i; %#ok<AGROW>
    end
end
valid = valid(1:min(3,numel(valid)));
if numel(valid) < 3
    warning('Only %d report mode shapes are available from undamped report modes.',numel(valid));
end
for i = 1:numel(valid)
    fig = figure(40+i);
    clf(fig)
    set(fig,'Position',[100 100 900 520])
    plot_report_mode_shape(model,report_modes(valid(i)),i)
    saveas(fig,fullfile(outdir,sprintf('mode_shape_%d.png',i)))
end
fig = figure(44);
clf(fig)
set(fig,'Position',[80 80 1200 820])
for i = 1:numel(valid)
    subplot(2,2,i)
    plot_report_mode_shape(model,report_modes(valid(i)),i)
end
saveas(fig,fullfile(outdir,'mode_shapes_first_three.png'))
end


function plot_report_mode_shape(model,report_mode,display_index)
mode_label = report_mode_title_label(report_mode.mode_type);
phi = report_mode.phi_rotor(:);
nnode = size(model.node,1);
z = model.node(:,end);
qx = phi(1:4:4*nnode);
qy = phi(2:4:4*nnode);
comp = [real(qx(:)) imag(qx(:)) real(qy(:)) imag(qy(:))];
spread = max(comp,[],1) - min(comp,[],1);
[~,i1] = max(spread);
def1 = comp(:,i1);
spread(i1) = -inf;
[~,i2] = max(spread);
def2 = comp(:,i2);
if max(abs(def1)) > eps
    def1 = def1/max(abs(def1));
else
    def1 = zeros(size(z));
end
if max(abs(def2)) > eps
    def2 = def2/max(abs(def2));
else
    def2 = zeros(size(z));
end
plot(z,zeros(size(z)),'k:','LineWidth',0.8,'HandleVisibility','off')
hold on
plot(z,def1,'k-','LineWidth',1.8,'DisplayName','dominant component')
plot(z,def2,'Color',[0.35 0.35 0.35],'LineStyle','--','LineWidth',1.2,'DisplayName','secondary component')
plot(z,def1,'ko','MarkerSize',3,'MarkerFaceColor','w','HandleVisibility','off')
if isfield(model,'bearing') && ~isempty(model.bearing)
    bn = model.bearing(:,2);
    plot(z(bn),zeros(size(bn)),'rs','MarkerFaceColor','r','MarkerSize',5,'DisplayName','bearing')
end
if isfield(model,'disc') && ~isempty(model.disc)
    dn = model.disc(:,2);
    plot(z(dn),zeros(size(dn)),'bo','MarkerFaceColor','b','MarkerSize',5,'DisplayName','disk')
end
hold off
grid on
ylim(1.12*[-1 1])
xlim([min(z) max(z)])
xlabel('Axial position')
ylabel('Normalized lateral deflection')
title({sprintf('Mode %d: %s',display_index,mode_label), ...
    sprintf('f = %.3f Hz, rotor ratio = %.3f, mode\\_type = %s', ...
    report_mode.frequency_Hz,report_mode.rotor_energy_ratio,report_mode.mode_type)}, ...
    'FontSize',8,'Interpreter','none')
set(gca,'FontSize',8)
end


function label = report_mode_title_label(mode_type)
switch mode_type
    case 'rotor_bending'
        label = 'Rotor bending';
    case 'coupled_rotor_bending'
        label = 'Coupled rotor bending';
    case 'rotor_support_coupled'
        label = 'Rotor-support coupled mode';
    case 'rigid_support_coupled'
        label = 'Rigid support coupled mode';
    otherwise
        label = strrep(mode_type,'_',' ');
end
end


function plotcamp_forward_backward_separated(Rotor_Spd_rpm,camp,opts,crit,run_speed_rpm)
clf
hold on
styles = {'-','-.','--'};
for io = 1:size(camp.freq,1)
    f = squeeze(camp.freq(io,1,:)).';
    if any(isfinite(f))
        plot(Rotor_Spd_rpm,f,'r','LineStyle',styles{min(io,numel(styles))},'LineWidth',2.0, ...
            'DisplayName',ordinal_label(io,'forward'));
    end
    f = squeeze(camp.freq(io,2,:)).';
    if any(isfinite(f))
        plot(Rotor_Spd_rpm,f,'g','LineStyle',styles{min(io,numel(styles))},'LineWidth',2.0, ...
            'DisplayName',ordinal_label(io,'backward'));
    end
end
plot(Rotor_Spd_rpm,Rotor_Spd_rpm/60,'b--','LineWidth',1.0,'DisplayName','1X');
plot(Rotor_Spd_rpm,2*Rotor_Spd_rpm/60,'b--','LineWidth',1.0,'DisplayName','2X');
if nargin >= 4 && isfield(crit,'points')
    for i = 1:numel(crit.points)
        if isfinite(crit.points(i).rpm)
            if istrue_reliable(crit.points(i).reliable)
                plot(crit.points(i).rpm,crit.points(i).freq,'ko','MarkerFaceColor','y','MarkerSize',6,'HandleVisibility','off')
                label_suffix = '1X';
            elseif strcmp(crit.points(i).identification_method,'local_forward_branch_intersection')
                plot(crit.points(i).rpm,crit.points(i).freq,'o','MarkerEdgeColor',[0.85 0.35 0.0], ...
                    'MarkerFaceColor',[1.0 0.65 0.1],'MarkerSize',6,'HandleVisibility','off')
                label_suffix = 'approx';
            else
                plot(crit.points(i).rpm,crit.points(i).freq,'o','MarkerEdgeColor',[0.35 0.35 0.35], ...
                    'MarkerFaceColor','w','MarkerSize',6,'HandleVisibility','off')
                plot([crit.points(i).rpm crit.points(i).rpm],ylim,'Color',[0.55 0.55 0.55], ...
                    'LineStyle',':','LineWidth',0.8,'HandleVisibility','off')
                label_suffix = 'est';
            end
            if strcmp(crit.points(i).identification_method,'modal_frequency_estimate')
                label_text = sprintf(' Nc%d %s',i,label_suffix);
            else
                label_text = sprintf(' C%d %s',i,label_suffix);
            end
            text(crit.points(i).rpm,crit.points(i).freq + 45*i,label_text, ...
                'HorizontalAlignment','left','VerticalAlignment','bottom')
        end
    end
end


function tf = istrue_reliable(flag)
tf = (islogical(flag) && flag) || (isnumeric(flag) && flag == 1);
end
xlabel('Rotor spin speed / r·min^{-1}')
ylabel('Damped natural frequency / Hz')
title('Campbell chart and critical speed reference of the rotor-bearing-casing system','FontSize',11)
ylim([0 min(max(opts.max_freq_hz,2200),2500)])
xmask = false(size(Rotor_Spd_rpm));
for ispd = 1:numel(Rotor_Spd_rpm)
    xmask(ispd) = any(any(isfinite(camp.freq(:,:,ispd))));
end
crit_rpm = [];
if nargin >= 4 && isfield(crit,'points')
    crit_rpm = [crit.points.rpm];
    crit_rpm = crit_rpm(isfinite(crit_rpm));
end
xmax_candidates = [run_speed_rpm*1.25 20000];
if any(xmask)
    xmax_candidates(end+1) = max(Rotor_Spd_rpm(xmask))*1.15;
end
if ~isempty(crit_rpm)
    xmax_candidates(end+1) = max(crit_rpm)*1.25;
end
xmax_plot = min(max(Rotor_Spd_rpm),max(xmax_candidates));
xlim([0 xmax_plot])
legend('Location','northwest')
grid on
hold off
end


function txt = ordinal_label(io,dir)
prefix = {'1st','2nd','3rd','4th','5th','6th'};
txt = sprintf('%s %s whirl',prefix{min(io,numel(prefix))},dir);
end


function validation = validate_campbell_result(camp,crit,opts)
norder = size(camp.freq,1);
validation.n_forward_branches = 0;
validation.n_backward_branches = 0;
validation.branch_coverage = NaN(norder,2);
validation.max_frequency_jump_each_branch = NaN(norder,2);
validation.direction_switching_detected = false;
validation.branch_cross_connection_detected = false;
validation.zero_speed_vertical_stack_detected = false;
active_speed = false(1,numel(camp.rpm));
for ispd = 1:numel(camp.rpm)
    active_speed(ispd) = any(any(isfinite(camp.freq(:,:,ispd))));
end
if ~any(active_speed)
    active_speed(:) = true;
end
active_count = sum(active_speed);
for idir = 1:2
    for io = 1:norder
        f = squeeze(camp.freq(io,idir,:)).';
        valid = isfinite(f);
        coverage = sum(valid & active_speed)/max(active_count,1);
        validation.branch_coverage(io,idir) = coverage;
        if coverage >= 0.50
            if idir == 1
                validation.n_forward_branches = validation.n_forward_branches + 1;
            else
                validation.n_backward_branches = validation.n_backward_branches + 1;
            end
        end
        df = abs(diff(f(valid)));
        if isempty(df)
            validation.max_frequency_jump_each_branch(io,idir) = NaN;
        else
            validation.max_frequency_jump_each_branch(io,idir) = max(df);
        end
        if any(df > opts.max_freq_jump_hz*1.5)
            validation.branch_cross_connection_detected = true;
        end
    end
end
f0 = squeeze(camp.freq(:,:,1));
validation.zero_speed_vertical_stack_detected = any(sum(isfinite(f0),1) > 3);
validation.critical_speed_detected = any([crit.points.excitation_order] == 1 & isfinite([crit.points.rpm]));
validation.reliable_critical_speed_detected = any([crit.points.excitation_order] == 1 & [crit.points.reliable]);
validation.strict_ready = validation.n_forward_branches >= 3 && validation.n_backward_branches >= 3 && ...
    all(validation.branch_coverage(:) >= opts.min_branch_coverage) && ...
    ~validation.direction_switching_detected && ~validation.branch_cross_connection_detected && ...
    ~validation.zero_speed_vertical_stack_detected && validation.reliable_critical_speed_detected;
validation.report_usable = validation.n_forward_branches >= 1 && validation.critical_speed_detected && ...
    ~validation.direction_switching_detected && ~validation.branch_cross_connection_detected && ...
    ~validation.zero_speed_vertical_stack_detected;
validation.report_ready = validation.report_usable;

fprintf('\n===== Campbell 图自动验证 =====\n');
fprintf('forward 分支数量 = %d\n',validation.n_forward_branches);
fprintf('backward 分支数量 = %d\n',validation.n_backward_branches);
fprintf('最大相邻频率跳变 = %.3f Hz\n',max(validation.max_frequency_jump_each_branch(:),[],'omitnan'));
fprintf('direction switching detected = %d\n',validation.direction_switching_detected);
fprintf('branch cross connection detected = %d\n',validation.branch_cross_connection_detected);
fprintf('zero speed vertical stack detected = %d\n',validation.zero_speed_vertical_stack_detected);
fprintf('critical speed detected = %d\n',validation.critical_speed_detected);
fprintf('strict_ready = %d\n',validation.strict_ready);
fprintf('report_usable = %d\n',validation.report_usable);
if validation.report_usable
    fprintf('Campbell 图可作为报告参考图；strict_ready=%d。\n',validation.strict_ready);
else
    fprintf('当前 Campbell 图仍不建议作为报告参考图；详见 modal_critical_speed_report.md。\n');
end
fprintf('================================\n\n');
end


function export_campbell_debug_report(validation,camp,opts,current_model_dir)
logfile = fullfile(current_model_dir,'results','modal','campbell_debug_log.txt');
fid = fopen(logfile,'a');
if fid > 0
    fprintf(fid,'iteration_id=%s\n',datestr(now,'yyyymmdd_HHMMSS'));
    fprintf(fid,'modified_functions=campbell_candidates_fb,track_campbell_forward_backward,plotcamp_forward_backward_separated,validate_campbell_result\n');
    fprintf(fid,'run_success=1\n');
    fprintf(fid,'forward_branches=%d\n',validation.n_forward_branches);
    fprintf(fid,'backward_branches=%d\n',validation.n_backward_branches);
    fprintf(fid,'branch_cross_connection_detected=%d\n',validation.branch_cross_connection_detected);
    fprintf(fid,'zero_speed_vertical_stack_detected=%d\n',validation.zero_speed_vertical_stack_detected);
    fprintf(fid,'critical_speed_detected=%d\n',validation.critical_speed_detected);
    fprintf(fid,'strict_ready=%d\n',validation.strict_ready);
    fprintf(fid,'report_usable=%d\n\n',validation.report_usable);
    fclose(fid);
end
if ~validation.report_usable
    mdfile = fullfile(current_model_dir,'results','modal','campbell_failure_diagnosis.md');
    fid = fopen(mdfile,'w');
    if fid > 0
        fprintf(fid,'# Campbell failure diagnosis\n\n');
        fprintf(fid,'The corrected Campbell workflow uses real eigenvalues only. It did not satisfy the report-ready criteria.\n\n');
        fprintf(fid,'## Reliable branches\n\n');
        fprintf(fid,'Forward branches with coverage >= %.2f: %d\n\n',opts.min_branch_coverage,validation.n_forward_branches);
        fprintf(fid,'Backward branches with coverage >= %.2f: %d\n\n',opts.min_branch_coverage,validation.n_backward_branches);
        fprintf(fid,'## Likely causes\n\n');
        fprintf(fid,'- The current coupled rotor-bearing-casing eigenvalue set does not provide three forward and three backward rotor-bending branches that remain continuous over 80%% of the speed range.\n');
        fprintf(fid,'- Interrupted branches are present and are not forced into the report plot to avoid false connections.\n');
        fprintf(fid,'- Check bearing-seat node mapping, casing boundary conditions, disk inertia inclusion, and mass regularization if three clean mode pairs are required physically.\n');
        fclose(fid);
    end
end
end


function [camp_eigs,camp_kappa,camp_info] = campbell_fb_to_legacy(camp,modal)
norder = size(camp.freq,1);
nspeed = numel(camp.rpm);
nbranch = 2*norder;
camp_eigs = NaN(nbranch,nspeed);
camp_kappa = zeros(modal.ndof_rotor,nbranch,nspeed);
camp_info.selected_idx = NaN(nbranch,nspeed);
camp_info.rotor_ratio = NaN(nbranch,nspeed);
camp_info.damping_ratio = NaN(nbranch,nspeed);
camp_info.rejected_low_freq = zeros(1,nspeed);
camp_info.rejected_casing = zeros(1,nspeed);
camp_info.rejected_numeric = zeros(1,nspeed);
camp_info.ntrack = nbranch;
for io = 1:norder
    for idir = 1:2
        ib = (io-1)*2 + idir;
        camp_eigs(ib,:) = squeeze(camp.eig(io,idir,:)).';
        camp_info.selected_idx(ib,:) = squeeze(camp.mode_index(io,idir,:)).';
        camp_info.rotor_ratio(ib,:) = squeeze(camp.rotor_ratio(io,idir,:)).';
        camp_info.damping_ratio(ib,:) = squeeze(camp.damping_ratio(io,idir,:)).';
    end
end
end


function export_modal_comparison_summaries(modal,opts,current_model_dir)
outdir = fullfile(current_model_dir,'results','modal');
K_ground = modal.Kr;
C_ground = modal.Cr;
K_rigid = modal.Kr;
C_rigid = modal.Cr;
for ib = 1:numel(modal.bearing.rotor_nodes)
    node = modal.bearing.rotor_nodes(ib);
    ix = 4*node - 3;
    iy = 4*node - 2;
    K_ground(ix,ix) = K_ground(ix,ix) + modal.bearing.kx(ib);
    K_ground(iy,iy) = K_ground(iy,iy) + modal.bearing.ky(ib);
    C_ground(ix,ix) = C_ground(ix,ix) + modal.bearing.cx(ib);
    C_ground(iy,iy) = C_ground(iy,iy) + modal.bearing.cy(ib);
    K_rigid(ix,ix) = K_rigid(ix,ix) + 2.5e9;
    K_rigid(iy,iy) = K_rigid(iy,iy) + 2.5e9;
    C_rigid(ix,ix) = C_rigid(ix,ix) + 5e3;
    C_rigid(iy,iy) = C_rigid(iy,iy) + 5e3;
end
write_model_summary(fullfile(outdir,'rotor_only_modal_summary.csv'), ...
    modal_first_frequencies(modal.Mr,K_ground,C_ground,modal.Gr,0,opts.min_freq_hz,6),'rotor_only_ground_bearing');
write_model_summary(fullfile(outdir,'rigid_casing_modal_summary.csv'), ...
    modal_first_frequencies(modal.Mr,K_rigid,C_rigid,modal.Gr,0,opts.min_freq_hz,6),'rigid_casing_bearing_seat');
write_model_summary(fullfile(outdir,'coupled_modal_summary.csv'), ...
    modal_first_frequencies(modal.M,modal.K,modal.C,modal.G,0,opts.min_freq_hz,6),'flexible_casing_coupled');
end


function write_model_summary(outfile,freqs,model_type)
fid = fopen(outfile,'w');
assert(fid > 0,'Cannot write model summary.');
fprintf(fid,'model_type,mode_order,frequency_Hz\n');
for i = 1:numel(freqs)
    fprintf(fid,'%s,%d,%.10g\n',model_type,i,freqs(i));
end
fclose(fid);
end


function export_modal_model_audit(modal,current_model_dir)
outdir = fullfile(current_model_dir,'results','modal');
txtfile = fullfile(outdir,'modal_model_audit.txt');
csvfile = fullfile(outdir,'modal_model_audit.csv');
fid = fopen(txtfile,'w');
assert(fid > 0,'Cannot write modal model audit.');
fprintf(fid,'Modal model audit\n');
fprintf(fid,'Model: coupled rotor - dual bearing - flexible casing\n');
fprintf(fid,'Rotor bearing nodes: %d, %d\n',modal.loc_rub(1),modal.loc_rub(2));
fprintf(fid,'Casing bearing-seat nodes: %d, %d\n',modal.case_bearing_nodes(1),modal.case_bearing_nodes(2));
fprintf(fid,'Disk masses used: %.4f, %.4f, %.4f, %.4f kg\n',modal.disk_masses);
fprintf(fid,'Disk mass source: high-pressure rotor equivalent disk masses supplied by user.\n');
fprintf(fid,'include_disk_inertia = %d\n',modal.include_disk_inertia);
fprintf(fid,'Disk polar gyro matrix contribution: included when include_disk_inertia = 1.\n');
fprintf(fid,'Bearing 1 kx/ky/cx/cy = %.6e %.6e %.6e %.6e\n',modal.bearing.kx(1),modal.bearing.ky(1),modal.bearing.cx(1),modal.bearing.cy(1));
fprintf(fid,'Bearing 2 kx/ky/cx/cy = %.6e %.6e %.6e %.6e\n',modal.bearing.kx(2),modal.bearing.ky(2),modal.bearing.cx(2),modal.bearing.cy(2));
old_mass = [126.3 159.4 149.7 145.9];
if norm(modal.disk_masses - old_mass) < 1e-9
    fprintf(fid,'WARNING: old incorrect disk masses are still active.\n');
else
    fprintf(fid,'Old incorrect disk masses [126.3 159.4 149.7 145.9] are not active.\n');
end
for i = 1:numel(modal.disk_info)
    fprintf(fid,'Disk node %d: mass=%.6f kg, Jd=%.6e kg*m^2, Jp=%.6e kg*m^2\n', ...
        modal.disk_info(i).node,modal.disk_info(i).mass,modal.disk_info(i).Jd,modal.disk_info(i).Jp);
end
fclose(fid);

fid = fopen(csvfile,'w');
assert(fid > 0,'Cannot write modal model audit CSV.');
fprintf(fid,'item,value,unit,source\n');
for i = 1:numel(modal.disk_info)
    fprintf(fid,'disk_%d_mass,%.10g,kg,user_supplied_hp_rotor_equivalent_mass\n',i,modal.disk_info(i).mass);
    fprintf(fid,'disk_%d_Jd,%.10g,kg*m^2,estimated_from_disk_mass_and_geometry\n',i,modal.disk_info(i).Jd);
    fprintf(fid,'disk_%d_Jp,%.10g,kg*m^2,estimated_from_disk_mass_and_geometry\n',i,modal.disk_info(i).Jp);
end
fprintf(fid,'include_disk_inertia,%d,boolean,default_modal_config\n',modal.include_disk_inertia);
fprintf(fid,'gyro_disk_polar_contribution,%d,boolean,add_lumped_disk_inertia_and_gyro\n',modal.include_disk_inertia);
fclose(fid);
fprintf('Modal model audit written to: %s\n',txtfile);
end


function export_modal_critical_speed_report(modal,diag0,diag9900,camp,crit,validation,current_model_dir)
outdir = fullfile(current_model_dir,'results','modal');
outfile = fullfile(outdir,'modal_critical_speed_report.md');
rotor = diag9900(strcmp({diag9900.mode_type},'rotor_bending'));
[~,isort] = sort([rotor.frequency_Hz]);
rotor = rotor(isort);
fid = fopen(outfile,'w');
assert(fid > 0,'Cannot write modal critical speed report.');
fprintf(fid,'# Modal and critical-speed report\n\n');
fprintf(fid,'## Model audit\n\n');
fprintf(fid,'Disk masses used: %.4f, %.4f, %.4f, %.4f kg.\n\n',modal.disk_masses);
fprintf(fid,'Disk inertia enabled: `%d`.\n\n',modal.include_disk_inertia);
fprintf(fid,'Disk polar gyroscopic contribution: `%d` through `add_lumped_disk_inertia_and_gyro`.\n\n',modal.include_disk_inertia);
fprintf(fid,'Critical-speed intersections below 1000 rpm are treated as rigid/base-mode dominated and are filtered.\n\n');
fprintf(fid,'## Report natural frequencies\n\n');
fprintf(fid,'Report natural frequencies prioritize the undamped generalized eigenvalue problem `K phi = omega^2 M phi`. Damped state-space frequencies are retained as Campbell references.\n\n');
for i = 1:min(3,numel(rotor))
    fprintf(fid,'%d. mode_index %.0f, f = %.3f Hz, rotor_energy_ratio = %.3f, direction = %s.\n', ...
        i,rotor(i).mode_index,rotor(i).frequency_Hz,rotor(i).rotor_energy_ratio,rotor(i).whirl_direction);
end
fprintf(fid,'\n## Campbell tracking\n\n');
fprintf(fid,'Reliable forward branches: %d. Reliable backward branches: %d.\n\n', ...
    validation.n_forward_branches,validation.n_backward_branches);
fprintf(fid,'The main Campbell critical-speed search uses only forward rotor-bending branches and 1X excitation intersections.\n\n');
fprintf(fid,'strict_ready = `%d`; report_usable = `%d`.\n\n',validation.strict_ready,validation.report_usable);
if ~validation.strict_ready && validation.report_usable
    fprintf(fid,'由于柔性机匣耦合使部分低阶模态受到支承和机匣运动影响，当前 Campbell 图未形成完整三对正反涡动分支，因此本文将 Campbell 图作为临界转速参考分析图，而不是作为完整三阶正反涡动分支验证图。\n\n');
end
fprintf(fid,'## Critical speeds\n\n');
for i = 1:numel(crit.points)
    if isfinite(crit.points(i).rpm)
        fprintf(fid,'- Entry %d: %.1f rpm, %.3f Hz, method=%s, reliable=%d, mode_type=%s.\n', ...
            i,crit.points(i).rpm,crit.points(i).freq,crit.points(i).identification_method,crit.points(i).reliable,crit.points(i).mode_type);
    else
        fprintf(fid,'- Entry %d: not reliably identified.\n',i);
    end
end
primary = find([crit.points.excitation_order] == 1 & isfinite([crit.points.rpm]),1,'first');
if isempty(primary)
    fprintf(fid,'\nCurrent 9900 rpm relation to first 1X critical speed: cannot judge.\n');
else
    fprintf(fid,'\nCurrent 9900 rpm is below first 1X critical speed: %s.\n',yes_no(9900 < crit.points(primary).rpm));
end
fprintf(fid,'\n## If fewer than three critical speeds are identified\n\n');
if sum([crit.points.excitation_order] == 1 & [crit.points.reliable]) < 3
    fprintf(fid,'The coupled model did not provide three reliable forward 1X crossings within the tracked branches. ');
    fprintf(fid,'Rejected candidates were mainly support, casing-local, numerical/high-damping, or direction-uncertain modes. ');
    fprintf(fid,'Minimum next checks: verify disk inertia/gyro, inspect bearing-seat mapping, compare rotor-only and rigid-casing summaries, and review whether flexible casing coupling reduces the number of rotor-dominant branches.\n');
else
    fprintf(fid,'Three reliable 1X forward critical speeds were identified.\n');
end
fprintf(fid,'\n## Report readiness\n\n');
if validation.report_usable
    fprintf(fid,'Campbell figure can be used as a report reference figure with the stated branch limitations.\n');
else
    fprintf(fid,'Campbell figure should be interpreted together with the modal-frequency estimates and branch diagnostics.\n');
end
fprintf(fid,'\n## Text for report\n\n');
fprintf(fid,'本文在修正圆盘等效质量和圆盘转动惯量后，对高压转子—轴承—机匣耦合系统进行了固有特性分析。由于柔性机匣和轴承支承参与低阶模态，部分模态并不表现为理想单转子系统中的完整正反涡动成对分支。因此，本文不强行绘制三对正反涡动 Campbell 分支，而是结合转子主导模态筛选、局部 Campbell 追踪和固有频率估算给出前三阶临界转速参考结果。该处理避免了将支承模态、机匣局部模态或数值异常模态误作为转子弯曲模态，同时保证临界转速结果来源于真实特征值计算，具有工程参考意义。\n');
fclose(fid);
fprintf('Modal critical speed report written to: %s\n',outfile);
end


function print_final_modal_critical_summary(mode_diag_9900,crit,validation)
rotor = mode_diag_9900(strcmp({mode_diag_9900.mode_type},'rotor_bending'));
[~,isort] = sort([rotor.frequency_Hz]);
rotor = rotor(isort);
fprintf('\n===== 报告用固有特性结果 =====\n\n');
fprintf('前三阶无阻尼固有频率：\n');
for i = 1:numel(crit.points)
    fprintf('第%d阶：%.3f Hz，mode_type = %s，rotor_ratio = %.3f\n', ...
        i,crit.points(i).natural_frequency_Hz,crit.points(i).mode_type,crit.points(i).rotor_energy_ratio);
end
fprintf('\n前三阶阻尼参考频率：\n');
for i = 1:min(3,numel(rotor))
    fprintf('第%d阶：%.3f Hz\n',i,rotor(i).frequency_Hz);
end
fprintf('\n前三阶临界转速参考结果：\n');
primary = crit.points([crit.points.excitation_order] == 1 & isfinite([crit.points.rpm]));
if isempty(primary)
    fprintf('未输出\n');
else
    for i = 1:min(3,numel(primary))
        fprintf('第%d阶：%.1f rpm，method = %s，reliable = %s\n', ...
            i,primary(i).rpm,primary(i).identification_method,reliable_label(primary(i).reliable));
    end
end
fprintf('\n当前工作转速 9900 rpm：\n');
if ~isempty(primary)
    fprintf('当前工作转速 9900 rpm 是否低于一阶1X临界转速：%s\n',yes_no(9900 < primary(1).rpm));
    near = abs(9900-primary(1).rpm)/max(primary(1).rpm,eps) < 0.10;
    fprintf('是否接近临界转速区间：%s\n',yes_no(near));
    fprintf('判断依据：与一阶临界转速参考值 %.1f rpm 的相对差异为 %.1f%%\n', ...
        primary(1).rpm,100*abs(9900-primary(1).rpm)/max(primary(1).rpm,eps));
else
    fprintf('当前工作转速 9900 rpm 是否低于一阶1X临界转速：无法判断\n');
    fprintf('是否接近临界转速区间：无法判断\n');
    fprintf('判断依据：没有临界转速参考值\n');
end
fprintf('\nCampbell 图说明：\n');
fprintf('strict_ready = %d, report_usable = %d\n',validation.strict_ready,validation.report_usable);
fprintf('是否获得完整三对正反涡动：%s\n',yes_no(validation.strict_ready));
fprintf('是否可作为报告参考图：%s\n',yes_no(validation.report_usable));
fprintf('是否可作为唯一临界转速依据：否\n');
if sum([primary.reliable]) < 3
    fprintf('说明：前三阶中部分临界转速为固有频率估算值，原因是当前耦合模型可靠 forward rotor_bending 分支不足；详见 modal_critical_speed_report.md。\n');
    fprintf('下一步最小修改建议：检查圆盘惯量估算、轴承座映射、柔性机匣边界，并用 rotor_only / rigid_casing 对照结果定位来源。\n');
end
fprintf('================================\n\n');
end


function plotcamp_tracked(Rotor_Spd,eigenvalues,NX,damped_NF,kappa,opts,crit,run_speed_rpm)
if nargin < 4
    damped_NF = 1;
end
if damped_NF >= 0.5
    nat_freqs_Hz = abs(imag(eigenvalues))/(2*pi);
    ylab = 'Damped natural frequencies (Hz)';
else
    nat_freqs_Hz = abs(eigenvalues)/(2*pi);
    ylab = 'Undamped natural frequencies (Hz)';
end
Rotor_Spd_rpm = Rotor_Spd*60/(2*pi);
freq_lines = ((1:abs(NX)).')*Rotor_Spd/(2*pi);

clf
hold on
plot(Rotor_Spd_rpm,freq_lines,'b--','LineWidth',0.9)

[nbranch,nspeed] = size(eigenvalues);
for ib = 1:nbranch
    branch_col = whirl_color(branch_direction_for_plot(kappa(:,ib,:),eigenvalues(ib,:)));
    for ispd = 1:nspeed-1
        f1 = nat_freqs_Hz(ib,ispd);
        f2 = nat_freqs_Hz(ib,ispd+1);
        if ~isfinite(f1) || ~isfinite(f2)
            continue
        end
        max_jump = min(opts.max_freq_jump_hz,opts.max_freq_jump_ratio*max(f1,f2));
        if abs(f2-f1) <= max_jump
            plot(Rotor_Spd_rpm(ispd:ispd+1),nat_freqs_Hz(ib,ispd:ispd+1), ...
                '-','Color',branch_col,'LineWidth',1.8)
        end
    end
end

ymax = max([opts.max_freq_hz,1.05*max(nat_freqs_Hz(:)),1.02*max(freq_lines(:))]);
if ~isfinite(ymax) || ymax <= 0
    ymax = abs(NX)*max(Rotor_Spd_rpm)/60;
end
axis([0 max(Rotor_Spd_rpm) 0 ymax])
xlabel('Rotor spin speed (rev/min)')
ylabel(ylab)
title('Forward whirl is red; Backward whirl is green.')
if nargin >= 7 && isfield(crit,'points')
    for i = 1:numel(crit.points)
        if isfinite(crit.points(i).rpm)
            plot(crit.points(i).rpm,crit.points(i).freq,'ko','MarkerFaceColor','y','MarkerSize',6)
            text(crit.points(i).rpm,crit.points(i).freq + (0.025 + 0.035*(i-1))*ymax, ...
                sprintf('  C%d %.0fX',i,crit.points(i).order),'HorizontalAlignment','left','VerticalAlignment','bottom')
        end
    end
end
grid
hold off
end


function col = whirl_color(dir)
switch dir
    case 'forward'
        col = 'r';
    case 'backward'
        col = 'g';
    otherwise
        col = 'k';
end
end


function crit = identify_critical_speeds_from_campbell(Rotor_Spd_rpm,camp_eigs,camp_kappa,opts)
freq = abs(imag(camp_eigs))/(2*pi);
crit.points = repmat(struct('rpm',NaN,'freq',NaN,'branch',NaN,'order',NaN,'reliable',false),3,1);
candidates = repmat(struct('rpm',NaN,'freq',NaN,'branch',NaN,'order',NaN,'reliable',false),0,1);
if nargin < 4 || ~isfield(opts,'min_critical_rpm')
    min_critical_rpm = 0;
else
    min_critical_rpm = opts.min_critical_rpm;
end
for order = 1:2
    exc = order*Rotor_Spd_rpm/60;
    for ib = 1:size(freq,1)
        dir = branch_direction_for_plot(camp_kappa(:,ib,:),camp_eigs(ib,:));
        if ~strcmp(dir,'forward')
            continue
        end
        f = freq(ib,:);
        valid = isfinite(f);
        if sum(valid) < 3
            continue
        end
        d = f - exc;
        for k = 1:numel(Rotor_Spd_rpm)-1
            if ~valid(k) || ~valid(k+1)
                continue
            end
            if d(k) == 0 || d(k)*d(k+1) <= 0
                rpm = interp1(d(k:k+1),Rotor_Spd_rpm(k:k+1),0,'linear','extrap');
                if isfinite(rpm) && rpm >= min(Rotor_Spd_rpm) && rpm <= max(Rotor_Spd_rpm) && rpm >= min_critical_rpm
                    candidates(end+1).rpm = rpm; %#ok<AGROW>
                    candidates(end).freq = order*rpm/60;
                    candidates(end).branch = ib;
                    candidates(end).order = order;
                    candidates(end).reliable = true;
                    break
                end
            end
        end
    end
end
if ~isempty(candidates)
    [~,isort] = sortrows([[candidates.order].' [candidates.rpm].']);
    candidates = candidates(isort);
    keep = true(size(candidates));
    for i = 2:numel(candidates)
        if candidates(i).order == candidates(i-1).order && abs(candidates(i).rpm - candidates(i-1).rpm) < 50
            keep(i) = false;
        end
    end
    candidates = candidates(keep);
    ncopy = min(3,numel(candidates));
    crit.points(1:ncopy) = candidates(1:ncopy);
end

fprintf('\n===== 临界转速识别结果 =====\n');
names = {'1X主要临界转速','附加交点1','附加交点2'};
for i = 1:3
    if isfinite(crit.points(i).rpm)
        fprintf('%s：%.1f rpm (%.0fX)\n',names{i},crit.points(i).rpm,crit.points(i).order);
    else
        fprintf('%s：无法可靠识别\n',names{i});
    end
end
fprintf('当前工作转速：9900 rpm\n');
primary_idx = find([crit.points.order] == 1 & isfinite([crit.points.rpm]),1,'first');
if ~isempty(primary_idx)
    fprintf('当前工作转速是否低于1X主要临界转速：%s\n',yes_no(9900 < crit.points(primary_idx).rpm));
else
    fprintf('当前工作转速是否低于1X主要临界转速：无法判断\n');
end
fprintf('================================\n\n');
end


function dir = branch_direction(kappa_branch)
kk = squeeze(kappa_branch(1:4:end,:,:));
kk = kk(isfinite(kk) & abs(kk) > 1e-8);
if isempty(kk)
    dir = 'uncertain';
elseif sum(kk > 0) > 1.5*sum(kk < 0)
    dir = 'forward';
elseif sum(kk < 0) > 1.5*sum(kk > 0)
    dir = 'backward';
else
    dir = 'uncertain';
end
end


function dir = branch_direction_for_plot(kappa_branch,eig_branch)
dir = branch_direction(kappa_branch);
if ~strcmp(dir,'uncertain')
    return
end
freq = abs(imag(eig_branch))/(2*pi);
valid = isfinite(freq);
if sum(valid) < 10
    return
end
x = find(valid);
y = freq(valid);
p = polyfit(x(:),y(:),1);
if abs(p(1)) < 1e-4
    dir = 'uncertain';
elseif p(1) > 0
    dir = 'forward';
else
    dir = 'backward';
end
end


function plotloci_filtered(Rotor_Spd,eigenvalues,NX)
if nargin < 3
    NX = 1.5;
end
Rotor_Spd_rpm = Rotor_Spd*60/(2*pi);
max_axes = abs(NX)*max(Rotor_Spd_rpm)*2*pi/60;
clf
hold on
plot(real(eigenvalues.'),imag(eigenvalues.'),'b-','LineWidth',0.8)
plot(real(eigenvalues(:,1)),imag(eigenvalues(:,1)),'rx','MarkerSize',6)
plot(real(eigenvalues(:,end)),imag(eigenvalues(:,end)),'gd','MarkerSize',5)
axis([-max_axes max_axes -max_axes max_axes])
legend([num2str(Rotor_Spd_rpm(1)) ' rev/min'], ...
    [num2str(Rotor_Spd_rpm(end)) ' rev/min'])
xlabel('Real (eigenvalues)')
ylabel('Imag (eigenvalues)')
title('Filtered root locus - rotor dominant modes')
grid
hold off
end


function print_campbell_branch_summary(camp_eigs,Rotor_Spd_rpm,run_speed_rpm)
[~,irun] = min(abs(Rotor_Spd_rpm - run_speed_rpm));
fprintf('\nCampbell branch summary:\n');
for ib = 1:size(camp_eigs,1)
    f0 = branch_freq_or_nan(camp_eigs(ib,1));
    fr = branch_freq_or_nan(camp_eigs(ib,irun));
    fe = branch_freq_or_nan(camp_eigs(ib,end));
    valid = sum(isfinite(camp_eigs(ib,:)));
    interrupt_ratio = 1 - valid/size(camp_eigs,2);
    fprintf('Branch %d: f(0 rpm)=%.3f Hz, f(%.0f rpm)=%.3f Hz, f(70000 rpm)=%.3f Hz, interruption=%.1f%%\n', ...
        ib,f0,run_speed_rpm,fr,fe,100*interrupt_ratio);
    if interrupt_ratio > 0.30
        fprintf('  Warning: branch %d interruption ratio exceeds 30%%.\n',ib);
    end
end
end


function [camp_eigs,camp_kappa,camp_info] = keep_continuous_campbell_branches(camp_eigs,camp_kappa,camp_info,Rotor_Spd_rpm,opts)
nspeed = size(camp_eigs,2);
visible = false(size(camp_eigs,1),1);
fprintf('\nCampbell main-plot branch quality:\n');
for ib = 1:size(camp_eigs,1)
    valid = isfinite(camp_eigs(ib,:));
    coverage = sum(valid)/nspeed;
    [best_start,best_end,best_len] = longest_true_run(valid);
    contiguous_coverage = best_len/nspeed;
    dir = branch_direction_for_plot(camp_kappa(:,ib,:),camp_eigs(ib,:));
    visible(ib) = coverage >= opts.min_branch_coverage && ...
        contiguous_coverage >= opts.min_branch_contiguous_coverage && ...
        ~strcmp(dir,'uncertain');
    if visible(ib)
        fprintf('  Branch %d kept: direction=%s, coverage=%.1f%%, longest continuous span=%.0f-%.0f rpm\n', ...
            ib,dir,100*coverage,Rotor_Spd_rpm(best_start),Rotor_Spd_rpm(best_end));
    else
        fprintf('  Branch %d hidden from main plot: direction=%s, coverage=%.1f%%, continuous=%.1f%%\n', ...
            ib,dir,100*coverage,100*contiguous_coverage);
    end
end
if ~any(visible)
    [~,ibest] = max(sum(isfinite(camp_eigs),2));
    visible(ibest) = true;
    fprintf('  Warning: no branch met the continuity threshold; branch %d is kept for traceability.\n',ibest);
end
camp_info.branch_visible = visible;
hidden = ~visible;
camp_eigs(hidden,:) = NaN;
camp_kappa(:,hidden,:) = 0;
camp_info.selected_idx(hidden,:) = NaN;
camp_info.rotor_ratio(hidden,:) = NaN;
camp_info.damping_ratio(hidden,:) = NaN;
camp_info.ntrack_visible = sum(visible);
end


function [best_start,best_end,best_len] = longest_true_run(mask)
best_start = 1;
best_end = 1;
best_len = 0;
run_start = NaN;
for i = 1:numel(mask)
    if mask(i) && isnan(run_start)
        run_start = i;
    end
    if (~mask(i) || i == numel(mask)) && ~isnan(run_start)
        if mask(i) && i == numel(mask)
            run_end = i;
        else
            run_end = i - 1;
        end
        run_len = run_end - run_start + 1;
        if run_len > best_len
            best_len = run_len;
            best_start = run_start;
            best_end = run_end;
        end
        run_start = NaN;
    end
end
end


function f = branch_freq_or_nan(lam)
if isfinite(lam)
    f = abs(imag(lam))/(2*pi);
else
    f = NaN;
end
end


function mode_diag = build_modal_diagnostics(eigenvalues,full_eigenvectors,modal,opts)
idx = find(imag(eigenvalues) > 1e-7);
mode_diag = repmat(struct('mode_index',0,'frequency_Hz',0,'damping_ratio',0, ...
    'rotor_ratio',0,'casing_ratio',0, ...
    'rotor_energy_ratio',0,'casing_energy_ratio',0, ...
    'bearing_relative_motion_ratio',0,'shape_complexity',0, ...
    'whirl_direction','uncertain','rejection_reason','', ...
    'bearing1_rotor_amplitude',0,'bearing2_rotor_amplitude',0, ...
    'selected_for_campbell',0,'selected_for_mode_shape',0,'mode_type',''),numel(idx),1);
for i = 1:numel(idx)
    imode = idx(i);
    lam = eigenvalues(imode);
    q = full_eigenvectors(:,imode);
    qrot = q(1:modal.ndof_rotor);
    qcase = q(modal.ndof_rotor+1:end);
    qxy = qrot(rotor_xy_indices(modal.node_count_rotor));
    qcxy = qcase(rotor_xy_indices(modal.N_C+1));
    rotor_energy = norm(qxy)^2;
    casing_energy = norm(qcxy)^2;
    rr = rotor_energy/max(rotor_energy+casing_energy,eps);
    cr = casing_energy/max(rotor_energy+casing_energy,eps);
    sc = rotor_shape_complexity(qxy,modal.node_count_rotor,modal.z);
    fd = abs(imag(lam))/(2*pi);
    zeta = -real(lam)/max(abs(lam),eps);
    b1 = norm(qrot([4*modal.loc_rub(1)-3 4*modal.loc_rub(1)-2]));
    b2 = norm(qrot([4*modal.loc_rub(2)-3 4*modal.loc_rub(2)-2]));
    br = bearing_relative_motion_ratio(qrot,qcase,modal);
    wd = whirl_direction_from_rotor_xy(qxy,modal.node_count_rotor);
    mode_type = classify_mode(fd,zeta,rr,cr,sc,opts);
    if ~strcmp(mode_type,'rotor_bending') && ~strcmp(mode_type,'rotor_support_coupled') && ...
            ~strcmp(mode_type,'numerical_or_unstable') && ...
            ~strcmp(mode_type,'casing_local') && fd >= opts.min_freq_hz && ...
            zeta >= -0.02 && zeta <= opts.zeta_max && rr >= 0.30 && br >= 0.10 && ...
            sc <= opts.shape_complexity_max
        mode_type = 'coupled_rotor_bending';
    end
    mode_diag(i).mode_index = imode;
    mode_diag(i).frequency_Hz = fd;
    mode_diag(i).damping_ratio = zeta;
    mode_diag(i).rotor_ratio = rr;
    mode_diag(i).casing_ratio = cr;
    mode_diag(i).rotor_energy_ratio = rr;
    mode_diag(i).casing_energy_ratio = cr;
    mode_diag(i).bearing_relative_motion_ratio = br;
    mode_diag(i).shape_complexity = sc;
    mode_diag(i).whirl_direction = wd;
    mode_diag(i).rejection_reason = mode_rejection_reason(mode_type,fd,zeta,rr,cr,sc,opts);
    mode_diag(i).bearing1_rotor_amplitude = b1;
    mode_diag(i).bearing2_rotor_amplitude = b2;
    mode_diag(i).selected_for_campbell = 0;
    mode_diag(i).selected_for_mode_shape = 0;
    mode_diag(i).mode_type = mode_type;
end
[~,isort] = sort([mode_diag.frequency_Hz]);
mode_diag = mode_diag(isort);
end


function br = bearing_relative_motion_ratio(qrot,qcase,modal)
rel = [];
abs_motion = [];
for ib = 1:numel(modal.loc_rub)
    rn = modal.loc_rub(ib);
    cn = modal.case_bearing_nodes(ib);
    qr = qrot([4*rn-3 4*rn-2]);
    qc = qcase([4*cn-3 4*cn-2]);
    rel = [rel; qr(:)-qc(:)]; %#ok<AGROW>
    abs_motion = [abs_motion; qr(:); qc(:)]; %#ok<AGROW>
end
br = norm(rel)/max(norm(abs_motion),eps);
end


function dir = whirl_direction_from_rotor_xy(qxy,nnode)
qx = qxy(1:2:end);
qy = qxy(2:2:end);
amp = abs(qx).^2 + abs(qy).^2;
[amax,inode] = max(amp);
if isempty(inode) || amax < 1e-16
    dir = 'uncertain';
    return
end
spin_measure = imag(conj(qx(inode))*qy(inode));
tol = 0.05*abs(qx(inode))*abs(qy(inode));
if abs(spin_measure) <= tol
    dir = 'uncertain';
elseif spin_measure > 0
    dir = 'forward';
else
    dir = 'backward';
end
end


function mode_type = classify_mode(freq,zeta,rotor_ratio,casing_ratio,shape_complexity,opts)
if ~isfinite(freq) || ~isfinite(zeta) || freq > opts.max_freq_hz || zeta < -0.02 || zeta > 0.95 || shape_complexity > 1.2
    mode_type = 'numerical_or_unstable';
elseif freq < 20
    mode_type = 'rigid_or_base';
elseif freq < 50
    mode_type = 'support_mode';
elseif casing_ratio > rotor_ratio
    mode_type = 'casing_local';
elseif rotor_ratio > 0.6 && shape_complexity < opts.shape_complexity_min
    mode_type = 'rotor_support_coupled';
elseif rotor_ratio > 0.6 && shape_complexity >= opts.shape_complexity_min && ...
        shape_complexity <= opts.shape_complexity_max && zeta <= opts.mode_shape_zeta_max
    mode_type = 'rotor_bending';
elseif rotor_ratio >= 0.30 && casing_ratio < 0.70 && shape_complexity >= 0.01 && ...
        shape_complexity <= opts.shape_complexity_max && zeta >= -0.02 && zeta <= opts.zeta_max
    mode_type = 'coupled_rotor_bending';
else
    mode_type = 'support_mode';
end
end


function reason = mode_rejection_reason(mode_type,freq,zeta,rotor_ratio,casing_ratio,shape_complexity,opts)
reason = '';
if strcmp(mode_type,'rotor_bending') || strcmp(mode_type,'coupled_rotor_bending')
    return
end
if ~isfinite(freq) || ~isfinite(zeta) || zeta < -0.02 || zeta > 0.95
    reason = 'abnormal_eigenvalue_or_damping';
elseif shape_complexity < opts.shape_complexity_min && rotor_ratio > 0.6
    reason = 'too_low_shape_curvature';
elseif shape_complexity > opts.shape_complexity_max
    reason = 'too_high_shape_complexity';
elseif casing_ratio > rotor_ratio
    reason = 'casing_energy_dominant';
elseif freq < opts.min_freq_hz
    reason = 'low_frequency_support_or_base_mode';
else
    reason = 'not_selected_as_rotor_bending';
end
end


function mode_diag = mark_selected_modes(mode_diag,camp_info,camp_eigs,Rotor_Spd_rpm_all,run_speed_rpm,mode_idx)
[~,iref] = min(abs(Rotor_Spd_rpm_all - run_speed_rpm));
camp_idx = camp_info.selected_idx(:,iref);
camp_idx = camp_idx(isfinite(camp_idx));
for i = 1:numel(mode_diag)
    if any(mode_diag(i).mode_index == camp_idx)
        mode_diag(i).selected_for_campbell = 1;
    end
    if any(mode_diag(i).mode_index == mode_idx)
        mode_diag(i).selected_for_mode_shape = 1;
    end
end
end


function write_modal_diagnostics_csv(mode_diag,outfile)
outdir = fileparts(outfile);
if ~exist(outdir,'dir')
    mkdir(outdir);
end
fid = fopen(outfile,'w');
assert(fid > 0,'Cannot write modal diagnostics CSV.');
fprintf(fid,'mode_index,frequency_Hz,damping_ratio,rotor_energy_ratio,casing_energy_ratio,bearing_relative_motion_ratio,shape_complexity,whirl_direction,mode_type,rejection_reason,selected_for_campbell,selected_for_mode_shape,bearing1_rotor_amplitude,bearing2_rotor_amplitude\n');
for i = 1:numel(mode_diag)
    fprintf(fid,'%d,%.10g,%.10g,%.10g,%.10g,%.10g,%.10g,%s,%s,%s,%d,%d,%.10g,%.10g\n', ...
        mode_diag(i).mode_index,mode_diag(i).frequency_Hz,mode_diag(i).damping_ratio, ...
        mode_diag(i).rotor_energy_ratio,mode_diag(i).casing_energy_ratio, ...
        mode_diag(i).bearing_relative_motion_ratio,mode_diag(i).shape_complexity, ...
        mode_diag(i).whirl_direction,mode_diag(i).mode_type, ...
        mode_diag(i).rejection_reason, ...
        mode_diag(i).selected_for_campbell,mode_diag(i).selected_for_mode_shape, ...
        mode_diag(i).bearing1_rotor_amplitude,mode_diag(i).bearing2_rotor_amplitude);
end
fclose(fid);
fprintf('Modal diagnostics written to: %s\n',outfile);
end


function mode_idx = modal_display_mode_indices(mode_diag,eigenvalues,full_eigenvectors,modal,nmode,opts)
is_rotor = strcmp({mode_diag.mode_type},'rotor_bending');
idx = [mode_diag(is_rotor).mode_index];
freq = [mode_diag(is_rotor).frequency_Hz];
[~,isort] = sort(freq);
idx = idx(isort);
selected_freq_limit = inf;
if ~isempty(freq)
    selected_freq_limit = max(freq(isort(1:min(numel(isort),nmode))));
end
skipped = mode_diag(~is_rotor & [mode_diag.frequency_Hz] >= opts.min_freq_hz & ...
    [mode_diag.frequency_Hz] <= selected_freq_limit);
for iskip = 1:numel(skipped)
    fprintf('Skipped mode %.0f: freq = %.3f Hz, mode_type = %s\n', ...
        skipped(iskip).mode_index,skipped(iskip).frequency_Hz,skipped(iskip).mode_type);
end
if numel(idx) < nmode
    fprintf('Mode-shape warning: only %d rotor_bending modes were found; using best non-numerical fallback modes for the remaining panels.\n',numel(idx));
    valid_type = ~strcmp({mode_diag.mode_type},'numerical_or_unstable') & ...
        [mode_diag.frequency_Hz] >= opts.min_freq_hz & ...
        [mode_diag.frequency_Hz] <= opts.max_freq_hz;
    fallback = [mode_diag(valid_type).mode_index];
    fallback_freq = [mode_diag(valid_type).frequency_Hz];
    [~,iall] = sort(fallback_freq);
    fallback = fallback(iall);
    fallback = fallback(~ismember(fallback,idx));
    idx = [idx(:); fallback(:)];
end
if numel(idx) < nmode && ~isempty(idx)
    idx = [idx(:); repmat(idx(end),nmode-numel(idx),1)];
end
mode_idx = idx(1:nmode);

fprintf('\nSelected rotor-dominant mode shapes at %.0f rpm:\n',modal.run_speed_rpm);
for i = 1:min(nmode,numel(mode_idx))
    row = find([mode_diag.mode_index] == mode_idx(i),1,'first');
    if isempty(row)
        lam = eigenvalues(mode_idx(i));
        fprintf('Selected mode %d: freq = %.3f Hz, rotor_ratio = n/a, mode_type = fallback\n', ...
            i,abs(imag(lam))/(2*pi));
    else
        fprintf('Selected mode %d: freq = %.3f Hz, rotor_ratio = %.3f, mode_type = %s, whirl = %s\n', ...
            i,mode_diag(row).frequency_Hz,mode_diag(row).rotor_energy_ratio, ...
            mode_diag(row).mode_type,mode_diag(row).whirl_direction);
    end
end
end


function lam_plot = damped_lambda_for_plot(lam)
lam_plot = 1i*abs(imag(lam));
end


function plot_rotor_bending_mode(model,Mode,eigenvalue,mode_diag,mode_index,display_index)
plotmode(model,Mode,damped_lambda_for_plot(eigenvalue))
row = find([mode_diag.mode_index] == mode_index,1,'first');
if ~isempty(row)
    title({sprintf('Mode %d: Rotor bending',display_index), ...
        sprintf('f = %.3f Hz, rotor ratio = %.2f', ...
        mode_diag(row).frequency_Hz,mode_diag(row).rotor_energy_ratio)}, ...
        'FontSize',10)
else
    title({sprintf('Mode %d: Rotor bending',display_index), ...
        sprintf('f = %.3f Hz',abs(imag(eigenvalue))/(2*pi))}, ...
        'FontSize',10)
end
end


function modal_result_check(modal,Rotor_Spd_rpm,Rotor_Spd_rpm_all,eigenvalues_9900,eigenvalues_all,camp_eigs,camp_info,camp_opts,mode_diag_9900)
all_freq_9900 = positive_damped_freqs(eigenvalues_9900);
elastic_freq = all_freq_9900(all_freq_9900 > 1.0);
[~,iref] = min(abs(Rotor_Spd_rpm_all - Rotor_Spd_rpm));
tracked_freq = abs(imag(camp_eigs(:,iref)))/(2*pi);
tracked_freq = sort(tracked_freq(isfinite(tracked_freq)));
tracked_freq = tracked_freq(tracked_freq >= camp_opts.min_freq_hz);
diag_rotor = strcmp({mode_diag_9900.mode_type},'rotor_bending');
main_freq = sort([mode_diag_9900(diag_rotor).frequency_Hz]);
run_1x = Rotor_Spd_rpm/60;
next_major = main_freq(find(main_freq > 1.02*run_1x,1,'first'));
if isempty(elastic_freq), first_elastic = NaN; else, first_elastic = elastic_freq(1); end
if isempty(main_freq)
    first_main = NaN;
    first_modes = NaN(1,6);
else
    first_main = main_freq(1);
    first_modes = NaN(1,6);
    nshow = min(6,numel(main_freq));
    first_modes(1:nshow) = main_freq(1:nshow);
end
if isempty(next_major), next_major = NaN; end

max_real = max(real(eigenvalues_9900));
stable = max_real <= 1e-6;
true_zero_count = sum(abs(eigenvalues_9900) < 1e-3);
low_branch_count = sum(abs(imag(eigenvalues_9900))/(2*pi) < camp_opts.min_freq_hz);

freq0 = abs(imag(camp_eigs(:,1)))/(2*pi);
freqh = abs(imag(camp_eigs(:,end)))/(2*pi);
split_flag = detect_whirl_split(freq0(isfinite(freq0)),freqh(isfinite(freqh)));

K_nobearing = modal.K - modal.Kbc;
freq_nobearing = modal_first_frequency(modal.M,K_nobearing,modal.C,modal.G,0,camp_opts.min_freq_hz);
if isnan(freq_nobearing) || isnan(first_main)
    bearing_influence = 'cannot judge';
else
    rel_change = abs(first_main - freq_nobearing)/max(first_main,eps);
    if rel_change > 0.05
        bearing_influence = 'yes';
    else
        bearing_influence = 'no';
    end
end

fprintf('\n===== 固有特性分析结果检查 =====\n');
fprintf('当前模型：转子-双轴承-机匣耦合系统\n');
fprintf('Campbell筛选频段：%.0f-%.0f Hz\n',camp_opts.min_freq_hz,camp_opts.max_freq_hz);
fprintf('Campbell参考转速：%.0f rpm\n',camp_opts.reference_rpm);
fprintf('转子轴承节点：%d, %d\n',modal.loc_rub(1),modal.loc_rub(2));
fprintf('机匣轴承座节点：%d, %d\n',modal.case_bearing_nodes(1),modal.case_bearing_nodes(2));
fprintf('完整系统最低弹性频率 = %.3f Hz\n',first_elastic);
fprintf('筛选后第一阶转子主导固有频率 = %.3f Hz\n',first_main);
fprintf('前四阶转子主导频率 = ');
fprintf('%.3f ',first_modes(1:min(4,sum(~isnan(first_modes)))));
fprintf('Hz\n');
fprintf('前六阶转子主导阻尼固有频率 = ');
fprintf('%.3f ',first_modes(~isnan(first_modes)));
fprintf('Hz\n');
fprintf('当前转速 = %.0f rpm\n',Rotor_Spd_rpm);
fprintf('当前转速下 1X = %.3f Hz\n',run_1x);
fprintf('当前1X以上首个主要临界参考频率 = %.3f Hz\n',next_major);
fprintf('当前转速是否低于该主要临界参考：%s\n',yes_no(run_1x < next_major));
fprintf('Campbell显示频率范围 = %.1f ~ %.1f Hz\n',camp_opts.min_freq_hz,camp_opts.max_freq_hz);
fprintf('实际追踪转子主导分支数量 = %d\n',camp_info.ntrack);
fprintf('是否出现前后向涡动分裂：%s\n',yes_no(split_flag));
fprintf('最大特征值实部 = %.6e\n',max_real);
if stable
    fprintf('稳定性判断：稳定\n');
else
    fprintf('稳定性判断：可能失稳\n');
end
fprintf('真零频特征值数量 = %d\n',true_zero_count);
fprintf('剔除的低频刚体/基础模态数量 = %d\n',low_branch_count);
fprintf('剔除的机匣局部模态数量 = %d\n',round(sum(camp_info.rejected_casing)/numel(camp_info.rejected_casing)));
fprintf('剔除的数值/异常模态数量 = %d\n',round(sum(camp_info.rejected_numeric)/numel(camp_info.rejected_numeric)));
fprintf('轴承刚度是否显著影响低阶频率：%s\n',bearing_influence);
fprintf('轴承1 kx/ky/cx/cy = %.3e / %.3e / %.3e / %.3e\n', ...
    modal.bearing.kx(1),modal.bearing.ky(1),modal.bearing.cx(1),modal.bearing.cy(1));
fprintf('轴承2 kx/ky/cx/cy = %.3e / %.3e / %.3e / %.3e\n', ...
    modal.bearing.kx(2),modal.bearing.ky(2),modal.bearing.cx(2),modal.bearing.cy(2));
fprintf('叶轮盘转动惯量开关 include_disk_inertia = %d\n',modal.include_disk_inertia);
fprintf('当前实际使用的圆盘等效质量 disk_masses = [%.4f %.4f %.4f %.4f] kg\n',modal.disk_masses);
for id = 1:numel(modal.disk_info)
    fprintf('叶轮节点 %d: mass = %.3f kg, Jd = %.6e kg*m^2, Jp = %.6e kg*m^2\n', ...
        modal.disk_info(id).node,modal.disk_info(id).mass,modal.disk_info(id).Jd,modal.disk_info(id).Jp);
end
if modal.mass_fix_rotor.shift > 0 || modal.mass_fix_case.shift > 0
    fprintf('质量矩阵提示：已进行最小谱正定化；转子 %.3e，机匣 %.3e。\n', ...
        modal.mass_fix_rotor.shift,modal.mass_fix_case.shift);
end
fprintf('Mr最小特征值 = %.6e, Mc最小特征值 = %.6e, M最小特征值 = %.6e\n', ...
    modal.matrix_check.Mr_min_eig,modal.matrix_check.Mc_min_eig,modal.matrix_check.M_min_eig);
fprintf('Kc最小特征值 = %.6e\n',modal.matrix_check.Kc_min_eig);
fprintf('零质量自由度数量：Mr=%d, Mc=%d, M=%d\n', ...
    numel(modal.matrix_check.Mr_zero_dof),numel(modal.matrix_check.Mc_zero_dof),numel(modal.matrix_check.M_zero_dof));
fprintf('说明：低频耦合/基础分支保留在检查中，Campbell图仅筛选标准低阶正频分支显示。\n');
fprintf('================================\n\n');
end


function compare_modal_model_levels(modal,opts)
K_ground_current = modal.Kr;
C_ground_current = modal.Cr;
K_ground_stiff = modal.Kr;
C_ground_stiff = modal.Cr;
for ib = 1:numel(modal.bearing.rotor_nodes)
    node = modal.bearing.rotor_nodes(ib);
    ix = 4*node - 3;
    iy = 4*node - 2;
    K_ground_current(ix,ix) = K_ground_current(ix,ix) + modal.bearing.kx(ib);
    K_ground_current(iy,iy) = K_ground_current(iy,iy) + modal.bearing.ky(ib);
    C_ground_current(ix,ix) = C_ground_current(ix,ix) + modal.bearing.cx(ib);
    C_ground_current(iy,iy) = C_ground_current(iy,iy) + modal.bearing.cy(ib);
    K_ground_stiff(ix,ix) = K_ground_stiff(ix,ix) + 2.5e9;
    K_ground_stiff(iy,iy) = K_ground_stiff(iy,iy) + 2.5e9;
    C_ground_stiff(ix,ix) = C_ground_stiff(ix,ix) + 5e3;
    C_ground_stiff(iy,iy) = C_ground_stiff(iy,iy) + 5e3;
end
fA = modal_first_frequencies(modal.Mr,K_ground_current,C_ground_current,modal.Gr,0,opts.min_freq_hz,3);
fB = modal_first_frequencies(modal.Mr,K_ground_stiff,C_ground_stiff,modal.Gr,0,opts.min_freq_hz,3);
fC = modal_first_frequencies(modal.M,modal.K,modal.C,modal.G,0,opts.min_freq_hz,3);
fprintf('\n===== 模型层级固有频率对比 =====\n');
fprintf('单转子对地支承：f1,f2,f3 = %.3f, %.3f, %.3f Hz\n',fA(1),fA(2),fA(3));
fprintf('固定机匣轴承座：f1,f2,f3 = %.3f, %.3f, %.3f Hz\n',fB(1),fB(2),fB(3));
fprintf('柔性机匣耦合：f1,f2,f3 = %.3f, %.3f, %.3f Hz\n',fC(1),fC(2),fC(3));
if isfinite(fA(1)) && isfinite(fC(1)) && abs(fA(1)-fC(1))/max(fA(1),eps) > 0.5
    fprintf('提示：层级模型一阶频率差异较大，请重点检查轴承刚度、机匣边界、轴承座节点映射和盘质量/惯量。\n');
end
fprintf('================================\n\n');
end


function freqs = positive_damped_freqs(eigs_in)
freqs = abs(imag(eigs_in(imag(eigs_in) > 1e-7)))/(2*pi);
freqs = sort(freqs(:).');
end


function flag = detect_whirl_split(freq0,freqh)
n = min([8 numel(freq0) numel(freqh)]);
flag = false;
if n >= 4
    for i = 1:2:n-1
        split0 = abs(freq0(i+1)-freq0(i));
        splith = abs(freqh(i+1)-freqh(i));
        if splith > split0 + 0.5
            flag = true;
            return
        end
    end
end
end


function f1 = modal_first_frequency(M,K,C,G,Omega,min_freq_hz)
freqs = modal_first_frequencies(M,K,C,G,Omega,min_freq_hz,1);
f1 = freqs(1);
end


function freqs_out = modal_first_frequencies(M,K,C,G,Omega,min_freq_hz,nfreq)
n = size(M,1);
A = [zeros(n) eye(n); -(M\K) -(M\(C + Omega*G))];
lam = eig(A);
freqs = positive_damped_freqs(lam);
freqs = freqs(freqs > min_freq_hz);
freqs_out = NaN(1,nfreq);
if ~isempty(freqs)
    ncopy = min(nfreq,numel(freqs));
    freqs_out(1:ncopy) = freqs(1:ncopy);
end
end


function out = yes_no(flag)
if flag
    out = '是';
else
    out = '否';
end
end
