function result = run_stage4B0_validation(mode)
%RUN_STAGE4B0_VALIDATION Zero-dynamic-load acceptance for Stage4-B only.
if nargin>0 && strcmpi(mode,'B1'), result=run_stage4B1_validation(); return; end
params=initial_conditions(); static=solve_static_equilibrium(params,false); assert(static.stage4A_static_physical_pass || static.normalized_residual<1e-6,'Stage4-A static work point is not valid.');
params.static_equilibrium_result=static; params.stage4B.enable=true; params.stage4B.mode='B0_zero_dynamic_load'; params.unbalance.nodes=[]; params.unbalance.mass_g=[]; params.unbalance.ecc_mm=[]; params.unbalance.phase=[];
sim=newmark_newton_multi(params); result.max_u=max(abs(sim.u),[],'all'); result.max_delta_fb=max(abs(sim.Fb_increment_hist),[],'all'); result.max_axial_constraint_error=max(sim.solver.axial_constraint_error_hist); result.max_newton_residual=max(sim.solver.err_R_hist); result.unconverged_steps=sim.solver.unconverged_steps; result.lambda_increment_max=max(abs(sim.solver.lambda_axial_hist)); result.local_tangent_size_pass=all(cellfun(@(x) isequal(size(x),[5 5]),sim.solver.bearing_Kb_last)) && all(cellfun(@(x) isequal(size(x),[5 5]),sim.solver.bearing_Cb_last)); result.pass=result.max_u<=1e-12 && result.max_delta_fb<=1e-10 && result.max_axial_constraint_error<=1e-12 && result.unconverged_steps==0 && result.local_tangent_size_pass;
fprintf('Stage4-B0: max|u|=%.3e, max|DeltaFb|=%.3e, max|Bu|=%.3e, max Newton residual=%.3e, max|lambda_dyn|=%.3e, Kb/Cb 5x5=%d, unconverged=%d, pass=%d\n',result.max_u,result.max_delta_fb,result.max_axial_constraint_error,result.max_newton_residual,result.lambda_increment_max,result.local_tangent_size_pass,result.unconverged_steps,result.pass);
assert(result.pass,'Stage4-B0 zero-dynamic-load acceptance failed.');
end

function result=run_stage4B1_validation()
params=initial_conditions(); static=solve_static_equilibrium(params,false); params.static_equilibrium_result=static; params.stage4B.enable=true; params.stage4B.mode='B1_small_harmonic'; params.unbalance.nodes=[]; params.unbalance.mass_g=[]; params.unbalance.ecc_mm=[]; params.unbalance.phase=[];
specs={struct('name','transverse','type','transverse_x','amplitude',1,'node',16),struct('name','axial','type','axial_z','amplitude',1,'node',16),struct('name','moment','type','moment_x','amplitude',0.1,'node',16)}; result.cases=cell(1,numel(specs));
for k=1:numel(specs), result.cases{k}=run_small_case(params,specs{k},64,false); result.linear{k}=run_small_case(params,specs{k},64,true); result.fine{k}=run_small_case(params,specs{k},128,false); end
result.linear_error=cellfun(@(a,b) norm(a.monitor-b.monitor)/max(norm(b.monitor),1e-15),result.cases,result.linear); result.dt_error=cellfun(@(a,b) abs(a.metrics-b.metrics)./max(abs(b.metrics),1e-15),result.cases,result.fine,'UniformOutput',false); slice13_spec=specs{1}; slice13_spec.slice_count=13; result.slice13=run_small_case(params,slice13_spec,64,false); result.slice_relative_error=norm(result.cases{1}.monitor-result.slice13.monitor)/max(norm(result.slice13.monitor),1e-15); result.pass=all(cellfun(@(x) x.pass,result.cases)) && all(result.linear_error<0.05) && all(cellfun(@(x) all(x<0.03),result.dt_error)) && result.slice_relative_error<0.05;
fprintf('Stage4-B1: linear errors=%s, dt errors=%s, roller 9/13 error=%.3e, pass=%d\n',mat2str(result.linear_error,3),mat2str(cell2mat(result.dt_error),3),result.slice_relative_error,result.pass); assert(result.pass,'Stage4-B1 validation failed.');
end

function out=run_small_case(base,spec,fen,linearized)
p=base; p.Fen=fen; p.n_Fen=1; p.stage4B.linearized_bearing=linearized; p.stage4B.excitation=struct('type',spec.type,'amplitude',spec.amplitude,'node',spec.node,'omega',p.omega,'phase',0); if isfield(spec,'slice_count'), p.bearing(2).stage4A_slice_count=spec.slice_count; end
sim=newmark_newton_multi(p); idx=monitor_dof(spec); out.monitor=sim.u(idx,:); out.metrics=[max(abs(out.monitor)) sqrt(mean(out.monitor.^2)) abs(sum(out.monitor.*exp(-1i*p.omega*sim.time))/numel(sim.time))]; out.max_bu=max(sim.solver.axial_constraint_error_hist); out.max_front_fz=max(abs(sim.Fb_global_hist(6*p.bearing(1).rotor_node-3,:))); out.max_rear_fz=max(abs(sim.Fb_global_hist(6*p.bearing(2).rotor_node-3,:))); out.action_reaction=max_bearing_action_reaction(sim,p); out.unconverged=sim.solver.unconverged_steps; out.residual_monotone=all(cellfun(@residual_descends,sim.solver.residual_history)); out.pass=out.max_bu<=1e-10 && out.max_front_fz>1e-12 && out.max_rear_fz<=1e-12 && out.action_reaction<=1e-10 && out.unconverged==0 && out.residual_monotone;
fprintf('  %s linear=%d: peak=%.3e, Bu=%.3e, Fz=[%.3e %.3e], action=%.3e, unconverged=%d, monotone=%d\n',spec.name,linearized,out.metrics(1),out.max_bu,out.max_front_fz,out.max_rear_fz,out.action_reaction,out.unconverged,out.residual_monotone);
end

function idx=monitor_dof(spec), if strcmp(spec.type,'transverse_x'), idx=6*spec.node-5; elseif strcmp(spec.type,'axial_z'), idx=6*spec.node-3; else, idx=6*spec.node-2; end, end
function e=max_bearing_action_reaction(sim,p), e=0; for ib=1:numel(p.bearing), ir=6*p.bearing(ib).rotor_node+(-5:-1); ic=sim.modelInfo.num_rotor_dof+6*p.bearing(ib).case_node+(-5:-1); e=max(e,max(vecnorm(sim.Fb_global_hist(ir,:)+sim.Fb_global_hist(ic,:)))); end, end
function yes=residual_descends(x), yes=isempty(x) || all(diff(x)<=1e-12); end
