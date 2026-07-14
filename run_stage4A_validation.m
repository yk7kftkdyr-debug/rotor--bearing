function result = run_stage4A_validation()
%RUN_STAGE4A_VALIDATION Static Stage4-A validation with front axial location.
params=initial_conditions(); static=solve_static_equilibrium(params,false); [M,~,~,modelInfo]=build_rotor_case_model(params); params.modelInfo=modelInfo;
result=stage4A_static_check(params,M,static.K_s,static.q0,static.F_bearing_static,static.bearing_state,modelInfo);
result.static_residual=static.normalized_residual; result.axial_location=static.axial_location; result.lambda_N=static.axial_location.lambda_N; result.front_fz_lambda_error_N=abs(result.force(1,3)-result.lambda_N); result.interface_verification_pass=result.checks.pass; result.stage4A_static_physical_pass=result.checks.pass && result.static_residual<1e-6 && abs(result.force(1,3))>1e-12 && result.front_fz_lambda_error_N<1e-6;
fprintf('Stage4-A axial location: residual=%.3e, Fz_front=%.6g N, Fz_rear=%.6g N, lambda=%.6g N, Fz-lambda error=%.3e N, pass=%d\n',result.static_residual,result.force(1,3),result.force(2,3),result.lambda_N,result.front_fz_lambda_error_N,result.stage4A_static_physical_pass);
write_stage4A_report(fullfile(pwd,'Stage4A_axial_location_report.txt'),params,result);
end

function result=stage4A_static_check(params,~,~,q,Fb,state,modelInfo)
result.force=zeros(2,5); result.action_reaction_error=zeros(1,2); result.tangent=cell(1,2); result.loaded_count=zeros(1,2); result.max_contact_load=zeros(1,2); result.contact_angle=zeros(1,2); result.c_work=zeros(1,2); result.messages=cell(1,2);
for ib=1:2
    b=params.bearing(ib); ir=6*b.rotor_node+(-5:-1); ic=modelInfo.num_rotor_dof+6*b.case_node+(-5:-1); result.force(ib,:)=Fb(ir).'; result.action_reaction_error(ib)=norm(Fb(ir)+Fb(ic)); result.tangent{ib}=local_tangent(q,params,ir); st=state.bearings(ib); result.loaded_count(ib)=st.loaded_count; result.max_contact_load(ib)=st.max_contact_load; result.contact_angle(ib)=st.contact_angle; result.c_work(ib)=st.c_work; result.messages{ib}=st.message;
end
zero=F_bearing(zeros(size(q)),zeros(size(q)),params,numel(q)); result.zero_force_norm=norm(zero); result.global_generalized_balance=norm(sum(result.force,1)*0); result.roller_slice_validation=roller_slice_validation(q,params,modelInfo);
ball_angle_deg=result.contact_angle(1)*180/pi; result.checks.nonnegative_contact=all(cellfun(@(s) all(s.Q(:)>=0),num2cell(state.bearings))); result.checks.action_reaction=max(result.action_reaction_error)<=1e-12; result.checks.zero_clearance_state=result.zero_force_norm<=1e-10; result.checks.ball_angle_range=ball_angle_deg>=1 && ball_angle_deg<=32; result.checks.floating_rear_fz=abs(result.force(2,3))<=1e-12; result.checks.pass=all(struct2array(result.checks));
fprintf('Stage4-A forces [Fx Fy Fz Mx My]: ball=%s, roller=%s\n',mat2str(result.force(1,:),8),mat2str(result.force(2,:),8));
fprintf('Stage4-A checks: action=%.3e, zero=%.3e, loaded=[%d %d], Qmax=[%.3e %.3e], angle=%.3fdeg, slice9to13=%.3e, pass=%d\n',max(result.action_reaction_error),result.zero_force_norm,result.loaded_count,result.max_contact_load,ball_angle_deg,result.roller_slice_validation.relative_force_change,result.checks.pass);
assert(result.checks.pass,'Stage4-A interface verification failed.');
end

function Kb=local_tangent(q,params,ir)
h=1e-8; Kb=zeros(5); for j=1:5, dq=zeros(size(q)); dq(ir(j))=h; fp=F_bearing(q+dq,zeros(size(q)),params,numel(q)); fm=F_bearing(q-dq,zeros(size(q)),params,numel(q)); Kb(:,j)=(fp(ir)-fm(ir))/(2*h); end
end

function out=roller_slice_validation(q,params,~)
b=params.bearing; b(2).stage4A_slice_count=b(2).stage4A_slice_validation_count; p=params; p.bearing=b; [F13,~]=F_bearing(q,zeros(size(q)),p,numel(q)); ir=6*b(2).rotor_node+(-5:-1); [F9,~]=F_bearing(q,zeros(size(q)),params,numel(q)); out.force9=F9(ir).'; out.force13=F13(ir).'; out.relative_force_change=norm(out.force13-out.force9)/max(norm(out.force13),1e-12); out.slice_count_9=9; out.slice_count_13=13;
end

function write_stage4A_report(file_name,params,result)
fid=fopen(file_name,'wt'); assert(fid>=0,'Cannot create Stage4A_axial_location_report.txt.'); c=onCleanup(@()fclose(fid)); fprintf(fid,'Stage4-A axial-location static report\nstatic_physical_pass=%d\ninterface_verification_pass=%d\n\n',result.stage4A_static_physical_pass,result.interface_verification_pass);
fprintf(fid,'confirmed_parameters:\n'); disp_struct(fid,params.stage4A.confirmed_parameters,''); fprintf(fid,'\nengineering_assumption_parameters (NOT_CALIBRATED):\n'); disp_struct(fid,params.stage4A.engineering_assumption_parameters,''); fprintf(fid,'\nfuture_required_parameters:\n'); disp_struct(fid,params.stage4A.future_required_parameters,'');
fprintf(fid,'\nforces [Fx Fy Fz Mx My]\nball %s\nroller %s\n',mat2str(result.force(1,:),12),mat2str(result.force(2,:),12));
fprintf(fid,'\naxial_location: rotor_node=%d case_node=%d gap_m=%.12e clearance_m=%.12e lambda_N=%.12e Fz_front_N=%.12e Fz_rear_N=%.12e Fz_minus_lambda_error_N=%.12e static_residual=%.12e\n',result.axial_location.rotor_node,result.axial_location.case_node,result.axial_location.gap_m,result.axial_location.clearance_m,result.lambda_N,result.force(1,3),result.force(2,3),result.front_fz_lambda_error_N,result.static_residual);
for ib=1:2, fprintf(fid,'\nBearing %d 5x5 tangent:\n',ib); fprintf(fid,'%.12e %.12e %.12e %.12e %.12e\n',result.tangent{ib}.'); end
fprintf(fid,'\nchecks: action_reaction=%s, loaded=%s, Qmax=%s, angle_deg=%s, zero_force=%.12e, slice_change=%.12e\n',mat2str(result.action_reaction_error,12),mat2str(result.loaded_count),mat2str(result.max_contact_load,12),mat2str(result.contact_angle*180/pi,12),result.zero_force_norm,result.roller_slice_validation.relative_force_change);
end

function disp_struct(fid,s,prefix)
names=fieldnames(s); for k=1:numel(names), v=s.(names{k}); if isstruct(v), fprintf(fid,'%s%s:\n',prefix,names{k}); disp_struct(fid,v,[prefix '  ']); elseif isempty(v), fprintf(fid,'%s%s = unknown\n',prefix,names{k}); elseif ischar(v), fprintf(fid,'%s%s = %s\n',prefix,names{k},v); else, fprintf(fid,'%s%s = %s\n',prefix,names{k},mat2str(v)); end, end
end
