function sim = newmark_newton_multi(params)
%NEWMARK_NEWTON_MULTI Full-Newton nonlinear perturbation dynamics about q0.
% q_total=q0+u; M*u_ddot+C_s*u_dot+K_s*u=F_dynamic+dF_b-B_axial'*lambda.

if isfield(params,'case_definition') && isfield(params.case_definition,'name') && strcmp(params.case_definition.name,'stage1_static_equilibrium_ball_roller')
    validate_official_case_config(params,'stage2 full nonlinear dynamics');
end
[MM,CC,KK,modelInfo] = build_rotor_case_model(params);
params.modelInfo=modelInfo; params.num_rotor_dof=modelInfo.num_rotor_dof; params.num_case_dof=modelInfo.num_case_dof;
if ~isfield(params,'static_equilibrium_result') || ~isfield(params.static_equilibrium_result,'q0'), error('Stage 2 requires static_equilibrium_result.q0.'); end
N=size(MM,1); q0=params.static_equilibrium_result.q0; nb=numel(params.bearing); axial=dynamic_axial_constraint(params,modelInfo,N);
if numel(q0)~=N, error('Static q0 length does not match the Stage-2 structural model.'); end
wi=params.omega; h=2*pi/wi/params.Fen; nt=params.Fen*params.n_Fen; time=(0:nt)*h; gamma=params.gamma; beta=params.beta;
a0=1/(beta*h^2); a1=gamma/(beta*h); a2=1/(beta*h); a3=1/(2*beta)-1;
max_iter=solver_value(params,'max_iter',20); tol_u=solver_value(params,'tol_u',1e-8); tol_R=solver_value(params,'tol_R',1e-6); u_ref=solver_value(params,'u_ref',1e-9); F_ref=solver_value(params,'F_ref',1); hu0=solver_value(params,'bearing_tangent_u',1e-8); hv0=solver_value(params,'bearing_tangent_v',1e-6);
params.current_time=0; [Fb0,state0]=F_bearing(q0,zeros(N,1),params,N);
if stage4b_value(params,'linearized_bearing',false), [~,~,~,Klinear,Clinear]=local_bearing_tangent(q0,zeros(N,1),zeros(N,1),Fb0,params,N,hu0,hv0,a1); params.stage4B.linear_Kglobal=Klinear; params.stage4B.linear_Cglobal=Clinear; end
u=zeros(N,nt+1); v=zeros(N,nt+1); a=zeros(N,nt+1); q_total=q0+u; Fb_total=zeros(N,nt+1); Fb_inc=zeros(N,nt+1); Fb_total(:,1)=Fb0;
F_b_hist=zeros(2*nb,nt+1); loaded_count_hist=zeros(nb,nt+1); slip_hist=zeros(nb,nt+1); delta_max_hist=zeros(nb,nt+1); oil_film_min_hist=zeros(nb,nt+1); contact_stiffness_hist=zeros(nb,nt+1); clearance_work_hist=zeros(nb,nt+1);
[F_b_hist,loaded_count_hist,slip_hist,delta_max_hist,oil_film_min_hist,contact_stiffness_hist,clearance_work_hist]=store_state(state0,1,F_b_hist,loaded_count_hist,slip_hist,delta_max_hist,oil_film_min_hist,contact_stiffness_hist,clearance_work_hist);
err_u=zeros(1,nt+1); err_R=zeros(1,nt+1); iter_hist=zeros(1,nt+1); converged=true(1,nt+1); line_search_hist=ones(1,nt+1); residual_history=cell(nt+1,1); axial_constraint_error=zeros(1,nt+1); lambda_axial_hist=zeros(1,nt+1); last_Kb=cell(1,nb); last_Cb=cell(1,nb);
if axial.enabled, [~,last_Kb,last_Cb]=local_bearing_tangent(q0,zeros(N,1),zeros(N,1),Fb0,params,N,hu0,hv0,a1); end
for n=1:nt
    t=time(n+1); params.current_time=t; Fdyn=startup_ramp_factor(t,wi,params)*unbalance_force(t,params,N)+stage4b_excitation_force(t,params,N); [Fb0t,~]=F_bearing(q0,zeros(N,1),params,N);
    un=u(:,n); vn=v(:,n); an=a(:,n); ui=un+h*vn+h^2*(0.5-beta)*an; hist=[]; ok=false; lambda=1; lambda_axial=lambda_axial_hist(n);
    for iter=1:max_iter
        [ai,vi]=newmark_state(ui,un,vn,an,h,gamma,a0,a2,a3); [dFb,~]=bearing_increment(q0,ui,vi,Fb0t,params,N);
        Rstruct=MM*ai+CC*vi+KK*ui-Fdyn-dFb+axial.B'*lambda_axial; R=[Rstruct; axial.B*ui]; eR=norm(R)/max(norm(Fdyn)+norm(dFb)+F_ref,F_ref); hist(end+1)=eR; %#ok<AGROW>
        if eR<=tol_R, eu=0; ok=true; break; end
        [Jb,last_Kb,last_Cb]=local_bearing_tangent(q0,ui,vi,Fb0t,params,N,hu0,hv0,a1); J=a0*MM+a1*CC+KK-Jb;
        dKKT=-safe_dynamic_kkt_solve(J,axial.B,R); du=dKKT(1:N); dlambda=dKKT(end); eu=norm(du)/max(norm(ui),u_ref);
        [lambda,ui,lambda_axial]=line_search(ui,lambda_axial,du,dlambda,un,vn,an,h,gamma,a0,a2,a3,MM,CC,KK,Fdyn,q0,Fb0t,params,N,axial.B,norm(R));
    end
    [anew,vnew]=newmark_state(ui,un,vn,an,h,gamma,a0,a2,a3); [dFb,staten]=bearing_increment(q0,ui,vnew,Fb0t,params,N); Rstruct=MM*anew+CC*vnew+KK*ui-Fdyn-dFb+axial.B'*lambda_axial; R=[Rstruct; axial.B*ui];
    eR=norm(R)/max(norm(Fdyn)+norm(dFb)+F_ref,F_ref); ok=ok || (eR<=tol_R && eu<=tol_u);
    if ~ok, warning('Full Newton did not converge at step %d: R=%.3e, du=%.3e.',n,eR,eu); end
    u(:,n+1)=ui; v(:,n+1)=vnew; a(:,n+1)=anew; q_total(:,n+1)=q0+ui; Fb_inc(:,n+1)=dFb; Fb_total(:,n+1)=Fb0t+dFb;
    [F_b_hist,loaded_count_hist,slip_hist,delta_max_hist,oil_film_min_hist,contact_stiffness_hist,clearance_work_hist]=store_state(staten,n+1,F_b_hist,loaded_count_hist,slip_hist,delta_max_hist,oil_film_min_hist,contact_stiffness_hist,clearance_work_hist);
    err_u(n+1)=eu; err_R(n+1)=eR; iter_hist(n+1)=iter; converged(n+1)=ok; line_search_hist(n+1)=lambda; residual_history{n+1}=hist; axial_constraint_error(n+1)=norm(axial.B*ui); lambda_axial_hist(n+1)=lambda_axial;
end
sim.MM=MM; sim.CC=CC; sim.KK=KK; sim.C_s=CC; sim.K_s=KK; sim.modelInfo=modelInfo; sim.q0=q0; sim.u=u; sim.yn=q_total; sim.dyn=v; sim.ddyn=a; sim.time=time; sim.F_b_hist=F_b_hist; sim.Fb_global_hist=Fb_total; sim.Fb_increment_hist=Fb_inc; sim.loaded_count_hist=loaded_count_hist; sim.slip_hist=slip_hist; sim.delta_max_hist=delta_max_hist; sim.oil_film_min_hist=oil_film_min_hist; sim.contact_stiffness_hist=contact_stiffness_hist; sim.clearance_work_hist=clearance_work_hist;
sim.solver.err_x_hist=err_u; sim.solver.err_R_hist=err_R; sim.solver.iter_hist=iter_hist; sim.solver.converged_hist=converged; sim.solver.line_search_hist=line_search_hist; sim.solver.residual_history=residual_history; sim.solver.axial_constraint_error_hist=axial_constraint_error; sim.solver.lambda_axial_hist=lambda_axial_hist; sim.solver.bearing_Kb_last=last_Kb; sim.solver.bearing_Cb_last=last_Cb; sim.solver.max_err_x=max(err_u); sim.solver.max_err_R=max(err_R); sim.solver.unconverged_steps=sum(~converged); sim.solver.max_iter=max_iter; sim.solver.tol_x=tol_u; sim.solver.tol_R=tol_R; sim.solver.dt=h; sim.solver.gamma=gamma; sim.solver.beta=beta; sim.solver.method='Full-Newton Newmark perturbation dynamics with local 5x5 bearing tangents and axial KKT constraint'; sim.params=params;
end

function [a,v]=newmark_state(u,un,vn,an,h,gamma,a0,a2,a3)
a=a0*(u-un)-a2*vn-a3*an; v=vn+h*((1-gamma)*an+gamma*a);
end

function [dFb,state]=bearing_increment(q0,u,v,Fb0,params,N)
if stage4b_value(params,'linearized_bearing',false), dFb=params.stage4B.linear_Kglobal*u+params.stage4B.linear_Cglobal*v; [~,state]=F_bearing(q0,zeros(N,1),params,N); return; end
[Fb,state]=F_bearing(q0+u,v,params,N); dFb=Fb-Fb0;
end

function [Jb,Klocal,Clocal,Kglobal,Cglobal]=local_bearing_tangent(q0,u,v,Fb0,params,N,hu0,hv0,a1)
Kglobal=sparse(N,N); Cglobal=sparse(N,N); Klocal=cell(1,numel(params.bearing)); Clocal=cell(1,numel(params.bearing));
for ib=1:numel(params.bearing)
    [B,ir]=bearing_assembly_map(params.bearing(ib),params.modelInfo,N); Kb=zeros(5); Cb=zeros(5);
    for j=1:5
        hu=max(hu0,sqrt(eps)*max(abs(u(ir(j))),1e-9)); hv=max(hv0,sqrt(eps)*max(abs(v(ir(j))),1e-6)); du=zeros(N,1); dv=zeros(N,1); du(ir(j))=hu; dv(ir(j))=hv;
        Fup=bearing_increment(q0,u+du,v,Fb0,params,N); Fum=bearing_increment(q0,u-du,v,Fb0,params,N); Fvp=bearing_increment(q0,u,v+dv,Fb0,params,N); Fvm=bearing_increment(q0,u,v-dv,Fb0,params,N);
        Kb(:,j)=(Fup(ir)-Fum(ir))/(2*hu); Cb(:,j)=(Fvp(ir)-Fvm(ir))/(2*hv);
    end
    Klocal{ib}=Kb; Clocal{ib}=Cb; Kglobal=Kglobal+B'*Kb*B; Cglobal=Cglobal+B'*Cb*B;
end
Jb=Kglobal+a1*Cglobal;
end

function [step_lambda,unew,lambda_new]=line_search(u,lambda_axial,du,dlambda,un,vn,an,h,gamma,a0,a2,a3,M,C,K,Fdyn,q0,Fb0,params,N,B,R0)
step_lambda=1;
for k=1:12
    trial=u+step_lambda*du; lambda_trial=lambda_axial+step_lambda*dlambda; [at,vt]=newmark_state(trial,un,vn,an,h,gamma,a0,a2,a3); dFb=bearing_increment(q0,trial,vt,Fb0,params,N); Rt=[M*at+C*vt+K*trial-Fdyn-dFb+B'*lambda_trial; B*trial];
    if norm(Rt)<=(1-1e-4*step_lambda)*R0, unew=trial; lambda_new=lambda_trial; return; end
    step_lambda=0.5*step_lambda;
end
unew=u+step_lambda*du; lambda_new=lambda_axial+step_lambda*dlambda;
end

function dx=safe_dynamic_kkt_solve(J,B,R)
scale=max(1,norm(J,inf)); if rcond(full(J))<1e-12, J=J+1e-10*scale*speye(size(J)); end
A=[J B'; B sparse(size(B,1),size(B,1))]; dx=A\R;
if any(~isfinite(dx)), error('Dynamic KKT Newton tangent produced a non-finite correction.'); end
end

function [B,ir]=bearing_assembly_map(b,modelInfo,N)
ir=6*b.rotor_node+(-5:-1); ic=modelInfo.num_rotor_dof+6*b.case_node+(-5:-1); B=sparse(1:5,ir,ones(5,1),5,N)+sparse(1:5,ic,-ones(5,1),5,N);
end

function axial=dynamic_axial_constraint(params,modelInfo,N)
axial=struct('B',sparse(0,N),'enabled',false); if ~isfield(params,'stage4B') || ~params.stage4B.enable, return; end
if ~isfield(params,'stage4A') || ~isfield(params.stage4A,'axial_location') || ~params.stage4A.axial_location.enabled, error('Stage4-B requires the validated Stage4-A front axial-location constraint.'); end
loc=params.stage4A.axial_location; ir=6*loc.rotor_node-3; ic=modelInfo.num_rotor_dof+6*loc.case_node-3; axial.B=sparse(1,[ir ic],[1 -1],1,N); axial.enabled=true;
end

function F=stage4b_excitation_force(t,params,N)
F=zeros(N,1); if ~isfield(params,'stage4B') || ~isfield(params.stage4B,'excitation') || isempty(params.stage4B.excitation), return; end
e=params.stage4B.excitation; amp=stage4b_value(e,'amplitude',0); node=stage4b_value(e,'node',NaN); if amp==0 || ~isfinite(node), return; end
value=amp*sin(stage4b_value(e,'omega',params.omega)*t+stage4b_value(e,'phase',0)); type=lower(stage4b_value(e,'type',''));
switch type
    case 'transverse_x', F(6*node-5)=value;
    case 'axial_z', F(6*node-3)=value;
    case 'moment_x', F(6*node-2)=value;
    otherwise, error('Unsupported Stage4-B excitation type: %s.',type);
end
end

function v=stage4b_value(s,name,default), if isfield(s,name) && ~isempty(s.(name)), v=s.(name); else, v=default; end, end

function [F,loaded,slip,dmax,hmin,kmax,cwork]=store_state(state,idx,F,loaded,slip,dmax,hmin,kmax,cwork)
for ib=1:numel(state.bearings)
    b=state.bearings(ib); F(2*ib-1:2*ib,idx)=[b.Fx;b.Fy]; loaded(ib,idx)=b.loaded_count; slip(ib,idx)=field_or_zero(b,'slip_ratio');
    dmax(ib,idx)=max_or_zero(field_or_zero(b,'delta')); hmin(ib,idx)=min_or_zero(field_or_zero(b,'h')); kmax(ib,idx)=field_or_zero(b,'k_contact_eff'); cwork(ib,idx)=field_or_zero(b,'c_work');
end
end

function ramp=startup_ramp_factor(t,omega,params)
cycles=field_or_zero(params,'startup_ramp_cycles'); if cycles<=0, ramp=1; return; end; s=min(max(t/(cycles*2*pi/omega),0),1); ramp=0.5-0.5*cos(pi*s);
end

function v=solver_value(params,name,default)
if isfield(params,'solver') && isfield(params.solver,name) && ~isempty(params.solver.(name)), v=params.solver.(name); else, v=default; end
end

function v=field_or_zero(s,name)
if isfield(s,name) && ~isempty(s.(name)), v=s.(name); else, v=0; end
end

function v=max_or_zero(x), if isempty(x), v=0; else, v=max(x,[],'all'); end, end
function v=min_or_zero(x), if isempty(x), v=0; else, v=min(x,[],'all'); end, end
