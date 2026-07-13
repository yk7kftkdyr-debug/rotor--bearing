function sim = newmark_newton_multi(params)
%NEWMARK_NEWTON_MULTI Full-Newton nonlinear perturbation dynamics about q0.
% q_total=q0+u; M*u_ddot+C_s*u_dot+K_s*u=F_dynamic+dF_b.

if isfield(params,'case_definition') && isfield(params.case_definition,'name') && strcmp(params.case_definition.name,'stage1_static_equilibrium_ball_roller')
    validate_official_case_config(params,'stage2 full nonlinear dynamics');
end
[MM,CC,KK,modelInfo] = build_rotor_case_model(params);
params.modelInfo=modelInfo; params.num_rotor_dof=modelInfo.num_rotor_dof; params.num_case_dof=modelInfo.num_case_dof;
if ~isfield(params,'static_equilibrium_result') || ~isfield(params.static_equilibrium_result,'q0'), error('Stage 2 requires static_equilibrium_result.q0.'); end
N=size(MM,1); q0=params.static_equilibrium_result.q0; nb=numel(params.bearing);
if numel(q0)~=N, error('Static q0 length does not match the Stage-2 structural model.'); end
wi=params.omega; h=2*pi/wi/params.Fen; nt=params.Fen*params.n_Fen; time=(0:nt)*h; gamma=params.gamma; beta=params.beta;
a0=1/(beta*h^2); a1=gamma/(beta*h); a2=1/(beta*h); a3=1/(2*beta)-1;
max_iter=solver_value(params,'max_iter',20); tol_u=solver_value(params,'tol_u',1e-8); tol_R=solver_value(params,'tol_R',1e-6); u_ref=solver_value(params,'u_ref',1e-9); F_ref=solver_value(params,'F_ref',1); hu0=solver_value(params,'bearing_tangent_u',1e-8); hv0=solver_value(params,'bearing_tangent_v',1e-6);
params.current_time=0; [Fb0,state0]=F_bearing(q0,zeros(N,1),params,N);
u=zeros(N,nt+1); v=zeros(N,nt+1); a=zeros(N,nt+1); q_total=q0+u; Fb_total=zeros(N,nt+1); Fb_inc=zeros(N,nt+1); Fb_total(:,1)=Fb0;
F_b_hist=zeros(2*nb,nt+1); loaded_count_hist=zeros(nb,nt+1); slip_hist=zeros(nb,nt+1); delta_max_hist=zeros(nb,nt+1); oil_film_min_hist=zeros(nb,nt+1); contact_stiffness_hist=zeros(nb,nt+1); clearance_work_hist=zeros(nb,nt+1);
[F_b_hist,loaded_count_hist,slip_hist,delta_max_hist,oil_film_min_hist,contact_stiffness_hist,clearance_work_hist]=store_state(state0,1,F_b_hist,loaded_count_hist,slip_hist,delta_max_hist,oil_film_min_hist,contact_stiffness_hist,clearance_work_hist);
err_u=zeros(1,nt+1); err_R=zeros(1,nt+1); iter_hist=zeros(1,nt+1); converged=true(1,nt+1); line_search_hist=ones(1,nt+1); residual_history=cell(nt+1,1);
for n=1:nt
    t=time(n+1); params.current_time=t; Fdyn=startup_ramp_factor(t,wi,params)*unbalance_force(t,params,N);
    un=u(:,n); vn=v(:,n); an=a(:,n); ui=un+h*vn+h^2*(0.5-beta)*an; hist=[]; ok=false; lambda=1;
    for iter=1:max_iter
        [ai,vi]=newmark_state(ui,un,vn,an,h,gamma,a0,a2,a3); [dFb,~]=bearing_increment(q0,ui,vi,Fb0,params,N);
        R=MM*ai+CC*vi+KK*ui-Fdyn-dFb; eR=norm(R)/max(norm(Fdyn)+norm(dFb)+F_ref,F_ref); hist(end+1)=eR; %#ok<AGROW>
        if eR<=tol_R, eu=0; ok=true; break; end
        Jb=local_bearing_tangent(q0,ui,vi,Fb0,params,N,hu0,hv0,a1); J=a0*MM+a1*CC+KK-Jb;
        du=-(J\R); eu=norm(du)/max(norm(ui),u_ref);
        [lambda,ui]=line_search(ui,du,un,vn,an,h,gamma,a0,a2,a3,MM,CC,KK,Fdyn,q0,Fb0,params,N,norm(R));
    end
    [anew,vnew]=newmark_state(ui,un,vn,an,h,gamma,a0,a2,a3); [dFb,staten]=bearing_increment(q0,ui,vnew,Fb0,params,N); R=MM*anew+CC*vnew+KK*ui-Fdyn-dFb;
    eR=norm(R)/max(norm(Fdyn)+norm(dFb)+F_ref,F_ref); ok=ok || (eR<=tol_R && eu<=tol_u);
    if ~ok, warning('Full Newton did not converge at step %d: R=%.3e, du=%.3e.',n,eR,eu); end
    u(:,n+1)=ui; v(:,n+1)=vnew; a(:,n+1)=anew; q_total(:,n+1)=q0+ui; Fb_inc(:,n+1)=dFb; Fb_total(:,n+1)=Fb0+dFb;
    [F_b_hist,loaded_count_hist,slip_hist,delta_max_hist,oil_film_min_hist,contact_stiffness_hist,clearance_work_hist]=store_state(staten,n+1,F_b_hist,loaded_count_hist,slip_hist,delta_max_hist,oil_film_min_hist,contact_stiffness_hist,clearance_work_hist);
    err_u(n+1)=eu; err_R(n+1)=eR; iter_hist(n+1)=iter; converged(n+1)=ok; line_search_hist(n+1)=lambda; residual_history{n+1}=hist;
end
sim.MM=MM; sim.CC=CC; sim.KK=KK; sim.C_s=CC; sim.K_s=KK; sim.modelInfo=modelInfo; sim.q0=q0; sim.u=u; sim.yn=q_total; sim.dyn=v; sim.ddyn=a; sim.time=time; sim.F_b_hist=F_b_hist; sim.Fb_global_hist=Fb_total; sim.Fb_increment_hist=Fb_inc; sim.loaded_count_hist=loaded_count_hist; sim.slip_hist=slip_hist; sim.delta_max_hist=delta_max_hist; sim.oil_film_min_hist=oil_film_min_hist; sim.contact_stiffness_hist=contact_stiffness_hist; sim.clearance_work_hist=clearance_work_hist;
sim.solver.err_x_hist=err_u; sim.solver.err_R_hist=err_R; sim.solver.iter_hist=iter_hist; sim.solver.converged_hist=converged; sim.solver.line_search_hist=line_search_hist; sim.solver.residual_history=residual_history; sim.solver.max_err_x=max(err_u); sim.solver.max_err_R=max(err_R); sim.solver.unconverged_steps=sum(~converged); sim.solver.max_iter=max_iter; sim.solver.tol_x=tol_u; sim.solver.tol_R=tol_R; sim.solver.dt=h; sim.solver.gamma=gamma; sim.solver.beta=beta; sim.solver.method='Full-Newton Newmark perturbation dynamics with local nonlinear bearing tangents'; sim.params=params;
end

function [a,v]=newmark_state(u,un,vn,an,h,gamma,a0,a2,a3)
a=a0*(u-un)-a2*vn-a3*an; v=vn+h*((1-gamma)*an+gamma*a);
end

function [dFb,state]=bearing_increment(q0,u,v,Fb0,params,N)
[Fb,state]=F_bearing(q0+u,v,params,N); dFb=Fb-Fb0;
end

function Jb=local_bearing_tangent(q0,u,v,Fb0,params,N,hu0,hv0,a1)
Jb=zeros(N); dofs=[]; nr=params.modelInfo.num_rotor_dof;
for ib=1:numel(params.bearing)
    b=params.bearing(ib); dofs=[dofs,4*b.rotor_node+(-3:-2),nr+4*b.case_node+(-3:-2)]; %#ok<AGROW>
end
for j=unique(dofs)
    hu=max(hu0,sqrt(eps)*max(abs(u(j)),1e-9)); hv=max(hv0,sqrt(eps)*max(abs(v(j)),1e-6)); du=zeros(N,1); dv=zeros(N,1); du(j)=hu; dv(j)=hv;
    Fup=bearing_increment(q0,u+du,v,Fb0,params,N); Fum=bearing_increment(q0,u-du,v,Fb0,params,N); Fvp=bearing_increment(q0,u,v+dv,Fb0,params,N); Fvm=bearing_increment(q0,u,v-dv,Fb0,params,N);
    Jb(:,j)=(Fup-Fum)/(2*hu)+a1*(Fvp-Fvm)/(2*hv);
end
end

function [lambda,unew]=line_search(u,du,un,vn,an,h,gamma,a0,a2,a3,M,C,K,Fdyn,q0,Fb0,params,N,R0)
lambda=1;
for k=1:12
    trial=u+lambda*du; [at,vt]=newmark_state(trial,un,vn,an,h,gamma,a0,a2,a3); dFb=bearing_increment(q0,trial,vt,Fb0,params,N); Rt=M*at+C*vt+K*trial-Fdyn-dFb;
    if norm(Rt)<=(1-1e-4*lambda)*R0, unew=trial; return; end
    lambda=0.5*lambda;
end
unew=u+lambda*du;
end

function [F,loaded,slip,dmax,hmin,kmax,cwork]=store_state(state,idx,F,loaded,slip,dmax,hmin,kmax,cwork)
for ib=1:numel(state.bearings)
    b=state.bearings(ib); F(2*ib-1:2*ib,idx)=[b.Fx;b.Fy]; loaded(ib,idx)=b.loaded_count; slip(ib,idx)=b.slip_ratio;
    dmax(ib,idx)=max_or_zero(b.delta); hmin(ib,idx)=min_or_zero(b.h); kmax(ib,idx)=field_or_zero(b,'k_contact_eff'); cwork(ib,idx)=field_or_zero(b,'c_work');
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

function v=max_or_zero(x), if isempty(x), v=0; else, v=max(x); end, end
function v=min_or_zero(x), if isempty(x), v=0; else, v=min(x); end, end
