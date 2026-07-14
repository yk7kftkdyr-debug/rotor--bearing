function [MM, CC, KK, modelInfo] = build_rotor_case_model(params)
%BUILD_ROTOR_CASE_MODEL 6-DOF/node rotor-case beam model: [ux uy uz thx thy thz].
if isfield(params,'use_uploaded_rotor_model') && params.use_uploaded_rotor_model, warning('Uploaded legacy 4-DOF model is disabled for Stage 3; using internal 6-DOF builder.'); end
xR=params.node_pos(:).'; xC=params.case_node_pos(:).'; nR=numel(xR); nC=numel(xC); ndR=6*nR; ndC=6*nC;
A=pi/4*(params.shaft_od^2-params.shaft_id^2); I=pi/64*(params.shaft_od^4-params.shaft_id^4); J=pi/32*(params.shaft_od^4-params.shaft_id^4);
[MR,KR]=assemble_beam6(xR,params.E,params.G,I,J,A,getv(params,'shaft_mass_rho',params.rho));
Ac=pi/4*(params.case_od^2-params.case_id^2); Ic=pi/64*(params.case_od^4-params.case_id^4); Jc=pi/32*(params.case_od^4-params.case_id^4);
[MC,KC]=assemble_beam6(xC,params.case_E,getv(params,'case_G',params.case_E/(2*(1-getv(params,'case_nu',0.3)))),Ic,Jc,Ac,getv(params,'case_mass_rho',params.case_rho));
if isfield(params,'concentrated_mass'), cm=params.concentrated_mass; if ~isfield(cm,'inertia_kgm2') || ~isequal(size(cm.inertia_kgm2),[numel(cm.nodes) 3]), error('concentrated_mass.inertia_kgm2 must be N-by-3 in [Ix Iy Iz] order.'); end; for k=1:numel(cm.nodes), MR=add_rigid_mass6(MR,cm.nodes(k),cm.kg(k),cm.inertia_kgm2(k,:)); end, end
if isfield(params,'disk_mass'), MR=add_rigid_mass6(MR,params.disk_mass.node,params.disk_mass.kg,getv(params.disk_mass,'inertia_kgm2',[0 0 0])); end
rot_ref=getv(params,'legacy_rotor_bending_reference_k',1e2); for i=1:nR, d=6*i+(-5:0); KR(d(4),d(4))=KR(d(4),d(4))+rot_ref; KR(d(5),d(5))=KR(d(5),d(5))+rot_ref; end
ground=getv(params,'case_ground_nodes',[1 nC]); kax=getv(params,'case_ground_axial_k',1e8); ktwist=getv(params,'case_ground_torsion_k',1e6); krot=getv(params,'case_ground_bending_rot_k',1e6); cax=getv(params,'case_ground_axial_c',100); ctwist=getv(params,'case_ground_torsion_c',10); crot=getv(params,'case_ground_bending_rot_c',10);
if getv(params,'stage3_engineering_initials_pending_calibration',false) || ~isfield(params,'case_ground_axial_k') || ~isfield(params,'case_ground_torsion_k'), warning('Stage 3 foundation axial/torsional values are configurable engineering initial values pending calibration.'); end
for i=ground
    d=6*i+(-5:0); KC(d(1),d(1))=KC(d(1),d(1))+params.case_ground_k; KC(d(2),d(2))=KC(d(2),d(2))+params.case_ground_k; KC(d(3),d(3))=KC(d(3),d(3))+kax; KC(d(4),d(4))=KC(d(4),d(4))+krot; KC(d(5),d(5))=KC(d(5),d(5))+krot; KC(d(6),d(6))=KC(d(6),d(6))+ktwist;
end
MM=blkdiag(MR,MC); KK=blkdiag(KR,KC); CC=params.rayleigh_alpha*MM+params.rayleigh_beta*KK;
for i=ground
    d=ndR+6*i+(-5:0); CC(d(1),d(1))=CC(d(1),d(1))+params.case_ground_c; CC(d(2),d(2))=CC(d(2),d(2))+params.case_ground_c; CC(d(3),d(3))=CC(d(3),d(3))+cax; CC(d(4),d(4))=CC(d(4),d(4))+crot; CC(d(5),d(5))=CC(d(5),d(5))+crot; CC(d(6),d(6))=CC(d(6),d(6))+ctwist;
end
GG=zeros(ndR+ndC); if isfield(params,'disk_mass') && isfield(params,'omega') && params.disk_mass.node>=1 && params.disk_mass.node<=nR, Id=getv(params.disk_mass,'inertia_kgm2',[0 0 0]); d=6*params.disk_mass.node+(-5:0); GG(d(4),d(5))=-Id(3); GG(d(5),d(4))=Id(3); CC=CC+params.omega*GG; end
modelInfo.num_rotor_nodes=nR; modelInfo.num_case_nodes=nC; modelInfo.num_rotor_dof=ndR; modelInfo.num_case_dof=ndC; modelInfo.dof_per_node=6; modelInfo.dof_order={'ux','uy','uz','theta_x','theta_y','theta_z'}; modelInfo.rotor_trans_dof=reshape([1:6:ndR;2:6:ndR;3:6:ndR],[],1); modelInfo.case_trans_dof=ndR+reshape([1:6:ndC;2:6:ndC;3:6:ndC],[],1); modelInfo.source='internal 6-DOF Euler-Bernoulli beam model'; modelInfo.bearing_model_stage='2D_force_on_6DOF_structure'; modelInfo.section.A=A; modelInfo.section.I=I; modelInfo.section.J=J; modelInfo.GG=GG; modelInfo.mass_components=getv(params,'mass_target',struct());
end

function [M,K]=assemble_beam6(x,E,G,I,J,A,rho)
n=numel(x); M=zeros(6*n); K=zeros(6*n);
for e=1:n-1
 L=x(e+1)-x(e); if L<=0,error('Node positions must be strictly increasing.');end
 [mb,kb]=beam4(E,I,rho,A,L); [ma,ka]=rod2(E,A,rho,L); [mt,kt]=rod2(G,J,rho,L); d1=6*e+(-5:0); d2=6*(e+1)+(-5:0);
 ix=[d1(1) d1(5) d2(1) d2(5)]; iy=[d1(2) d1(4) d2(2) d2(4)]; iz=[d1(3) d2(3)]; it=[d1(6) d2(6)]; M(ix,ix)=M(ix,ix)+mb; K(ix,ix)=K(ix,ix)+kb; M(iy,iy)=M(iy,iy)+mb; K(iy,iy)=K(iy,iy)+kb; M(iz,iz)=M(iz,iz)+ma; K(iz,iz)=K(iz,iz)+ka; M(it,it)=M(it,it)+mt; K(it,it)=K(it,it)+kt;
end
end

function [m,k]=beam4(E,I,rho,A,L)
k=E*I/L^3*[12 6*L -12 6*L;6*L 4*L^2 -6*L 2*L^2;-12 -6*L 12 -6*L;6*L 2*L^2 -6*L 4*L^2]; m=rho*A*L/420*[156 22*L 54 -13*L;22*L 4*L^2 13*L -3*L^2;54 13*L 156 -22*L;-13*L -3*L^2 -22*L 4*L^2];
end
function [m,k]=rod2(EAorGJ,areaOrJ,rho,L)
k=EAorGJ*areaOrJ/L*[1 -1;-1 1]; m=rho*areaOrJ*L/6*[2 1;1 2];
end
function M=add_rigid_mass6(M,node,mass,inertia)
if mass<0,error('Mass must be nonnegative.');end; d=6*node+(-5:0); M(d(1:3),d(1:3))=M(d(1:3),d(1:3))+mass*eye(3); M(d(4:6),d(4:6))=M(d(4:6),d(4:6))+diag(inertia(:));
end
function v=getv(s,name,default), if isfield(s,name)&&~isempty(s.(name)),v=s.(name);else,v=default;end,end
