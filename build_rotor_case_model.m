function [MM, CC, KK, modelInfo] = build_rotor_case_model(params)
%BUILD_ROTOR_CASE_MODEL Document-based segmented 6-DOF rotor-casing model.
% DOF order at every node is [ux uy uz theta_x theta_y theta_z].
if isfield(params,'use_uploaded_rotor_model') && params.use_uploaded_rotor_model, warning('Uploaded legacy 4-DOF model is disabled for Stage 1; using the document-based segmented builder.'); end
required = {'rotor_elements','case_elements','node_pos','case_node_pos'};
for k=1:numel(required), if ~isfield(params,required{k}), error('Stage 1 requires params.%s.',required{k}); end, end
xR=params.node_pos(:).'; xC=params.case_node_pos(:).'; nR=numel(xR); nC=numel(xC); ndR=6*nR; ndC=6*nC;
[Ar,Ir,Jr,Lr,sr]=section_properties(params.rotor_elements); [Ac,Ic,Jc,Lc,sc]=section_properties(params.case_elements);
if nR~=19 || numel(Ar)~=18 || nC~=13 || numel(Ac)~=12, error('Stage 1 requires 18/19 rotor and 12/13 casing segmentation.'); end
[MR,KR]=assemble_beam6(xR,params.E,params.G,Ir,Jr,Ar,params.rho,sr);
[MC,KC]=assemble_beam6(xC,params.case_E,params.case_G,Ic,Jc,Ac,params.case_rho,sc);
cm=params.concentrated_mass; if ~isequal(size(cm.inertia_kgm2),[numel(cm.nodes) 3]), error('concentrated_mass.inertia_kgm2 must be N-by-3 in [Ix Iy Iz] order.'); end
for k=1:numel(cm.nodes), MR=add_rigid_mass6(MR,cm.nodes(k),cm.kg(k),cm.inertia_kgm2(k,:)); end
MR=add_rigid_mass6(MR,params.disk_mass.node,params.disk_mass.kg,params.disk_mass.inertia_kgm2);
cbm=params.case_bearing_mass; for k=1:numel(cbm.nodes), MC=add_translational_mass6(MC,cbm.nodes(k),cbm.kg(k)); end
ground=params.case_ground_nodes;
for i=ground, d=6*i+(-5:0); KC(d(1),d(1))=KC(d(1),d(1))+params.case_ground_k; KC(d(2),d(2))=KC(d(2),d(2))+params.case_ground_k; end
MM=blkdiag(MR,MC); KK=blkdiag(KR,KC);
C_foundation=zeros(ndR+ndC);
for i=ground, d=ndR+6*i+(-5:0); C_foundation(d(1),d(1))=params.case_ground_c; C_foundation(d(2),d(2))=params.case_ground_c; end
C_rayleigh_current=params.rayleigh_alpha*MM+params.rayleigh_beta*KK; % Stage 8 pending recalibration.
GG=zeros(ndR+ndC);
for k=1:numel(cm.nodes), GG=add_gyro(GG,cm.nodes(k),cm.inertia_kgm2(k,3)); end
GG=add_gyro(GG,params.disk_mass.node,params.disk_mass.inertia_kgm2(3));
CC=C_rayleigh_current+C_foundation+params.omega*GG;
rotor_distributed_mass=sum(params.rho.*Ar.*Lr.*sr); case_distributed_mass=sum(params.case_rho.*Ac.*Lc.*sc); rotor_lumped_mass=sum(cm.kg)+params.disk_mass.kg; case_lumped_mass=sum(cbm.kg);
mass_components=struct('shaft_N',rotor_distributed_mass*9.80665,'concentrated_N',sum(cm.kg)*9.80665,'disk_N',params.disk_mass.kg*9.80665,'case_N',(case_distributed_mass+case_lumped_mass)*9.80665);
mass_mapping=struct('rotor_distributed_mass_kg',rotor_distributed_mass,'rotor_concentrated_mass_kg',sum(cm.kg),'disk_mass_kg',params.disk_mass.kg,'case_distributed_mass_kg',case_distributed_mass,'case_bearing_mass_kg',case_lumped_mass);
modelInfo=struct('num_rotor_nodes',nR,'num_case_nodes',nC,'num_rotor_dof',ndR,'num_case_dof',ndC,'dof_per_node',6,'dof_order',{{'ux','uy','uz','theta_x','theta_y','theta_z'}},'rotor_trans_dof',reshape([1:6:ndR;2:6:ndR;3:6:ndR],[],1),'case_trans_dof',ndR+reshape([1:6:ndC;2:6:ndC;3:6:ndC],[],1),'source','document-based segmented 6DOF rotor-casing beam model','bearing_model_stage','2D_force_on_6DOF_structure','section',struct('A',Ar,'I',Ir,'J',Jr),'num_rotor_elements',18,'num_case_elements',12,'rotor_node_position_m',xR(:),'case_node_position_m',xC(:),'rotor_element_length_m',Lr,'rotor_inner_diameter_m',params.rotor_elements.inner_diameter_m(:),'rotor_outer_diameter_m',params.rotor_elements.outer_diameter_m(:),'rotor_area_m2',Ar,'rotor_second_moment_m4',Ir,'rotor_polar_moment_m4',Jr,'rotor_mass_scale',sr,'case_element_length_m',Lc,'case_inner_diameter_m',params.case_elements.inner_diameter_m(:),'case_outer_diameter_m',params.case_elements.outer_diameter_m(:),'case_area_m2',Ac,'case_second_moment_m4',Ic,'case_polar_moment_m4',Jc,'rotor_distributed_mass_kg',rotor_distributed_mass,'case_distributed_mass_kg',case_distributed_mass,'rotor_lumped_mass_kg',rotor_lumped_mass,'case_lumped_mass_kg',case_lumped_mass,'total_rotor_mass_kg',rotor_distributed_mass+rotor_lumped_mass,'total_case_mass_kg',case_distributed_mass+case_lumped_mass,'mass_mapping',mass_mapping,'GG',GG,'C_foundation',C_foundation,'C_rayleigh_current',C_rayleigh_current,'pending_reference_constraints',true,'mass_components',mass_components);
end

function [M,K]=assemble_beam6(x,E,G,I,J,A,rho,mass_scale)
n=numel(x); M=zeros(6*n); K=zeros(6*n);
for e=1:n-1
    L=x(e+1)-x(e); if L<=0,error('Node positions must be strictly increasing.');end
    [mb,kb]=beam4(E,I(e),rho*mass_scale(e),A(e),L); [ma,ka]=rod2(E,A(e),rho*mass_scale(e),L); [mt,kt]=rod2(G,J(e),rho*mass_scale(e),L); d1=6*e+(-5:0); d2=6*(e+1)+(-5:0);
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
function M=add_translational_mass6(M,node,mass)
if mass<0,error('Mass must be nonnegative.');end; d=6*node+(-5:-3); M(d,d)=M(d,d)+mass*eye(3);
end
function GG=add_gyro(GG,node,Iz)
d=6*node+(-5:0); GG(d(4),d(5))=GG(d(4),d(5))-Iz; GG(d(5),d(4))=GG(d(5),d(4))+Iz;
end
function [A,I,J,L,mass_scale]=section_properties(elements)
L=elements.length_m(:); Di=elements.inner_diameter_m(:); Do=elements.outer_diameter_m(:); mass_scale=elements.mass_scale(:);
if any(~isfinite([L;Di;Do;mass_scale])) || any(L<=0) || any(Di<0) || any(Do<=Di) || numel(mass_scale)~=numel(L), error('Invalid segmented beam geometry or mass scale.'); end
A=pi/4*(Do.^2-Di.^2); I=pi/64*(Do.^4-Di.^4); J=pi/32*(Do.^4-Di.^4);
end
