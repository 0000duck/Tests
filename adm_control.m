%% Admittance controller
%% Description: admittance controller to enforce an apparent impedance behaviour at task-space

%%inputs: xd,dxd,ddxd = desired reference trajectory
%         or_des = desired rotation
%         f_ext = external wrench on EE (with respect to compliant reference frame)
%         time = simulation time
%         Md,Kd,Bd = impedance matrices
%%outpus: xc,dxc,ddxc = compliant trajectory to enforce desired impedance behaviour between the frames
%         or_c = compliant orientation (fixed for now)

function [xc,dxc,ddxc,or,e,de] = adm_control(xd,dxd,ddxd,or_des,e,de,wrench_ext,Md,Kd,Bd,time)

cdt = time(2) - time(1); %sampling time 

%% Define position and orientation errors

eul = [or_des(1) or_des(2) or_des(3)]; %euler angles
Rd = eul2rotm(eul); %rotation matrix (ZYX)

err = e;
e_dot = de;  

%compute admittance equation
dde = inv(Md)*(-Bd*e_dot - Kd*err + wrench_ext); %admittance equation
de  = dde*cdt + de;
e = de*cdt + e;

ddxc = ddxd - dde(1:3,:);
dxc = dxd - de(1:3,:);
xc = xd - e(1:3,:);
or = or_des - e(4:6,:); 

end

