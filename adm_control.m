%% Admittance controller
%% Description: admittance controller to enforce an apparent impedance behaviour at task-space

%%Inputs: xd,dxd,ddxd = desired reference trajectory; [3x1]
%         or_des = desired rotation (ZYX euler angles); [3x1]
%         e = previous error; [6x1]
%         de = previous computed position error; [6x1]
%         wrench_ext = external wrench on EE (with respect to compliant
%         reference frame) [6x1];
%         time = simulation time
%         Md,Kd,Bd = impedance matrices [6x6].

%%Outpus: xc,dxc,ddxc = compliant trajectory to enforce desired impedance behaviour between the frames
%         or_c = compliant orientation (fixed for now)

function [xc,dxc,ddxc,or,e,de] = adm_control(xd,dxd,ddxd,or_des,e,de,wrench_ext,Md,Kd,Bd,time)

cdt = time(2) - time(1); %sampling time 

%% Define position and orientation errors

eul = [or_des(1) or_des(2) or_des(3)]; %euler angles [ZYX]

err = e;
e_dot = de;  

%% Compute admittance equation
dde = inv(Md)*(-Bd*e_dot - Kd*err + wrench_ext); %admittance equation
de  = dde*cdt + de;
e = de*cdt + e;

ddxc = ddxd - dde(1:3,:);
dxc = dxd - de(1:3,:);
xc = xd - e(1:3,:);
or = or_des - e(4:6,:); 

end

