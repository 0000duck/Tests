%% Main;
clear all;
clear classes;
warning off;
clc;

%% Addpath 
%% Addpath 

p1 = genpath('matlab_original');
p2 = genpath('functions');
p3 = genpath('Vrep_utils');
addpath(p1,p2,p3); 

disp('Loading data..')

%% Joint limits
q_min = [-2.8973   -1.7628   -2.8973   -3.0718   -2.8973   -0.0175  -2.8973];
q_max = [ 2.8973    1.7628    2.8973  -0.0698    2.8973    3.7525    2.8973];

%% Constants
C8 = DQ.C8; 
cdt = 0.01; %sampling time
time = 0:cdt:2.5; %simulation time
tt = time; 

%% Initial conditions
% q_in = [0 0 0 -1.5708 0 1.5708 0]'; %rad
q_in = [0 0 0 -1.5708  0.3491 1.5708 0]';
% q_in = [ 1.1519 0.38397 0.2618 -1.5708 0 1.3963 0]'; %rad

%% Forward kinematics
[DH, Conv] = Load_Franka_DH();
[p,R] = DirectKinematic(DH,[q_in;0],Conv);
z0 = p; % initial end-effector position
dz0 = zeros(3,1); %inital linear velocity

%% Build robot DQ kinematics
FEp_DH_theta = [0, 0, 0, 0, 0, 0, 0];
FEp_DH_d = [0.333, 0, 0.316, 0, 0.384, 0, 0.107];
FEp_DH_a = [0, 0, 0.0825, -0.0825, 0, 0.088 0.0003];
FEp_DH_alpha = [-pi/2, pi/2, pi/2, -pi/2, pi/2, pi/2 0];
FEp_DH_matrix = [FEp_DH_theta; FEp_DH_d; FEp_DH_a; FEp_DH_alpha]; 
franka_dh_matrix = FEp_DH_matrix; 
franka = DQ_SerialManipulator(FEp_DH_matrix,'standard');

x_in = franka.fkm(q_in); 
p0_in = vec4(x_in.translation);
r0_in = vec4(x_in.P);

%% euler angles
phi = atan2(R(2,1),R(1,1));
teta = atan2(-R(3,1),sqrt(R(3,2)^2 + R(3,3)^2)); 
psi = atan2(R(3,2),R(3,3));

or_in = [phi teta psi]; %initial orientation


%% Interaction task with table
I = eye(6); 
Md1 = 1.5*I;  %desired mass matrix
Kd1 = 300*I;  %desired stiffness matrix 
Bd1 = 4*sqrt(4*Kd1*Md1);   %desired damping matrix

%utils
z_table = 0.35; % m
k_table = 5000; %N/m

disp('Loading done!')
