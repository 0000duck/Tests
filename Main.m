%% Main;
clear all;
clear classes;
warning off;
clc;

%% Define demo simulation
%fuse = 1; %free motion tracking task --> franka_scene.ttt
fuse = 2;  %interaction task ---> franka_int.ttt

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
if fuse == 1
    q_in = [ 1.1519 0.38397 0.2618 -1.5708 0 1.3963 0]'; %rad freee motion
else
    q_in = [0 0 0 -1.5708  0.3491 1.5708 0]'; %rad int task
end

%% Forward kinematics
[DH, Conv] = Load_Franka_DH();
[p,R] = DirectKinematic(DH,[q_in;0],Conv);
z0 = p; % initial end-effector position
dz0 = zeros(3,1); %inital linear velocity

%% euler angles (ZYX parametrization)
phi = atan2(R(2,1),R(1,1));
teta = atan2(-R(3,1),sqrt(R(3,2)^2 + R(3,3)^2)); 
psi = atan2(R(3,2),R(3,3));

or_in = [phi teta psi]; %initial orientation


%% Impedance values
I = eye(6); 
Md1 = 1.5*I;  %desired mass matrix
Kd1 = 300*I;  %desired stiffness matrix 
Bd1 = 4*sqrt(4*Kd1*Md1);   %desired damping matrix

%utils int_task 
z_table = 0.35; % m
k_table = 5000; %N/m

disp('Loading done!')
