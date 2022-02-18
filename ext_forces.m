function wrench_ext = ext_forces(x)
%% Description: model external forces acting on the end effectors
%% Assumptions: elastic reaction of the environment; negligible external torques
% Output: wrench_ext = vector 6x1 representing the external wrench on EE (world frame)
% Inputs: x = current EE pose;


k_table = 5000; %N/m, environment stiffness
pc = 0.35; %contact position (z axis)

z = [x(1); x(2); x(3)];

if z(3) < pc
    F_ext = -k_table*(z(3) - pc); %elastic reaction
else
    F_ext = 0;

end
wrench_ext = [0;0;F_ext;0;0;0];

end