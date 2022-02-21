%% Interaction task
%% Desired trajectory
function [xd,dxd,ddxd,or_data] = int_traj(x_in,r0,time)

%% Description: generates minimum jerk task-space trajectory for interaction task with environment.
%%Outputs: xd,dxd,ddxd = desired task-space trajectory; [3x1]
%          or_data = desired orientation of end effector expressed with ZYX euler angles (fixed for now) [3x1]         [3x1]

%%Inputs:  x_in = initial EE position; [3x1]
%          r0 = initial EE rotation; [3x1]
%          time = simulation time.

%% Initialize variables
xd = [zeros(size(time,2),3)];
dxd = [zeros(size(time,2),3)];
ddxd = [zeros(size(time,2),3)];
or_data = [zeros(size(time,2),3)];

%% Retrive initial conditions
pos_i = [x_in(1);x_in(2);x_in(3)];
i = 1;  

for i = 1:size(time,2)
    if (time(i) >=0 && time(i) < 1) %go down
        pos_f = pos_i + [0;0;-0.3];
        tf = 1;
        t = time(i);
    elseif (time(i) >=1 && time(i) < 1.3) %pause
        pos_i = [x_in(1);x_in(2);x_in(3)-0.3];
        pos_f = pos_i;
        tf = 0.3;
        t = time(i) - 1;
    elseif (time(i) >= 1.3 && time(i) < 2.3) %go up
        pos_i = [x_in(1);x_in(2);x_in(3)-0.3];
        pos_f = pos_i + [0;0;0.3];
        tf = 1;
        t = time(i) - 1.3;
    else
        pos_i = [x_in(1);x_in(2);x_in(3)];
        pos_f = pos_i;
        tf = 1000;
        t = time(i) - 2.3;
    end

    %% Minimum jerk interpolation
    zd = pos_i + (pos_i - pos_f)*(15*(t/tf)^4 - 6*(t/tf)^5 -10*(t/tf)^3);
    dzd = (pos_i - pos_f)*(60*(t^3)/(tf^4) - 30*(t^4)/(tf^5) -30*(t^2)/(tf^3));
    ddzd = (pos_i - pos_f)*(180*(t^2)/(tf^4) - 120*(t^3)/(tf^5) -60*(t)/(tf^3));
    
    %Compute desired trajectory
    xd(i,:) = zd;
    dxd(i,:) = dzd;
    ddxd(i,:) = ddzd;
    or_data(i,:) = r0;
    i = i+1;
    
end

end



