
function [xd,dxd,ddxd,rot] = gen_traj(x_in,r0,time)

%% Description: generates minimum jerk task-space trajectory for the end-effector
%%Outputs: xd,dxd,ddxd = desired position,velocity,acceleration [3x1]
%          or_data = desired orientation (ZYX euler angles)[3x1]

%%Inputs: x_in = initial EE position [3x1]
%         r0 = initial EE rotation [3x1]
%         time = simulation time


%%initialize
xd = [zeros(size(time,2),3)];
dxd = [zeros(size(time,2),3)];
ddxd = [zeros(size(time,2),3)];
rot = [zeros(size(time,2),3)];

%%retrieve initial position

pos_i = [x_in(1);x_in(2);x_in(3)];
i = 1;  

for i = 1:size(time,2)
    if (time(i)>=0 && time(i)<0.5) %go down
        pos_f = pos_i + [0;0;-0.3];
        tf = 0.5;
        t = time(i);
    elseif (time(i)>=0.5 && time(i)<0.6) %pause
        pos_i = [x_in(1);x_in(2);x_in(3)-0.3];
        pos_f = pos_i;
        tf = 0.1;
        t = time(i) - 0.5;
    elseif (time(i)>=0.6 && time(i)<1.1) %go +x
        pos_i = [x_in(1);x_in(2);x_in(3)-0.3];
        pos_f = pos_i + [0.2;0;0];
        tf = 0.5;
        t = time(i) - 0.6;
    elseif (time(i)>=1.1 && time(i)<1.2) %pause
        pos_i = [x_in(1)+0.2;x_in(2);x_in(3)-0.3];
        pos_f = pos_i;
        tf = 0.1;
        t = time(i) - 1.1;
    elseif (time(i)>=1.2 && time(i)<1.7) %go up and backwards
        pos_i = [x_in(1)+0.2;x_in(2);x_in(3)-0.3];
        pos_f = pos_i + [-0.2;0;+0.3];
        tf = 0.5;
        t = time(i)-1.2;
    else
        pos_i = [x_in(1);x_in(2);x_in(3)];
        pos_f = pos_i;
        tf = 1000;
        t = time(i) - 1.7;
    end
    %% Minimum jerk interpolation
    zd = pos_i + (pos_i - pos_f)*(15*(t/tf)^4 - 6*(t/tf)^5 -10*(t/tf)^3);
    dzd = (pos_i - pos_f)*(60*(t^3)/(tf^4) - 30*(t^4)/(tf^5) -30*(t^2)/(tf^3));
    ddzd = (pos_i - pos_f)*(180*(t^2)/(tf^4) - 120*(t^3)/(tf^5) -60*(t)/(tf^3));

    
    xd(i,:) = zd;
    dxd(i,:) = dzd;
    ddxd(i,:) = ddzd;
    rot(i,:) = r0;
    i = i+1;

end

end



