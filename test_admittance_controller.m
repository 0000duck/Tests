%% TEST ADMITTANCE 

%des trajectory
% [xd,dxd,ddxd,or_data] = gen_traj(z0,or_in,time);
[xd,dxd,ddxd,or_data] = int_traj(z0,or_in,time); 

wrench_ext_data = zeros(size(time,2),6); 

f_ext = [0;0;-10;0;0;0]; % to be expressed with respect to the compliant frame

%j = 1;
%for j = 1:size(time,2)
%   wrench_ext_data(j,:) = [0;0;-10;0;0;0];
%end
%
%figure()
%plot(time,wrench_ext_data(:,3)); 

xc_data = zeros(size(time,2),3);
dxc_data = zeros(size(time,2),3);
ddxc_data = zeros(size(time,2),3);
e_data = zeros(size(time,2),6); 
de_data = zeros(size(time,2),6); 


j = 1;
for j = 1:size(time,2)
    if j~=1
        xr = xc_data(j-1,:)';
        or = or_data(j-1,:)';
        e = e_data(j-1,:)'; 
        de = de_data(j-1,:)';
    else
        xr = z0;
        or = or_in; 
        e = [xd(1,:)' - xr; zeros(3,1)]; 
        de = zeros(6,1); 
    end

    %compliant traj
    [xc,dxc,ddxc,or,e,de] = adm_control(xd(j,:)',dxd(j,:)',ddxd(j,:)',or_data(j,:)',e,de,or,f_ext,Md1,Kd1,Bd1,time); 

    xc_data(j,:) = xc; 
    dxc_data(j,:) = dxc;
    ddxc_data(j,:) = ddxc;
    or_data(j,:) = or; 
    e_data(j,:) = e; 
    de_data(j,:) = de; 

    j = j+1;
end


% %% PLOTS

figure()
tiledlayout(3,1) 
nexttile
plot(time, xd(:,1), 'Linewidth',2, 'Color', '[0.9290, 0.6940, 0.1250]')
hold on 
grid on
plot(time, xc_data(:,1), 'Linewidth',1.5, 'Color', 'b','LineStyle','--')
hold on
grid on
xlabel('time [s]')
ylabel('x [m]')
legend('des','comp')
nexttile
plot(time, xd(:,2), 'Linewidth',2, 'Color', '[0.9290, 0.6940, 0.1250]')
hold on 
grid on
plot(time, xc_data(:,2), 'Linewidth',1.5, 'Color', 'b','LineStyle','--')
hold on
grid on
xlabel('time [s]')
ylabel('y [m]')
legend('des','comp')

nexttile
plot(time, xd(:,3), 'Linewidth',2, 'Color', '[0.9290, 0.6940, 0.1250]')
hold on 
grid on
plot(time, xc_data(:,3), 'Linewidth',1.5, 'Color', 'b','LineStyle','--')
hold on
grid on
xlabel('time [s]')
ylabel('z [m]')
legend('des','comp')


figure;
plot(time, xd(:,3) - xc_data(:,3), 'Linewidth',2, 'Color', '[0.9290, 0.6940, 0.1250]')
xlabel('time [s]')
ylabel('displacement [m]')

