%% Plot comparison

%% Position comparison
figure()
plot(tt,sres_eul.xd(1,:),'r--','LineWidth',3); 
hold on, grid on
plot(tt,sres_eul.x(1,:),'c','LineWidth',2);
hold on, grid on
plot(tt,sres.x(2,:),'b','LineWidth',2);
legend('xd','x_eul','x_dq')

figure()
plot(tt,sres_eul.xd(2,:),'r--','LineWidth',3); 
hold on, grid on
plot(tt,sres_eul.x(2,:),'c','LineWidth',2);
hold on, grid on
plot(tt,sres.x(3,:),'b','LineWidth',2);
legend('yd','y_eul','y_dq')

figure()
plot(tt,sres_eul.xd(3,:),'r--','LineWidth',3); 
hold on, grid on
plot(tt,sres_eul.x(3,:),'c','LineWidth',2);
hold on, grid on
plot(tt,sres.x(4,:),'b','LineWidth',2);
legend('zd','z_eul','z_dq')

%% Ext force comparison
figure()
plot(tt,sres_eul.f_ext(3,:),'LineWidth',2);
hold on,grid on
plot(tt,sres.fext(3,:),'r--','LineWidth',3);
legend('fz_eul','fz_dq')
