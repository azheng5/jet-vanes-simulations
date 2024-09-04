% Initialize simulation
clear
clc
close all

% Define initial state
x0 = [0;
      0;
      0;
      0;
      0;
      0;
      0;
      0;
      0;
      0;
      0;
      0];

% Define initial control input
u = [0;
     0;
     0;
     0];

SIM_TIME = 60; % simulink sim time

model = 'jetVane6DOFSimulation.slx';
out = sim(model); % run simulink model

% Plot results
t = out.simX.Time;

x1 = out.simX.Data(:,1);
x2 = out.simX.Data(:,2);
x3 = out.simX.Data(:,3);
x4 = out.simX.Data(:,4);
x5 = out.simX.Data(:,5);
x6 = out.simX.Data(:,6);
x7 = out.simX.Data(:,7);
x8 = out.simX.Data(:,8);
x9 = out.simX.Data(:,9);
x10 = out.simX.Data(:,10);
x11 = out.simX.Data(:,11);
x12 = out.simX.Data(:,12);

u1 = out.simU.Data(:,1);
u2 = out.simU.Data(:,2);
u3 = out.simU.Data(:,3);
u4 = out.simU.Data(:,4);

figure
subplot(1,3,1)
plot(t,x10)
legend('x10')
grid on

subplot(1,3,2)
plot(t,x11)
legend('x11')
grid on

subplot(1,3,3)
plot(t,x12)
legend('x12')
grid on

% %Plot control inputs
% figure
% 
% subplot(4,1,1)
% plot(t,u1)
% legend('u_1');
% grid on
% 
% subplot(4,1,2)
% plot(t,u2)
% legend('u_2');
% grid on
% 
% subplot(4,1,3)
% plot(t,u3)
% legend('u_3');
% grid on
% 
% subplot(4,1,4)
% plot(t,u4)
% legend('u_4');
% grid on
% 
% % Plot states
% figure
% 
% % uvw
% subplot(3,4,1)
% plot(t,x1)
% legend('x_1');
% grid on
% 
% subplot(3,4,4)
% plot(t,x2)
% legend('x_2');
% grid on
% 
% subplot(3,4,7)
% plot(t,x3)
% legend('x_3');
% grid on
% 
% % pqr
% subplot(3,4,2)
% plot(t,x4)
% legend('x_4');
% grid on
% 
% subplot(3,4,5)
% plot(t,x5)
% legend('x_5');
% grid on
% 
% subplot(3,4,8)
% plot(t,x6)
% legend('x_6');
% grid on
% 
% %pitch roll yaw
% subplot(3,4,3)
% plot(t,x7)
% legend('x_7');
% grid on
% 
% subplot(3,4,6)
% plot(t,x8)
% legend('x_8');
% grid on
% 
% subplot(3,4,9)
% plot(t,x9)
% legend('x_9');
% grid on