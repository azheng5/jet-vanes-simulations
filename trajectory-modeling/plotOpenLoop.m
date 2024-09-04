%% %%%%%%%%%%%%%%%% Plotting %%%%%%%%%%%%%%%

figure(1)
plot3(0,0,0,'o','MarkerSize',10)
hold on
plot3(traj.x(end,2),traj.x(end,3),traj.x(end,1),'x','MarkerSize',10)
hold on
plot3(traj.x(:,2),traj.x(:,3),traj.x(:,1))
grid on
xlabel('x')
ylabel('y')
zlabel('z')
title('Open Loop Trajectory')

%%
figure(2)
plot(rkt_time_span,impulse)
legend('u1','u2','u3','u4')

%%
figure(3)
plot(traj.t,traj.alpha)

%%
figure(4)
plot(traj.t,traj.velBody)
legend('x','y','z')

%%
figure(5)
plot(traj.t,traj.angVel)
legend('x','y','z')

%%
figure(6)
plot(traj.t,traj.qWorld)
legend('q0','q1','q2','q3')

%%
figure(7)
plot(traj.t,traj.posWorld)
legend('x','y','z')

%%
figure(5)
plot(traj.t,traj.angVelDot)
legend('x','y','z')

%%
figure(6)
plot(traj.t,traj.M_ps)

%%
figure(7)
plot(traj.t,traj.MT_b)
legend('x','y','z')

%%
figure(8)
eul = quat2eul(traj.qWorld)
plot(traj,t,eul(3:-1:1))
legend('x','y','z')