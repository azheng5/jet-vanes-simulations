%%
figure(1)

trueEul = quat2eul(log.true.q_bf);
plot(log.true.time, trueEul(:,3:-1:1))
title('True Attitude - Euler Angles')
xlabel('Time (s)')
ylabel('Attitude (rad)')
legend('X','Y','Z')
grid on

%%
figure(2)

% define q0 as IC of true q0 (unchanging)
eul = quat2eul(log.linearized.q_bf);
plot(log.linearized.time, eul(:,3:-1:1))
title('Linearized Attitude - Euler Angles')
xlabel('Time (s)')
ylabel('Attitude (rad)')
legend('X','Y','Z')
grid on

%%
figure(3)

plot(log.true.time,log.true.q_bf)
title('True Attitude - Quaternion')
xlabel('Time (s)')
ylabel('Attitude')
grid on
legend('q0','q1','q2','q3')

%%
figure(4)

plot(log.linearized.time,log.linearized.q_bf)
title('Linearized Attitude - Quaternion')
xlabel('Time (s)')
ylabel('Attitude')
grid on
legend('q0','q1','q2','q3')

%%
figure(5)

plot(log.true.time,log.true.angVel)
title('True Angular Velocity')
xlabel('Time (s)')
ylabel('Rate (rad/s)')
grid on
legend('X','Y','Z')

%%
figure(6)

plot(log.linearized.time,log.linearized.angVel)
title('Linearized Angular Velocity')
xlabel('Time (s)')
ylabel('Rate (rad/s)')
grid on
legend('X','Y','Z')

%%
figure(7)

plot(log.linearized.time,log.linearized.angAccel)
title('Linearized Angular Acceleration')
xlabel('Time (s)')
ylabel('Acceleration (rad/s^2)')
grid on
legend('X', 'Y', 'Z')

%%
figure(8)

plot(log.true.time,eul(:,3:-1:1) - trueEul(:,3:-1:1))
title('Open Loop Euler Angle Error')
xlabel('Time (s)')
ylabel('Error (rad)')
grid on
legend('X','Y','Z')

%%
figure(9)

plot(atdConfig.tSpan, impulse)
title('Impulses')
xlabel('Time (s)')
ylabel('Moment (Nm)')
grid on
legend('X','Y','Z')