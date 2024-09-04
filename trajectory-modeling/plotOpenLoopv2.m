%% %%%%%%%%%%%%%%%% Plotting %%%%%%%%%%%%%%%

figure(1)
plot3(0,0,0,'o','MarkerSize',10)
hold on
plot3(log.loggedData.posFlat(end,2),log.loggedData.posFlat(end,3),log.loggedData.posFlat(end,1),'x','MarkerSize',10)
hold on
plot3(log.loggedData.posFlat(:,2),log.loggedData.posFlat(:,3),log.loggedData.posFlat(:,1))
grid on
xlabel('x')
ylabel('y')
zlabel('z')
title('Open Loop Trajectory')
% axis equal

%%
figure(2)
plot(log.loggedData.config.tSpan,log.loggedData.config.uSpan)
legend('u1','u2','u3','u4')

%%
figure(3)

for runIndex = 1:numRuns
    plot(log(1).loggedData.time,rad2deg(log(runIndex).loggedData.angleOfAttack));
    hold on
end
title('Angle of Attack Trajectory')
xlabel('Time (s)')
ylabel('Alpha (deg)')
legend('1 deg','3 deg','6 deg', '9 deg')
grid on

%%
figure(4)
plot(log.loggedData.time,log.loggedData.velBody)
legend('x','y','z')
title('Body Velocity')
ylabel('v_b (m/s)')
xlabel('Time (s)')
grid on

%%
figure(5)

for runIndex = 1:numRuns
    plot(log(1).loggedData.time,rad2deg(log(runIndex).loggedData.angVel(:,3)));
    hold on
end
title('Body Angular Velocity')
ylabel('wbi_b (deg/s)')
xlabel('time (s)')
legend('1 deg','3 deg','6 deg', '9 deg')
grid on

%%
figure(6)
plot(log.loggedData.time,log.loggedData.q_bf)
legend('q0','q1','q2','q3')

%%
figure(7)
plot(log.loggedData.time,log.rad2deg(loggedData.angAccel))
legend('x','y','z')
%%
figure(8)
eul = quat2eul(log.loggedData.q_bf);
plot(log.loggedData.time,rad2deg(eul(:,3:-1:1)))
legend('x','y','z')
title('euler angles')

%% Linearized Attitude - Quaternion
figure(2)

for plotIndex = 1:4
    subplot(4,1,plotIndex)
    for runIndex = 1:numRuns
        plot(mcLog(1).loggedData.time,mcLog(runIndex).loggedData.q_bf(:,plotIndex))
        hold on
    end

    grid on
    title('q'+string(plotIndex-1))

end

h = axes(gcf, 'Visible', 'off');
h.XLabel.Visible = 'on';
h.YLabel.Visible = 'on';
xlabel(h, 'Time (s)');
ylabel(h, 'Attitude');

sgtitle('Linearized Attitude - Quaternion')

%%
figure(9)

for runIndex = 1:numRuns
    plot(log(1).loggedData.time,log(runIndex).loggedData.MA_b(:,3));
    hold on
end
title('Aerodynamic Body Moment About CG')
ylabel('MA_b (Nm)')
xlabel('time (s)')
legend('1 deg','3 deg','6 deg', '9 deg')
grid on

%%
figure(10)
plot(log.loggedData.time,log.loggedData.MT_b);
legend('x','y','z')

%%
figure(11)
plot(log.loggedData.angleOfAttack,log.loggedData.MA_b)
title('Aero Moment vs Angle of Attack')
xlabel('alpha (deg)')
ylabel('MA_b (Nm')

%%
figure(12)
plot(log.loggedData.time,log.loggedData.posFlat)
title('posFlat')
legend('x','y','z')
