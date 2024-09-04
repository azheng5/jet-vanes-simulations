
%%
figure(1)

grid on
title('Linearized Attitude - Euler Angles')
xlabel('Time (s)')
ylabel('Attitude (rad)')
legend('X','Y','Z')

for runIndex = 1:numRuns
    % define q0 as IC of true q0 (unchanging)
    eul = quat2eul(mcLog(runIndex).q_bf);
    
    plot(mcLog(1).time, eul(:,3:-1:1));
    hold on

end

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
figure(9)

plot(atdConfig.tSpan, impulse)
title('Impulses')
xlabel('Time (s)')
ylabel('Moment (Nm)')
grid on
legend('X','Y','Z')