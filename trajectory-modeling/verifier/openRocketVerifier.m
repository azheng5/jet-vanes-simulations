function openRocketVerifier()
% Visualize Open Rocket trajectory - used for verification of open loop
% sims

xlsxFile = readtable('openRocketTrajectory.xlsx');

time = xlsxFile.Time_s_;
altitude = xlsxFile.Altitude_ft_;
verticalVelocity = xlsxFile.VerticalVelocity_ft_s_;
verticalAcceleration = xlsxFile.VerticalAccelerationFt_s_2_;
thrust = xlsxFile.Thrust_lbf_;

figure(1)
plot(time,altitude)
title('OpenRocket Altitude')
xlabel('Time (s)')
ylabel('Altitude (ft)')
grid on

figure(2)
plot(time,verticalVelocity)
title('OpenRocket Vertical Velocity')
xlabel('Time (s)')
ylabel('Vertical Velocity (ft/s)')
grid on

figure(3)
plot(time,verticalAcceleration)
title('OpenRocket Vertical Acceleration')
xlabel('Time (s)')
ylabel('Vertical Acceleration (ft/s^2)')
grid on

figure(4)
plot(time,thrust)
title('OpenRocket Thrust')
xlabel('Time (s)')
ylabel('Thrust (lbf)')
grid on

end