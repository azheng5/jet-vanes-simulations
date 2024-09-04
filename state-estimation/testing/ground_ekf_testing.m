ADIS16500_accel_params = accelparams('MeasurementRange',392, ...
                                     'ConstantBias',[58.8e-3 58.8e-3 58.8e-3], ...
                                     'NoiseDensity',[880e-6 880e-6 732e-6], ...
                                     'BiasInstability',[125.0e-6 125.0e-6 134.0e-6]);

ADIS_16500_gyro_params = gyroparams('MeasurementRange',deg2rad(2000), ...
                                    'ConstantBias',deg2rad([0.14 1.4 0.14]), ...
                                    'NoiseDensity',deg2rad([6.1e-3 6.1e-3 7e-3]), ...
                                    'BiasInstability',deg2rad([7.5 8.1 4.9])*(1/3600), ...
                                    'AccelerationBias',deg2rad([0.572e-3 1.02e-3 0.408e-3]));

LIS3MDL_params = magparams('MeasurementRange',G2microT(16), ...
                           'ConstantBias',[0 0 0]);

IMU = imuSensor('IMUType','accel-gyro-mag', ...
                'SampleRate',10, ...
                'MagneticField',[27.5550 -2.4169 -16.0849], ... % at lat long alt 0
                'Accelerometer',ADIS16500_accel_params, ...
                'Gyroscope',ADIS_16500_gyro_params, ...
                'Magnetometer',LIS3MDL_params, ...
                'RandomStream','Global stream', ...
                'Seed',67);

% Ground truth data
num_samples = 6000;
acceleration = zeros(num_samples,3);
angVel = zeros(num_samples,3);

[accel_reading, gyro_reading, mag_reading] = IMU(acceleration,angVel);

%% Plot generation

t = (0:(num_samples-1))/IMU.SampleRate;

figure(1)

subplot(3,1,1)
plot(t,accel_reading)
legend('X-axis','Y-axis','Z-axis')
title('Accelerometer Readings')
ylabel('Acceleration (m/s^2)')

subplot(3,1,2)
plot(t,gyro_reading)
legend('X-axis','Y-axis','Z-axis')
title('Gyroscope Readings')
ylabel('Angular Velocity (rad/s)')

subplot(3,1,3)
plot(t,mag_reading)
legend('X-axis','Y-axis','Z-axis')
title('Magnetometer Readings')
xlabel('Time (s)')
ylabel('Magnetic Field (uT)')

function microT_value = G2microT(gauss_value)
    microT_value = gauss_value * 100;
end