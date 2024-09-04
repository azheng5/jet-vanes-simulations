%% Gyro Main

dt = 0.02;
initialTime = 0;
timeStep = 0;
elapsedTime = 0;

q_current = getInitialAttitude(0, pi/2, 0); %psi, theta, phi -- straight up on pad
quat_initial = quaternion(q_current(1), q_current(2), q_current(3), q_current(4));
patch = poseplot(quat_initial)
xlabel('North')
ylabel('East')
zlabel('Down')


%% NED Frame

poseplot(quaternion(1,0,0,0))
%%
for i = 1:1:10000

    [wx, wy, wz] = getGyro(elapsedTime);
    q_delt = gyroToInstantaneousRotationQuat(wx, wy, wz, dt);
    q_new = quatMulitply(q_current, q_delt);

    quat_new = quaternion(q_new(1), q_new(2), q_new(3), q_new(4));
    set(patch, Orientation=quat_new)
    drawnow
    
    q_current = q_new;
    elapsedTime = initialTime + i*dt;

end
