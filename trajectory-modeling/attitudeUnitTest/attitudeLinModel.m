function xDot = attitudeLinModel(t,x,atdConfig)
% attitudeLinModel Linearized state space model of attitude
% x = [q1 q3 q3 wx wy wz]
% tic

%% Extract config variables
K = atdConfig.K;
tSpan = atdConfig.tSpan;
runClosedLoop = atdConfig.runClosedLoop;
refAtd = atdConfig.refAtd;
tSpan = atdConfig.tSpan;
uSpan = atdConfig.uSpan;
Ixx = atdConfig.Ixx;
Iyy = atdConfig.Iyy;
Izz = atdConfig.Izz;



%% Define state
q1 = x(1);
q2 = x(2);
q3 = x(3);
wx = x(4);
wy = x(5);
wz = x(6);

if runClosedLoop
    r = interp1(tSpan,refAtd,t);
    uExt = interp1(tSpan,uSpan,t);
    u = r' - K*x + uExt';
else
    Mx = interp1(tSpan,uSpan(:,1),t);
    My = interp1(tSpan,uSpan(:,2),t);
    Mz = interp1(tSpan,uSpan(:,3),t);
    u = [Mx;My;Mz];
end

A = [ 0, 0, 0,                    1/2,                     0,                      0;
      0, 0, 0,                      0,                   1/2,                      0;
      0, 0, 0,                      0,                     0,                    1/2;
      0, 0, 0,                      0, (Iyy*wz - Izz*wz)/Ixx,  (Iyy*wy - Izz*wy)/Ixx;
      0, 0, 0, -(Ixx*wz - Izz*wz)/Iyy,                     0, -(Ixx*wx - Izz*wx)/Iyy;
      0, 0, 0,  (Ixx*wy - Iyy*wy)/Izz, (Ixx*wx - Iyy*wx)/Izz,                      0];

B = [     0,     0,     0;
          0,     0,     0;
          0,     0,     0;
      1/Ixx,     0,     0;
          0, 1/Iyy,     0;
          0,     0, 1/Izz];

xDot = A*x + B*u;

% toc

end