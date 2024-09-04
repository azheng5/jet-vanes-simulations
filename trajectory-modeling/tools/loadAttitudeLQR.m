function K = loadAttitudeLQR(Ixx,Iyy,Izz,wx,wy,wz)
% Assume w=0, which is only valid for small deviations from w
% TODO write LQR loader for multiple trim points, not just w=0

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

q = [10 10];
Q = [q(1) 0 0 0 0 0;
     0 q(1) 0 0 0 0;
     0 0 q(1) 0 0 0;
     0 0 0 q(2) 0 0;
     0 0 0 0 q(2) 0;
     0 0 0 0 0 q(2)];
R = eye(3);

[K, S, P] = lqr(A,B,Q,R);

end