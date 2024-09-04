% Symbolic work

%% Derive DCM from body to NED frame (Cvb)
% This DCM will work for any orientation of body and NED, as long as
% both are right handed frames
syms x7 x8 x9 % phi theta psi

% vehicle to intermediate frame 1
C1v = [cos(x9) sin(x9) 0;
       -sin(x9) cos(x9) 0;
       0 0 1];
C21 = [cos(x8) 0 -sin(x8);
       0 1 0;
       sin(x8) 0 cos(x8)];
Cb2 = [1 0 0;
       0 cos(x7) sin(x7);
       0 -sin(x7) cos(x7)];
Cbv = Cb2*C21*C1v;
Cvb = Cbv.';

%% Derive ag_b
syms g
ag_v = [0;0;g];
ag_b = Cbv * ag_v;