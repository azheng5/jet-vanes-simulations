function [XDOT,log] = rocket_ode(t,x,uSpan,tSpan,rkt)
%rocket_ode Takes in state, outputs the xdot
    % Time varying system due to varying thrust and mass
t
m = rkt.m;
baseIb = rkt.baseIb;
invBaseIb = rkt.invBaseIb;
rho = rkt.airDensity;
R = rkt.R;
g = rkt.g;

%% Extract state vector and control input
% velocity in body frame v_b (m/s)
vx = x(1);
vy = x(2);
vz = x(3);
V_b = [vx;vy;vz];

% angular velocity wrt earth frame expressed in body frame (rad/s)
wx = x(4);
wy = x(5);
wz = x(6);
wbe_b = [wx;wy;wz];

% unit quat that rotates from V to B (q = q_bv)
q0 = x(7);
q1 = x(8);
q2 = x(9);
q3 = x(10);
q = [q0 q1 q2 q3];
q = quatnormalize(q);

% position expressed in world frame (m)
Px = x(11);
Py = x(12);
Pz = x(13);

% jet vane deflections (rad/s)
if uSpan == 0 % interp1 strong effect on sim perf
        u1 = 0;
        u2 = 0;
        u3 = 0;
        u4 = 0;
else
        u1 = interp1(tSpan,uSpan(:,1),t);
        u2 = interp1(tSpan,uSpan(:,2),t);
        u3 = interp1(tSpan,uSpan(:,3),t);
        u4 = interp1(tSpan,uSpan(:,4),t);
end

%% Time varying values
T = get_thrust(rkt.thrust_curve, t, rkt.motor_burn_time); %get thrust based on thrust curve, elapsed time, and motor burn time
%TODO add mSpan from openrocket
cg = interp1(rkt.cgSpan(:,1),rkt.cgSpan(:,2),t,'linear','extrap');
cg = cg * 0.305; % ft to m
cg = rkt.length - cg; % dist from nozzle to cg

%% Intermediate variables
Va = sqrt(vx^2 + vy^2 + vz^2); % airspeed (m/s)
alpha = -atan2(vz,vx); % angle of attack (rad)
beta = atan2(vy,vx); % angle of sideslip (rad) asin(vy/Va)?
Q = 0.5*rho*Va^2;  % dynamic pressure (Pa)

%% Gravity force
FG_v = -m*g*[1;0;0]; % gravity force in vehicle carried frame (aligned with world frame)
FG_b = quat_rotate(q,FG_v'); % parts(quatconj(q_obj)*quaternion([0;FG_v]')*q_obj);
FG_b = FG_b';

%% Thrust forces and moments
%thrust force in body frame
FT_b = [T;
        0.25*T*(u1+u3);
        0.25*T*(u2+u4)];

%thrust moment in body frame
MT_b = [(T/4)*(R/4)*(-u1+u2+u3-u4);
        (T/4)*cg*(u2+u4);
        -(T/4)*cg*(u1+u3)];

%% Aerodynamic forces and moments in body frame
% L - lift
% D0 - zero lift drag
% Di - induced drag
% M_ps - passive moment stability
[D0, L, Di, M_ps] = jvr_passive_moment_sim(Va, Px, t, alpha*(180/pi));
M_ps = M_ps*1.356; % convert from lbft to Nm
FA_b = 0;%L + D0 + Di;
MA_b = [0;M_ps;0];

%% State derivatives
Ib = m.*baseIb;
invIb = (1/m).*invBaseIb;

% uvw eqns of motion
F_b = FG_b + FT_b + FA_b;
V_b_dot = (1/m)*F_b;% - cross(wbe_b, V_b);

% pqr eqns of motion
M_b = MT_b + MA_b;
wbe_b_dot = invIb * (M_b - cross(wbe_b, Ib*wbe_b));

% quaternion eqns of motion
q_dot0 = 0.5 * (-wx*q1 - wy*q2 - wz*q3);
q_dot1 = 0.5 * (wx*q0 + wz*q2 - wy*q3);
q_dot2 = 0.5 * (wy*q0 - wz*q1 + wx*q3);
q_dot3 = 0.5 * (wz*q0 + wy*q1 - wx*q2);
q_dot = [q_dot0;q_dot1;q_dot2;q_dot3];

% position eqns of motion
P_v_dot = quat_rotate(quatconj(q),V_b'); %  parts(q_obj*quaternion([0;V_b]')*quatconj(q_obj));
P_v_dot = P_v_dot';

% stack components
XDOT = [V_b_dot; wbe_b_dot; q_dot; P_v_dot];

log.alpha = alpha;
log.beta = beta;
log.D0 = D0;
log.L = L;
log.Di = Di;
log.M_ps = M_ps;
log.MT_b = MT_b';
log.MA_b = MA_b';

end