function [xDot,log] = xDotPrimitive(t,x,uSpan,tSpan,rkt)
%rocket_ode Takes in state, outputs the xdot
    % Time varying system due to varying thrust and mass
t
m = rkt.m;
baseIb = rkt.baseIb;
invBaseIb = rkt.invBaseIb;
airDensity = rkt.airDensity;
R = rkt.R;
g = rkt.g;

%% Extract states

% position in flat earth frame f (m)
px = x(1);
py = x(2);
pz = x(3);

% velocity in body frame v_b (m/s)
vx = x(4);
vy = x(5);
vz = x(6);
v_b = [vx;vy;vz];

% unit quat that rotates from FlatEarth to Body
q0 = x(7);
q1 = x(8);
q2 = x(9);
q3 = x(10);
q_bf = quatnormalize([q0 q1 q2 q3]);

% angular velocity wrt flat earth frame expressed in body frame (rad/s)
wx = x(11);
wy = x(12);
wz = x(13);
wbf_b = [wx;wy;wz];

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

%% Get intermediate values
T = get_thrust(rkt.thrust_curve, t, rkt.motor_burn_time); %get thrust based on thrust curve, elapsed time, and motor burn time
%TODO add mSpan from openrocket
% cg = interp1(rkt.cgSpan(:,1),rkt.cgSpan(:,2),t,'linear','extrap');
% cg = cg * 0.305; % ft to m
% cg = rkt.length - cg; % dist from nozzle to cg
CG = getCG(t);
alpha = atan2(vy,vx); % angle of attack (rad)
% beta = atan2(vy,vx); % angle of sideslip (rad) asin(vy/Va)?

%% GRAVITY
FG_v = -m*g*[1;0;0]; % gravity force in vehicle carried frame (aligned with flat earth frame)
FG_b = quat_rotate(q_bf,FG_v'); % parts(quatconj(q_obj)*quaternion([0;FG_v]')*q_obj);
FG_b = FG_b';

%% THRUST
%thrust force in body frame
FT_b = [T;
        0.25*T*(u1+u3);
        0.25*T*(u2+u4)];

%thrust moment in body frame
MT_b = [(T/4)*(R/4)*(-u1+u2+u3-u4);
        (T/4)*cg*(u2+u4);
        -(T/4)*cg*(u1+u3)];

%% AERODYNAMICS
[FA_b, MA_b] = calcAeroBody(alpha,norm(v_b),airDensity,CG);

%% State derivatives
Ib = m.*baseIb;
invIb = (1/m).*invBaseIb;

% position qns of motion
p_f_dot = quat_rotate(quatconj(q_bf),v_b'); %  parts(q_obj*quaternion([0;V_b]')*quatconj(q_obj));
p_f_dot = p_f_dot';

% velocity eqns of motion
F_b = FG_b + FT_b + FA_b;
v_b_dot = (1/m)*F_b;% - cross(wbe_b, V_b);

% quaternion eqns of motion
q0_dot = 0.5 * (-wx*q1 - wy*q2 - wz*q3);
q1_dot = 0.5 * (wx*q0 + wz*q2 - wy*q3);
q2_dot = 0.5 * (wy*q0 - wz*q1 + wx*q3);
q3_dot = 0.5 * (wz*q0 + wy*q1 - wx*q2);
q_bf_dot = [q0_dot;q1_dot;q2_dot;q3_dot];

% angular rate eqns of motion
M_b = MT_b + MA_b;
wbf_b_dot = invIb * (M_b - cross(wbf_b, Ib*wbf_b));

% stack components
xDot = [p_f_dot; v_b_dot; q_bf_dot; wbf_b_dot];

% add to log
log.alpha = alpha;
log.beta = beta;
% log.D0 = D0;
% log.L = L;
% log.Di = Di;
% log.M_ps = M_ps;
log.MT_b = MT_b';
% log.MA_b = MA_b';
log.cg = cg;
log.FT_b = FT_b';
log.F_b = F_b';

end