function [f0]= cost_PB_rocket_model(Z)

%% Sort Z vector into respective elements

X = Z(1:10);
U = Z(11:13);

u = Z(1);
v = Z(2);
w = Z(3);
p = Z(4);
q = Z(5);
r = Z(6);
q0 = Z(7);
q1 = Z(8);
q2 = Z(9);
q3 = Z(10);

M_roll = Z(11);
M_pitch = Z(12);
T = Z(13);

xdot = PB_rocket_model(X,U); %Find rate of change of states with dynamic model

udot = xdot(1);
vdot = xdot(2);
wdot = xdot(3);
pdot = xdot(4);
qdot = xdot(5);
rdot = xdot(6);
q0dot = xdot(7);
q1dot = xdot(8);
q2dot = xdot(9);
q3dot = xdot(10);

%% Set up the cost function

num_constraints = 20; %Let the number of constraints happen here

Q = ones(num_constraints, 1);
Q(1) = vdot;
Q(2) = wdot;
Q(3) = pdot;
Q(4) = qdot;
Q(5) = rdot;
Q(6) = q0 - 1;
Q(7) = q1;
Q(8) = q2;
Q(9) = q3;
Q(10) = u - 100;
Q(11) = v;
Q(12) = w;
Q(13) = p;
Q(14) = q;
Q(15) = r;
Q(16) = q0dot;
Q(17) = q1dot;
Q(18) = q2dot;
Q(19) = q3dot;
Q(20) = T - 1500;

H = diag(ones(1,length(num_constraints))); %Should be dimensions of the number of constraints
H(20, 20) = 5; %Increase cost scalar for particular constraint if needed
H(16, 16) = 3;

f0 = Q'*H*Q; %Computer the cost function


end