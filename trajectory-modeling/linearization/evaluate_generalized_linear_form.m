%% Script to Evaluate the matrices E, A', and B' to generate a generalized rocket linear systems

% State variable derivatives

syms x1dot x2dot x3dot x4dot x5dot x6dot x7dot x8dot x9dot x10dot

% State variables

syms x1 x2 x3 x4 x5 x6 x7 x8 x9 x10

% Control inputs (u3 = thrust, not actually a control input but dynamically
% needs to be considered as one)

syms u1 u2 u3

% Variables that vary with trim point parameters

syms m g Ixx Iyy Izz

% Variables that need to be later evaluated in terms of state variables

syms FAxb FAyb FAzb MAxb MAyb MAzb


% Define the system of equations

F = [FAxb/m + u3/m + 2*g*x8*x10 - 2*g*x7*x9 - x5*x3 + x2*x6 - x1dot,
     FAyb/m + 2*g*x7*x8 + 2*g*x9*x10 - x1*x6 + x3*x4 - x2dot,
     FAzb/m + g*x7*x7 - g*x8*x8 - g*x9*x9 + g*x10*x10 - x2*x4 + x1*x5 - x3dot,
     MAxb/Ixx + u1/Ixx - x5*x6*(Izz - Iyy)/Ixx - x4dot,
     MAyb/Iyy + u2/Iyy - x4*x6*(Ixx - Izz)/Iyy - x5dot,
     MAzb/Izz - x4*x5*(Iyy - Ixx)/Izz - x6dot,
     -x4*x8/2 - x5*x9/2 - x6*x10/2 - x7dot,
     x4*x7/2 + x6*x9/2 - x5*x10/2 - x8dot,
     x5*x7/2 - x6*x8/2 + x4*x10/2 - x9dot,
     x6*x7/2 + x5*x8/2 - x4*x9/2 - x10dot];



%Define what the derivatives are being taken with respect to

xdot = [x1dot, x2dot, x3dot, x4dot, x5dot, x6dot, x7dot, x8dot, x9dot, x10dot];
x = [x1 x2 x3 x4 x5 x6 x7 x8 x9 x10];
u = [u1 u2 u3];


% Find the matrix E

E = jacobian(F, xdot)

% Find the matrix A'

A_prime = jacobian(F, x)

% Find the matrix B'

B_prime = jacobian(F, u)



% Linearized form of the system
A = -1*(inv(E))*A_prime
B = -1*(inv(E))*B_prime

% Then delta_xdot = A*delta_x + B*delta_u



