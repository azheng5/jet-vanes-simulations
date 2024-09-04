%% 3 DOF Rocket Simulation (GNC)

T = 500; %Newtons
rho = 95900; %Pa (N/m^2). Air density at 450 m altitude
d = 0.1016; % meters
m = 10; %kg
g = 9.81; %m/s^2
J = 4;%moment of intertia Ix or Iz (Iy is about roll axis), kgm^2
b = 200; %yaw damping, Nm/rad
rCG = 0.5; %distance between nozzle and center of mass, m

D = (0.31*rho*pi*d^2)/(8*m); %Drag, N
    %Assumes C_D roughly 0.31

%State vector is x, x_dot, y, y_dot, theta, theta_dot

y0 = [0;0;0;0;0;0]; %Initial conditions

A = [0 1 0 0 0 0;
     0 0 0 0 T/m 0;
     0 0 0 1 0 0;
     0 0 0 -D 0 0;
     0 0 0 0 0 1;
     0 0 0 0 0 -b/J];

vec = [0;0;0;T/m - g; 0; 0];

B = [0;0;0;0;0;-T*rCG];

u = 0.01745; %thrust deflection in radians, 0.01745 rads = 1 degree

tspan = 0:0.05:100;	% Simulation time span
options = odeset('RelTol', 1e-12, 'AbsTol', 1e-12);

[~, y_ode45] = ode45(@(t, y) dynamics(t, y, A, vec, B,u), tspan, y0, options);

subplot(3,1,1)
plot(tspan, y_ode45(:, 1))
xlabel('Time (s)')
ylabel('X Position (m)')

subplot(3,1,2)
plot(tspan, y_ode45(:, 3))
xlabel('Time (s)')
ylabel('Y Position (m)')

subplot(3,1,3)
plot(tspan, y_ode45(:, 5))
xlabel('Time (s)')
ylabel('Theta (radians)')

function [ydot] = dynamics(~, y, A, vec, B,u)
	ydot = A*y + vec + B.*u;
end