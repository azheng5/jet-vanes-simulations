%% Script for calculating ground EKF functions

% Define state variables
syms abx aby abz abrx abry abrz gbx gby gbz gbrx gbry gbrz

% Define other stuff
syms dt ax ay az gx gy gz

state = [abx aby abz abrx abry abrz gbx gby gbz gbrx gbry gbrz];
state_transition_fn = [abx + dt*abrx;
                       aby + dt*abry;
                       abz + dt*abrz;
                       abrx;
                       abry;
                       abrz;
                       gbx + dt*gbrx;
                       gby + dt*gbry;
                       gbz + dt*gbrz;
                       gbrx;
                       gbry;
                       gbrz;];
state_transition_jacob = jacobian(state_transition_fn,state);

measurement = [ax;ay;az;gx;gy;gz];
observation_fn = [0 + abx;
                  0 + aby;
                  9.81 + abz;
                  0 + gbx;
                  0 + gby;
                  0 + gbz];
observation_jacob = jacobian(observation_fn,state);

% function f = state_transition_fn(ekf)
%     % Calculates next state for ground EKF
% 
%     f = [PN;
%          PE;
%          PD;
%          q0 + dt*q1*(gbx/2 - gx/2) + dt*q2*(gby/2 - gy/2) + dt*q3*(gbz/2 - gz/2);
%          q1 - dt*q0*(gbx/2 - gx/2) + dt*q3*(gby/2 - gy/2) - dt*q2*(gbz/2 - gz/2);
%          q2 - dt*q3*(gbx/2 - gx/2) - dt*q0*(gby/2 - gy/2) + dt*q1*(gbz/2 - gz/2);
%          q3 + dt*q2*(gbx/2 - gx/2) - dt*q1*(gby/2 - gy/2) - dt*q0*(gbz/2 - gz/2);
%          abx + dt*abrx;
%          aby + dt*abry;
%          abz + dt*abrz;
%          abrx;
%          abry;
%          abrz;
%          gbx + dt*gbrx;
%          gby + dt*gbry;
%          gbz + dt*gbrz;
%          gbrx;
%          gbry;
%          gbrz;
%          mbx + dt*mbrx;
%          mby + dt*mbry;
%          mbz + dt*mbrz;
%          mbrx;
%          mbry;
%          mbrz];
% end

% function dfdx = state_transition_jacob(ekf)
%     % Calculates state transition jacobian for ground EKF
% 
%     dfdx = [[1, 0, 0,                  0,                  0,                  0,                  0, 0, 0, 0,  0,  0,  0,          0,          0,          0,  0,  0,  0, 0, 0, 0,  0,  0,  0];
%             [0, 1, 0,                  0,                  0,                  0,                  0, 0, 0, 0,  0,  0,  0,          0,          0,          0,  0,  0,  0, 0, 0, 0,  0,  0,  0];
%             [0, 0, 1,                  0,                  0,                  0,                  0, 0, 0, 0,  0,  0,  0,          0,          0,          0,  0,  0,  0, 0, 0, 0,  0,  0,  0];
%             [0, 0, 0,                  1,  dt*(gbx/2 - gx/2),  dt*(gby/2 - gy/2),  dt*(gbz/2 - gz/2), 0, 0, 0,  0,  0,  0,  (dt*q1)/2,  (dt*q2)/2,  (dt*q3)/2,  0,  0,  0, 0, 0, 0,  0,  0,  0];
%             [0, 0, 0, -dt*(gbx/2 - gx/2),                  1, -dt*(gbz/2 - gz/2),  dt*(gby/2 - gy/2), 0, 0, 0,  0,  0,  0, -(dt*q0)/2,  (dt*q3)/2, -(dt*q2)/2,  0,  0,  0, 0, 0, 0,  0,  0,  0];
%             [0, 0, 0, -dt*(gby/2 - gy/2),  dt*(gbz/2 - gz/2),                  1, -dt*(gbx/2 - gx/2), 0, 0, 0,  0,  0,  0, -(dt*q3)/2, -(dt*q0)/2,  (dt*q1)/2,  0,  0,  0, 0, 0, 0,  0,  0,  0];
%             [0, 0, 0, -dt*(gbz/2 - gz/2), -dt*(gby/2 - gy/2),  dt*(gbx/2 - gx/2),                  1, 0, 0, 0,  0,  0,  0,  (dt*q2)/2, -(dt*q1)/2, -(dt*q0)/2,  0,  0,  0, 0, 0, 0,  0,  0,  0];
%             [0, 0, 0,                  0,                  0,                  0,                  0, 1, 0, 0, dt,  0,  0,          0,          0,          0,  0,  0,  0, 0, 0, 0,  0,  0,  0];
%             [0, 0, 0,                  0,                  0,                  0,                  0, 0, 1, 0,  0, dt,  0,          0,          0,          0,  0,  0,  0, 0, 0, 0,  0,  0,  0];
%             [0, 0, 0,                  0,                  0,                  0,                  0, 0, 0, 1,  0,  0, dt,          0,          0,          0,  0,  0,  0, 0, 0, 0,  0,  0,  0];
%             [0, 0, 0,                  0,                  0,                  0,                  0, 0, 0, 0,  1,  0,  0,          0,          0,          0,  0,  0,  0, 0, 0, 0,  0,  0,  0];
%             [0, 0, 0,                  0,                  0,                  0,                  0, 0, 0, 0,  0,  1,  0,          0,          0,          0,  0,  0,  0, 0, 0, 0,  0,  0,  0];
%             [0, 0, 0,                  0,                  0,                  0,                  0, 0, 0, 0,  0,  0,  1,          0,          0,          0,  0,  0,  0, 0, 0, 0,  0,  0,  0];
%             [0, 0, 0,                  0,                  0,                  0,                  0, 0, 0, 0,  0,  0,  0,          1,          0,          0, dt,  0,  0, 0, 0, 0,  0,  0,  0];
%             [0, 0, 0,                  0,                  0,                  0,                  0, 0, 0, 0,  0,  0,  0,          0,          1,          0,  0, dt,  0, 0, 0, 0,  0,  0,  0];
%             [0, 0, 0,                  0,                  0,                  0,                  0, 0, 0, 0,  0,  0,  0,          0,          0,          1,  0,  0, dt, 0, 0, 0,  0,  0,  0];
%             [0, 0, 0,                  0,                  0,                  0,                  0, 0, 0, 0,  0,  0,  0,          0,          0,          0,  1,  0,  0, 0, 0, 0,  0,  0,  0];
%             [0, 0, 0,                  0,                  0,                  0,                  0, 0, 0, 0,  0,  0,  0,          0,          0,          0,  0,  1,  0, 0, 0, 0,  0,  0,  0];
%             [0, 0, 0,                  0,                  0,                  0,                  0, 0, 0, 0,  0,  0,  0,          0,          0,          0,  0,  0,  1, 0, 0, 0,  0,  0,  0];
%             [0, 0, 0,                  0,                  0,                  0,                  0, 0, 0, 0,  0,  0,  0,          0,          0,          0,  0,  0,  0, 1, 0, 0, dt,  0,  0];
%             [0, 0, 0,                  0,                  0,                  0,                  0, 0, 0, 0,  0,  0,  0,          0,          0,          0,  0,  0,  0, 0, 1, 0,  0, dt,  0];
%             [0, 0, 0,                  0,                  0,                  0,                  0, 0, 0, 0,  0,  0,  0,          0,          0,          0,  0,  0,  0, 0, 0, 1,  0,  0, dt];
%             [0, 0, 0,                  0,                  0,                  0,                  0, 0, 0, 0,  0,  0,  0,          0,          0,          0,  0,  0,  0, 0, 0, 0,  1,  0,  0];
%             [0, 0, 0,                  0,                  0,                  0,                  0, 0, 0, 0,  0,  0,  0,          0,          0,          0,  0,  0,  0, 0, 0, 0,  0,  1,  0];
%             [0, 0, 0,                  0,                  0,                  0,                  0, 0, 0, 0,  0,  0,  0,          0,          0,          0,  0,  0,  0, 0, 0, 0,  0,  0,  1]];
% 
% 
% end