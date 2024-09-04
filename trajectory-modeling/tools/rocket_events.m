function [value,isterminal,direction] = rocket_events(t,x)
%rocket_events Called by ode45 to determine various rocket events
    % i.e. rocket hitting ground
value = x(1); % define event as Px = 0
isterminal = 1;  % stop integration when event happens
direction = -1;   % only stop if Px is increasing (altitude is decreasing)

end