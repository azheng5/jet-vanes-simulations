function [thrust_curve, burn_time] = generate_thrust_curve(Tmax, burn_time)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
thrust_curve = [Tmax %0 seconds
                0.76*Tmax %1 second
                0.78*Tmax %2 seconds
                0.75*Tmax %3 seconds
                0.68*Tmax %4 seconds
                0.63*Tmax %5 seconds
                0.54*Tmax %6 seconds
                0.47*Tmax %7 seconds
                0.36*Tmax %8 seconds
                0.28*Tmax %9 seconds
                0.20*Tmax %10 seconds
                0.15*Tmax %11 seconds
                0.11*Tmax %12 seconds
                0.04*Tmax]; %13 seconds

burn_time = burn_time;

%Data is roughly for Cesaroni 7400M520-P motor from www.thrustcurve.org
%Burn time ~13 seconds
%Each entry in the array is the burn time at (index - 1) seconds
end