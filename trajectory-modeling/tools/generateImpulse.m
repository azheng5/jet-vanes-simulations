function impulse = generateImpulse(mag, stdev, period, pointyFactor, tSpan)
%generateImpulse Generates realistic interpolated impulse signal at a specified magnitude
%and rate. Interpolation mitigates the effect of sudden discontinuities in ideal 
%impulse signals.
%
% mag - value of impulse (can be negative)
% stdev - stdev of impulse mag
% period - period of impulse (ie 10 means fire impulse once every 10 sec)
% size - # of cols (aka states) to fire an impulse on
% pointyFactor - increasing causes "less pointy" impulse
    % NOTE: ensure impulse time step is larger than actual time step
% tSpan - desired time profile to send impulse signal on (ie 0:0.01:100)
% impulse - column vector

if pointyFactor < 1
    error('Impulse time step scale factor must be greater than 1')
end

impulseTimeStep = mean(diff(tSpan)) * pointyFactor;
impulseTime = 0:impulseTimeStep:tSpan(end);
impulseMask = [1:period*(1/impulseTimeStep):length(impulseTime)];
rawImpulse = zeros(length(impulseTime),1);

probDist = makedist('Normal','mu',mag,'sigma',stdev);
for i = 1:length(impulseMask)
    rawImpulse(impulseMask(i)) = random(probDist);
end

impulse = interp1(impulseTime,rawImpulse,tSpan);
impulse = impulse';

end