function [wx, wy, wz] = getGyro(elapsedTime)
   wx = 0.2; %rad/s
   wy = 0.15*sin(elapsedTime); %rad/s
   wz = 0.04*cos(elapsedTime); %rad/s
   
end