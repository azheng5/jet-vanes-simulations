function [FA_b MA_b] = calcAeroBody(alpha,vel,rho,CG)
%calcAeroBody Wrapper for aero model for calculating aero forces and 
% moments for the nonlinear rocket model. Aero equations are only valid 
% up to roughly 10 degrees angle of attack.
%
% alpha - angle of attack (rad)
% vel - velocity (m/s)
% rho - air density (kg/m^3)
% CG - location of CG (m)
%
% FA_b - aero force in body frame (N)
% MA_b - aero moment in body frame about current CG (Nm)

    %% Convert from metric to english for aero fittings
    
    alphaRad = alpha;
    alphaDeg = alphaRad * (180/pi) ; % rad to deg
    vel = vel * 3.281; % m/s to ft/s
    rho = rho * 0.00194;% kg/m^3 to slug/ft^3
    CG = CG * 3.281; % m/s to ft/s

    %% Aero fittings (in British units)
    CM = @(a, v) a .* ((4.12421 ./ (27.81553 + 16.83430 .* sqrt(1 - (v./1097.40).^2))) + -0.00014 .* a + 0.01273);
    CD = @(a) 0.00253 * a.^2 + 0.13932;
    CL = @(a, v) 0.64181 .* a .* (0.64181 ./ (3.59306 + sqrt(1 - (v./1097.40).^2)));
    
    Mo = @(a, v, rho) CM(a, v) .* 0.5 .* rho .* v.^2 .* 0.9375;
    Dmag = @(a, v, rho) CD(a) .* 0.5 .* rho .* v.^2 .* 0.9375;
    Lmag = @(a, v, rho) CL(a,v) .* 0.5 .* rho .* v.^2 .* 0.9375;

    %% Aero force in body frame
    L_b = Lmag(alphaDeg,vel,rho) .* [sin(alphaRad);-cos(alphaRad);0];
    D_b = Dmag(alphaDeg,vel,rho) .* [-cos(alphaRad);-sin(alphaRad);0];
    FA_b = L_b + D_b; % lbf
    FA_b = FA_b .* (1/0.225); % lbf to N

    %% Aero moment in body frame about current CG

    % Need to perform moment transfer
    % Aero fittings for moment are taken about an OpenRocket CG0
    % Need to perform a moment transfer to SolidWorks CG0 then to current
    % CG0

    % Difference btw OpenRocket CG0 and SolidWorks CG0
    % TODO assuming 0 for now will need to change later
    CG0diff = 0; % ft

    % Moment about OpenRocket CG0
    MA_b_CG0 = [0;0;Mo(alphaDeg,vel,rho)];
    
    % Moment about SolidWorks CG0
    % r points from CG0_corr to CG0
%     MA_b_CG0_corr = MA_b_CG0 + cross(FA_b,-1.*[(CG+CG0diff);0;0]);

    % Moment about current CG
%     MA_b = MA_b_CG0_corr + cross(FA_b,-1*[CG;0;0]);

    MA_b = MA_b_CG0 .* 1.36; % lbf-ft to Nm
    

end