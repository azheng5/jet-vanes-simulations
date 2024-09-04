% Final planar aero model for jet vanes sims - courtesy of Rusheer Shegandi

% clc; clear; close all

%% Aerodynamics anonymous functions (British units)
% only valid for a = 10
CM = @(a, v) a .* ((4.12421 ./ (27.81553 + 16.83430 .* sqrt(1 - (v./1097.40).^2))) + -0.00014 .* a + 0.01273);
CD = @(a) 0.00253 * a.^2 + 0.13932;
CL = @(a, v) 0.64181 .* a .* (0.64181 ./ (3.59306 + sqrt(1 - (v./1097.40).^2)));

Mo = @(a, v, rho) CM(a, v) .* 0.5 .* rho .* v.^2 .* 0.9375;
D = @(a, v, rho) CD(a) .* 0.5 .* rho .* v.^2 .* 0.9375;
L = @(a, v, rho) CL(a,v) .* 0.5 .* rho .* v.^2 .* 0.9375;

rho = @(h) 0.0023769 .* (1 - 6.877e-6 .* h).^4.25588;

%% Visualize (assume ground level air density)
hSpan = 0:0.1:11000; % altitude span (ft)
airDensity = rho(0); % density span (slug/ft^3)

%% Visualize aero with varying AoA over complete velocity span
aList = 0:1:20;
vSpan = 0:0.1:700; % velocity span (ft/s)

numColors = length(aList); % number of colors in the gradient
colorMap = winter(numColors); % map of RGB colors

figure(1)
for a = aList
    MoSpan = Mo(a,vSpan,airDensity);
    plot(vSpan,MoSpan,'LineWidth',2,'Color',colorMap(find(aList==a),:))
    hold on
end
hold off
title('Moment at varying AoA (deg)')
xlabel('Velocity (ft/s)')
ylabel('Mo (lbf-ft)')
legend(arrayfun(@num2str,aList,'UniformOutput',false))
grid on
% cb = colorbar;
% clim([min(aList),max(aList)]);
% cb.Label.String = 'AoA (deg)';

%%
figure(2)
for a = aList
    Lspan = L(a,vSpan,airDensity);
    plot(vSpan,Lspan,'LineWidth',2,'Color',colorMap(find(aList==a),:))
    hold on
end
hold off
legend(arrayfun(@num2str,aList,'UniformOutput',false))
title('Lift at varying AoA')
xlabel('Velocity (ft/s)')
ylabel('Lift (lbf)')
grid on

figure(3)
for a = aList
    Dspan = D(a,vSpan,airDensity);
    plot(vSpan,Dspan,'LineWidth',2,'Color',colorMap(find(aList==a),:))
    hold on
end
hold off
legend(arrayfun(@num2str,aList,'UniformOutput',false))
title('Drag at varying AoA')
xlabel('Velocity (ft/s)')
ylabel('Drag (lbf)')
grid on

%% Visualize aero with varying velocity over AoA span

vList = 0:25:700;
aSpan = -10:0.01:10;

numColors = length(vList); % number of colors in the gradient
colorMap = winter(numColors); % map of RGB colors

figure(11)
for v = vList
    MoSpan = Mo(aSpan,v,airDensity);
    plot(aSpan,MoSpan,'LineWidth',2,'Color',colorMap(find(vList==v),:))
    hold on
end
hold off
title('Moment at varying velocity (ft/s)')
xlabel('AoA (deg)')
ylabel('Mo (lbf-ft)')
legend(arrayfun(@num2str,vList,'UniformOutput',false))
grid on

figure(12)
for v = vList
    Lspan = L(aSpan,v,airDensity);
    plot(aSpan,Lspan,'LineWidth',2,'Color',colorMap(find(vList==v),:))
    hold on
end
hold off
legend(arrayfun(@num2str,vList,'UniformOutput',false))
title('Lift at varying velocity (ft/s)')
xlabel('AoA (deg)')
ylabel('Lift (lbf)')
grid on

figure(13)
for v = vList
    Dspan = D(v,aSpan,airDensity);
    plot(aSpan,Dspan,'LineWidth',2,'Color',colorMap(find(vList==v),:))
    hold on
end
hold off
legend(arrayfun(@num2str,vList,'UniformOutput',false))
title('Drag at varying velocity (ft/s)')
xlabel('AoA (deg)')
ylabel('Drag (lbf)')
grid on