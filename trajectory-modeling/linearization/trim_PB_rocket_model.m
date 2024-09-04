% trim_PB_rocket_model

clear
clc
close all

initialization = 0;

if (initialization == 0)
    Z_guess = zeros(13,1);

    %Initial guesses
    Z_guess(1) = 100; %u
    Z_guess(2) = 0; %v
    Z_guess(3) = 0; %w
    Z_guess(4) = 0; %p
    Z_guess(5) = 0; %q
    Z_guess(6) = 0; %r
    Z_guess(7) = 1; %q0
    Z_guess(8) = 0; %q1
    Z_guess(9) = 0; %q2
    Z_guess(10) = 0; %q3
    Z_guess(11) = 0; %M_roll
    Z_guess(12) = 0; %M_pitch
    Z_guess(13) = 1700; %thrust in N

else
    load trim_values_trimpoint_id
    Z_guess = [XStar;UStar];
end

[ZStar, f0] = fminsearch('cost_PB_rocket_model',Z_guess,...
    optimset('TolX',1e-10,'MaxFunEvals',10000,'MaxIter',10000))

XStar = ZStar(1:10);
UStar = ZStar(11:13);

%Verify that this satisfies constraints
XdotStar = PB_rocket_model(XStar, UStar)
%Other constraints

save trim_values_trimpoint_id XStar UStar