function [OUT] = rocket_model_implicit(XDOT,X,U)
%rocket_model_implicit Implicit form of the rocket dynamics

OUT = rocket_model(X,U) - XDOT;

end