function CG = getCG(t)
% getCG Gets CG vector at a point in time
    % Note: CG0 is the Solidworks CG0, not the OpenRocket CG0

    CGrate = 0; % TODO: waiting on CG rate
    CG = CG0 - CGrate * t;

end