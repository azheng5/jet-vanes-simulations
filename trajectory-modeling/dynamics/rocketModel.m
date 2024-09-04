function [xDot,log] = rocketModel(t,x,config)
% rocketModel Open loop model for jet vanes rocket dynamics
%
% t = current time
% x = current state = [px py pz vx vy vz q0 q1 q2 q3 wx wy wz]'
% config = rocket config struct
%
% xDot = f(x,u)
% log = stuff to log

    %% Print time to 2ndary terminal
    persistent terminalCounter resetTerminalCounter;
    
    if isempty(terminalCounter)
        terminalCounter = 0;
    end
    
    if isempty(resetTerminalCounter)
        resetTerminalCounter = 0;
    end
    
    terminalCounter = terminalCounter + 1;
    resetTerminalCounter = resetTerminalCounter + 1;
    
    if resetTerminalCounter >= 3000
         tprintf('rocketModel');
         resetTerminalCounter = 0;
    end
    
    if terminalCounter >= 100
        tprintf('rocketModel',num2str(t));
        terminalCounter = 0;
    end

    %% Extract config variables
    tSpan = config.tSpan;
    uSpan = config.uSpan;
    wetMass = config.wetMass;
    dryMass = config.dryMass;
    % baseIb = config.baseIb;
    % invBaseIb = config.invBaseIb;
    radius = config.radius;
    g = config.g;
    burnTime = config.burnTime;
    wetMOI = config.wetMOI;
    dryMOI = config.dryMOI;
    thrustCurve = config.thrustCurve;
    CG0 = config.CG0;
    finalCG = config.finalCG;
    radius = config.radius;

    %% Extract states
    
    % position in flat earth frame f (m)
    px = x(1);
    py = x(2);
    pz = x(3);
    p_f = [px;py;pz];

    % velocity in body frame f (m/s)
    vx = real(x(4));
    vy = real(x(5));
    vz = real(x(6));
    v_b = [vx;vy;vz];
    
    % unit quat that rotates from flat earth to Body
    q0 = real(x(7));
    q1 = real(x(8));
    q2 = real(x(9));
    q3 = real(x(10));
    q_bf = quatnormalize([q0 q1 q2 q3]);
    
    % angular velocity wrt flat earth frame expressed in body frame (rad/s)
    wx = x(11);
    wy = x(12);
    wz = x(13);
    wbf_b = [wx;wy;wz];
    
    %% Get intermediate values
    % getRho = @(h) 0.0023769 .* (1 - 6.877e-6 .* h).^4.25588;
    rho = 0.0023769 .* (1 - 6.877e-6 .* px).^4.25588; % air density
    rho = rho * 515.379;
    T = get_thrust(thrustCurve, t,burnTime); %get thrust based on thrust curve, elapsed time, and motor burn time

    if t < burnTime
        m = wetMass - ((wetMass - dryMass)/burnTime) * t;
        CG = CG0 - ((CG0 - finalCG)/burnTime) * t;
        Ib = wetMOI - ((wetMOI - dryMOI)/burnTime) .* t;
    else
        m = dryMass;
        CG = finalCG;
        Ib = dryMOI;
    end

    alpha = atan2(real(vy),real(vx)); % angle of attack (rad)
    beta = atan2(vz,(vx^2 + vy^2)^0.5); % angle of sideslip (rad) asin(vy/Va)?
    
    %% GRAVITY
    FG_v = -m*g*[1;0;0]; % gravity force in vehicle carried frame (aligned with flat earth frame)
    FG_b = quat_rotate(q_bf,FG_v'); % parts(quatconj(q_obj)*quaternion([0;FG_v]')*q_obj);
    FG_b = FG_b';
    
    %% THRUST
    %     %thrust force in body frame
    %     FT_b = [T;
    %             0.25*T*(u1+u3);
    %             0.25*T*(u2+u4)];
    %     
    %     %thrust moment in body frame
    %     MT_b = [(T/4)*(radius/4)*(-u1+u2+u3-u4);
    %             (T/4)*CG*(u2+u4);
    %             -(T/4)*CG*(u1+u3)];
    FT_b = [T;0;0]; % 0.98

    if uSpan == 0 % interp1 strong effect on sim perf
            MT_bx = 0;
            MT_by = 0;
            MT_bz = 0;
    else
            MT_bx = interp1(tSpan,uSpan(:,1),t);
            MT_by = interp1(tSpan,uSpan(:,2),t);
            MT_bz = interp1(tSpan,uSpan(:,3),t);

            % NOTE: for f(x,u) do this instead
            % MT_bx = u(1);
            % MT_by = u(2);
            % MT_bz = u(3);
    end
    MT_b = [MT_bx;MT_by;MT_bz];

    %% AERODYNAMICS
    [FA_b, MA_b] = calcAeroBody(alpha,norm(v_b),rho,CG);
    
    %% State derivatives
    % Currently we are unable to factor out m(t) from the MOI. Factoring
    % out an m(t) is beneficial bc it allows the base MOI to be constant,
    % which speeds up sims. If we ever are able to in the future, use 
    % this trick vvv
    % Ib = m.*baseIb;
    % invIb = (1/m).*invBaseIb;
    
    % position qns of motion
    p_f_dot = quat_rotate(quatconj(q_bf),v_b'); %  parts(q_obj*quaternion([0;V_b]')*quatconj(q_obj));
    p_f_dot = p_f_dot';
    
    % velocity eqns of motion
    F_b = FG_b + FT_b + FA_b;
    v_b_dot = (1/m)*F_b - cross(wbf_b, v_b);
    
    % quaternion eqns of motion
    q0_dot = 0.5 * (-wx*q1 - wy*q2 - wz*q3);
    q1_dot = 0.5 * (wx*q0 + wz*q2 - wy*q3);
    q2_dot = 0.5 * (wy*q0 - wz*q1 + wx*q3);
    q3_dot = 0.5 * (wz*q0 + wy*q1 - wx*q2);
    q_bf_dot = [q0_dot;q1_dot;q2_dot;q3_dot];
    
    % angular rate eqns of motion
    M_b = MT_b + MA_b;
    
    %     wbf_b_dot = [0;0;0];
    %     wbf_b_dot(1) = 0;
    %     wbf_b_dot(2) = 0;
    %     wbf_b_dot(3) = M_b(3)/Ib(3,3);

    wbf_b_dot = Ib\(M_b - cross(wbf_b, Ib*wbf_b)); % inv(A)*b == A\b
%     wbf_b_dot = inv(Ib) * (M_b - cross(wbf_b, Ib*wbf_b)); % slower
    
    % stack components
    xDot = [p_f_dot; v_b_dot; q_bf_dot; wbf_b_dot];
    
    % add to log
    log.alpha = alpha;
    log.beta = beta;
    % log.D0 = D0;
    % log.L = L;
    % log.Di = Di;
    % log.M_ps = M_ps;
    % log.MT_b = MT_b';
    % log.MA_b = MA_b';
    log.CG = CG;
    % log.FT_b = FT_b';
    % log.F_b = F_b';
    log.MA_b = MA_b';
    log.MT_b = MT_b';
    
    end