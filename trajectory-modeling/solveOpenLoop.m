function log = solveOpenLoop(mcConfig,numRuns,calcXdots)
% solveOpenLoop Solve open loop rocket model for a number of specified runs
%
% config - xlsx file containing open loop configs
% numRuns - number of runs to perform Monte Carlo
% calcXdots - Flag that enables relooping thru the ode solver to obtain
% xdot (state derivative) data in addition to the default x (state) 
% data. Disabling speeds up sims by a decent amount.

    % Begin recording elapsed time
    tic

    % Allocate memory for array of structs
    log(numRuns) = struct();

    % Loop through the specified number of MC runs
    for runIndex = 1:numRuns

        fprintf('Simulating open loop trajectory: Run %d out of %d...\n',runIndex,numRuns)
        

        %% Generate a model configuration for the current run
        config = mcConfig;

        % TODO nice to have - remove hardcoded indices to atd config params, autodect
        % param names in the excel file
        xlsxFile = readtable(mcConfig.xlsxFile);

        initRoll = random(makedist(xlsxFile.Distribution{1},'mu',xlsxFile.Mean(1),'sigma',xlsxFile.ThreeSigma(1)/3)); % xlsx RPY is in DEGREES
        initPitch = random(makedist(xlsxFile.Distribution{2},'mu',xlsxFile.Mean(2),'sigma',xlsxFile.ThreeSigma(2)/3));
        initYaw = random(makedist(xlsxFile.Distribution{3},'mu',xlsxFile.Mean(3),'sigma',xlsxFile.ThreeSigma(3)/3));
        initRoll = initRoll * (pi/180);
        initPitch = initPitch * (pi/180);
        initYaw = initYaw * (pi/180);

        tempQuat = eul2quat([initYaw, initPitch, initRoll],"ZYX"); %  takes the form [zAngle yAngle xAngle] if ZYX
        config.initState(7:10) = [tempQuat(1);tempQuat(2);tempQuat(3);tempQuat(4)];

        initAlpha = random(makedist(xlsxFile.Distribution{1},'mu',xlsxFile.Mean(runIndex+3),'sigma',xlsxFile.ThreeSigma(runIndex+3)/3));
        init_vy = 0.01;
        init_vx = init_vy / tan(deg2rad(initAlpha));
        config.initState(4) = init_vx;
        config.initState(5) = init_vy;
        fprintf('%d %d %d\n',initAlpha,init_vx,init_vy)

        %% Solve for continuous time closed loop linearized trajectory
        [traj.t, traj.x, traj.te, traj.ye, traj.ie] = ode45(@(t,x) rocketModel(t,x,config), ...
                                                            config.tSpan, ...
                                                            config.initState, ...
                                                            config.options);

        %% Log data for the current run
        tempLog.config = config;
        tempLog.time = traj.t;
        tempLog.posFlat = traj.x(:,1:3);
        tempLog.velBody = traj.x(:,4:6);
        tempLog.q_bf = traj.x(:,7:10);
        tempLog.angVel = traj.x(:,11:13);
        [tempLog.length ~] = size(traj.t); % length of trajectory time array
        tempLog.angleOfAttack = zeros([length(traj.t),1]); % alpha
        tempLog.sideslipAngle = zeros([length(traj.t),1]); % beta
        tempLog.CG = zeros([length(traj.t)],1);
        % tempLog.F_b = zeros([length(traj.t),3]);
        tempLog.angAccel = zeros(length(traj.t),3);
        tempLog.MA_b = zeros(length(traj.t),3);
        tempLog.MT_b = zeros(length(traj.t),3);
        
        if calcXdots == 0
            for i = 1:length(traj.t)
                [xdot,modelLog] = rocketModel(traj.t(i),traj.x(i,:)',config);
                tempLog.angleOfAttack(i) = modelLog.alpha;
                tempLog.sideslipAngle(i) = modelLog.beta;
                tempLog.CG(i) = modelLog.CG;
                tempLog.angAccel(i,:) = xdot(11:13);
                tempLog.MA_b(i,:) = modelLog.MA_b;
                tempLog.MT_b(i,:) = modelLog.MT_b;
            end
        end
        
        % Hacky method used - log has a redundant "loggedData" substruct to
        % get to the actual data (tempLog). Without it, a 'Subscripted 
        % assignment between dissimilar structures' will be thrown on 
        % empty struct. A possible alternative soln is to initialize the
        % exact same format that a log struct would use and standardize it.
        log(runIndex).loggedData = tempLog;


    end

    fprintf("Finished open loop runs!\n")
    toc

end