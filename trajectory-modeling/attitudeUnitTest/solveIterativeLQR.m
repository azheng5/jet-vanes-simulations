function log = solveIterativeLQR(lqrConfig,numRuns,calcXdots)
    % solveIterativeLQR Runs monte carlos for iterative LQR controller -
    % attitude unit test version
    %
    % lqrConfig - config struct for iterative LQR
    % numRuns - number of runs to perform Monte Carlo
    % calcXdots - Flag that enables relooping thru the ode solver to obtain
    % xdot (state derivative) data in addition to the default x (state) 
    % data. Disabling speeds up sims by a decent amount.

    % Begin recording elapsed time
    tic

    % Allocate memory for array of structs
    log(numRuns) = struct();

    % Loop through the specified number of MC runs
    % TODO implement MC parallelization feature w parallel toolbox
    for runIndex = 1:numRuns

        fprintf('Simulating monte carlo iterative LQR: Run %d out of %d...\n',runIndex,numRuns)
        
        % Generate a model configuration for the current run
        % TODO nice to have - remove hardcoded indices to atd config params, autodect
        % param names in the excel file
        atdConfig.runClosedLoop = mcConfig.runClosedLoop;
        xlsxFile = readtable(mcConfig.xlsxFile);
        atdConfig.Ixx = random(makedist(xlsxFile.Distribution{1},'mu',xlsxFile.Mean(1),'sigma',xlsxFile.ThreeSigma(1)/3));
        atdConfig.Iyy = random(makedist(xlsxFile.Distribution{2},'mu',xlsxFile.Mean(2),'sigma',xlsxFile.ThreeSigma(2)/3));
        atdConfig.Izz = random(makedist(xlsxFile.Distribution{3},'mu',xlsxFile.Mean(3),'sigma',xlsxFile.ThreeSigma(3)/3));

        atdConfig.K = loadAttitudeLQR(atdConfig.Ixx, atdConfig.Iyy, atdConfig.Izz,0,0,0);
        
        initRoll = random(makedist(xlsxFile.Distribution{4},'mu',xlsxFile.Mean(4),'sigma',xlsxFile.ThreeSigma(4)/3)); % xlsx RPY is in DEGREES
        initPitch = random(makedist(xlsxFile.Distribution{5},'mu',xlsxFile.Mean(5),'sigma',xlsxFile.ThreeSigma(5)/3));
        initYaw = random(makedist(xlsxFile.Distribution{6},'mu',xlsxFile.Mean(6),'sigma',xlsxFile.ThreeSigma(6)/3));
        initRoll = initRoll * (pi/180);
        initPitch = initPitch * (pi/180);
        initYaw = initYaw * (pi/180);

        tempQuat = eul2quat([initYaw, initPitch, initRoll],"ZYX"); %  takes the form [zAngle yAngle xAngle] if ZYX
        atdConfig.initTrueAtd = [tempQuat(1);tempQuat(2);tempQuat(3);tempQuat(4);0;0;0;initRoll;initPitch;initYaw];
        atdConfig.initAtd = [tempQuat(2);tempQuat(3);tempQuat(4);0;0;0];

        atdConfig.endTime = mcConfig.endTime;
        atdConfig.tStep = mcConfig.tStep;
        atdConfig.simHz = 1/atdConfig.tStep;
        atdConfig.tSpan = 0:atdConfig.tStep:atdConfig.endTime;
        atdConfig.refAtd = ones( length(atdConfig.tSpan),3) .* [0 0 0];

        % Use impulses
        impulse_x = generateImpulse(0,0.001,20,2,atdConfig.tSpan);
        impulse_x(1) = 0; % if impulse at t-0.0 huge discontinuity occurs
        impulse_y = generateImpulse(0,0.00,30,200,atdConfig.tSpan);
        impulse_z = generateImpulse(0,0.00,10,200,atdConfig.tSpan);
        impulse = [impulse_x impulse_y impulse_z];
        atdConfig.uSpan = impulse;

        % Solve for continuous time closed loop linearized trajectory
        % TODO log intermediate variables like in run_open_loop
        for runIndex = 1:
        [atdT, atdX] = ode45(@(t,x) attitudeLinModel(t,x,atdConfig), ...
                            atdConfig.tSpan, ...
                            atdConfig.initAtd);

        % Calculate xdots
        if calcXdots == 0
            atdXdot = zeros(length(atdT),6);
            for i = 1:length(atdT)
                atdXdot(i,:) = attitudeLinModel(atdT(i),atdX(i,:)',atdConfig).';
            end
        end

        % Log data for the current run
        tempLog.q_bf = [ones(length(atdT),1) , atdX(:,1:3)];
        tempLog.angVel = atdX(:,4:6);
        if calcXdots == 0
            tempLog.angAccel = atdXdot(:,4:6);
        end
        tempLog.time = atdT;
        tempLog.atdConfig = atdConfig;
        
        % Hacky method used - log has a redundant "loggedData" substruct to
        % get to the actual data (tempLog). Without it, a 'Subscripted 
        % assignment between dissimilar structures' will be thrown on 
        % empty struct. A possible alternative soln is to initialize the
        % exact same format that a log struct would use and standardize it.
        log(runIndex).loggedData = tempLog;


    end

    fprintf("Finished Monte Carlo runs for linearized attitude trajectory!\n")
    toc

end