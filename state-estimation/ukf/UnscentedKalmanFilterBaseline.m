%% Unscented Kalman Filter 

%Initialize state vector and covariance matrix


%Still needs:
    %P, Q, and R
    %Process model implementation --> processModel()
    %Fake sensor data --> generateSensorData()
    %Determine dimension of control vector
    
x0 = [0; %linear velocity, x direction
      0; %linear velocity, y direction
      0; %linear velocity, z direction
      0; %rotational velocity, x direction
      0; %rotational velocity, y direction
      0; %rotational velocity, z direction
      0; %phi
      0; %theta
      0; %psi
      0; %position north
      0; %position east
      0;]; %position down

dim = length(x0);
measurementDim = 9;
timeStep = 0.2;

a = 0.1; %Parameter alpha, (0 1)
b = 2; %Parameter beta, a good choice is 2 for Gaussian problems
k = 3 - dim; %Parameter kappa, k = 3 -n

P = [1 0 0 0 0 0 0 0 0 0 0 0;
     0 1 0 0 0 0 0 0 0 0 0 0;
     0 0 1 0 0 0 0 0 0 0 0 0;
     0 0 0 1 0 0 0 0 0 0 0 0;
     0 0 0 0 1 0 0 0 0 0 0 0;
     0 0 5 0 0 1 0 0 0 0 0 0;
     0 0 0 0 0 0 1 0 0 0 0 0;
     0 0 0 0 0 0 0 1 0 0 0 0;
     0 0 0 0 0 0 0 0 1 0 0 0;
     0 0 0 0 0 0 0 0 0 3 0 0;
     0 0 0 0 0 0 0 0 0 0 5 0;
     0 0 0 0 0 0 0 0 0 0 0 8]; %initial covariance matrix, 12x12

Q = [2 0 0 0 0 0 0 0 0 0 0 0;
     0 2 0 0 0 0 0 0 0 0 0 0;
     0 0 2 0 0 0 0 0 6 0 0 0;
     0 0 0 2 0 0 0 0 0 0 0 0;
     0 0 0 0 2 0 0 0 0 0 0 0;
     0 0 0 0 0 2 0 0 0 0 0 0;
     0 0 0 0 0 0 2 0 0 0 0 0;
     0 0 0 0 0 0 0 2 0 0 0 0;
     0 0 0 0 0 0 0 0 2 0 0 0;
     0 0 0 0 0 0 0 0 0 2 0 0;
     0 0 0 0 0 0 0 0 0 0 2 0;
     0 0 0 0 0 0 0 0 0 0 0 2]; %process covariance, 12x12

R = [1 0 0 0 0 0 0 0 0;
     0 1 0 0 0 0 0 0 0;
     0 0 1 0 0 0 0 0 0;
     0 0 0 0.25 0 0 0 0 0;
     0 0 0 0 0.25 0 0 0 0;
     0 0 0 0 0 0.25 0 0 0;
     0 0 0 0 0 0 9 0 0;
     0 0 0 0 0 0 0 9 0;
     0 0 0 0 0 0 0 0 9]; %measurement covariance 9x9
x = x0;
u = zeros(4); %initialize control vector
resultArr = x0; %array for testing
ySensorData = [0]; %array for testing

%Overall algorithm
for i = 1:1:100

%Predict Step

[sigmaPointMatrix, meanWeights, covWeights] = generateSigmaPoints(x, dim, a, b, k, P);
    %sigmaPointMatrix has each sigma point in a column
    %meanWeights and covWeights are (2n+1)x1 vectors
forwardedSigmaPointMatrix = processModel(sigmaPointMatrix, timeStep, u);
    %projects the sigma points forward in time based on nonlinear process
    %model
[meanSigmaPts, covSigmaPts] = unscentedTransformPredict(meanWeights, covWeights, forwardedSigmaPointMatrix, dim, Q);
    %computes the mean and covariance of the sigma points projected forward
    %in time
%Predict step is mostly done, just need to implement the nonlinear
%equations in the process model function

%Update Step

measurementSigmaPts = measurementFunc(forwardedSigmaPointMatrix, sigmaPointMatrix, timeStep);
%"Measurement function" that converts the sigma points of the prior into a
%measurement. So this would take in forwardedSigmaPointMatrix and convert
%them into those things that are measured by the sensors (IMU, GPS). This
%generates a new set of sigma points with each sigma point representing a
%measurement. These sigma points can have a dimension less than that of the
%previous state sigma points. 

[measureMeanWeights, measureCovWeights] = computeMeasurementWeights(dim, a, b, k);
[measureMeans, measureCovs] = measurementUnscentedTransform(measureMeanWeights, measureCovWeights, measurementSigmaPts, measurementDim, R);
%Then, compute the mean and covariance of these sigma points using the
%unscented transform, taking into account the measurement covariance R.
%This is done with the two functions above.

iteration = i; %i is from the for loop that this is running within
sensorData = generateSensorData(timeStep, iteration);
residual = sensorData - measureMeans;
%Next, take into account the measurements from the sensors. Subtract the
%mean of the measurement sigma points from the actual measurements to form
%the residual.

P_xz = computeCrossCov(dim, measureCovWeights, measurementSigmaPts, measureMeans, forwardedSigmaPointMatrix, meanSigmaPts);
%Compute the cross covariance of the state and the measurements P_xz
K = P_xz*inv(measureCovs);
%Compute the Kalman gain, P_xz *inv(P_z). 12x9 matrix

x_new = meanSigmaPts + K*residual;
%Compute the new state estimate using residual and Kalman gain, x_new =
%x_old + K*y

P_new = covSigmaPts - K*measureCovs*(transpose(K));
%Compute the new covariance, P_new = P_old - K*P_z*transpose(K)

x = x_new;
P = P_new;

resultArr = [resultArr x];
ySensorData = [ySensorData sensorData(8)];

end

t = 0:0.2:20; %Compare perfect dynamics with results after sensor noise
plot(t, resultArr(11,:))
hold on
plot(t, 0.5.*t.*t)
plot(t, ySensorData)
legend('Filtered Y Position', 'True Motion in Y Direction', 'Sensor Data for Y Position')
xlabel('Time (s)', 'FontSize', 20)
ylabel('Y Position (m)', 'FontSize', 20)
title('1-D Kinematic UKF Results', 'FontSize', 30)



function[sigPtMat, meanWeights, covWeights]= generateSigmaPoints(x, dim, a, b, k, P)

lambda = (a^2)*(dim + k) - dim;

sigPt_0 = transpose(x);
sigmaPt_1 = zeros(12); %dummy matrices for sigma points beyond the mean
sigmaPt_2 = zeros(12);

offset_squared = (dim + lambda).*P;
offset_squared = 0.5*(offset_squared + offset_squared') + dim*eye(dim);
offset = chol(offset_squared); %"square root of a matrix" via Cholesky factorization

for i = 1:1:dim
    sigmaPt = transpose(x) + offset(i, :);
    sigmaPt_1(i, :) = sigmaPt;
end

for i = 1:1:dim
    sigmaPt = transpose(x) - offset(i, :);
    sigmaPt_2(i,:) = sigmaPt;
end

sigPtMatTranspose = [sigPt_0; sigmaPt_1; sigmaPt_2]; %(2n+1)x12 matrix containing each row as a sigma point
sigPtMat = transpose(sigPtMatTranspose); %12x(2n+1) matrix containing each column as a sigma point

%Now need to calculate weights

Wm_0 = lambda/(dim+lambda); %weight of the mean sigma point
Wm_i = 1/(2*(dim+lambda)); %weight of every other sigma point

Wc_0 = (lambda/(dim+lambda)) + 1 - a^2 + b; %weight of the covariance of the mean sigma point
Wc_i = 1/(2*(dim+lambda)); %weight of the covariance of every other sigma points

meanWeightFill = zeros(2*dim + 1, 1);
covWeightFill = zeros(2*dim + 1,1);

meanWeightFill(1) = Wm_0;
meanWeightFill(2:end) = Wm_i;
covWeightFill(1) = Wc_0;
covWeightFill(2:end) = Wc_i;

meanWeights = meanWeightFill;
covWeights = covWeightFill;

end

function[updatedMean, updatedCov]= unscentedTransformPredict(meanWeights, covWeights, sigmaPointMat, dim, Q)
%computes the mean and covariance of the sigma points in the "predict" step
%of the algorithm, taking into account process covariance Q
updatedMean = sigmaPointMat*meanWeights;

emptyCov = zeros(dim,dim);

    for i = 1:1:(2*dim+1)
        sigmaPt = sigmaPointMat(:, i);
        block1 = sigmaPt - updatedMean;
        block2 = transpose(block1);
        weight = covWeights(i);

        unweightedCov = block1*block2;
        weightedCov = weight*unweightedCov;
        emptyCov = emptyCov + weightedCov;
    end

updatedCov = emptyCov + Q;
end

function[newSigmaPoints]= processModel(sigmaPointMat, timeStep, u)
    %This function projects the sigma points forward in time according to
    %the nonlinear process model.

   sigmaPointMat(1,:) = sigmaPointMat(1,:); %u
   sigmaPointMat(2,:) = sigmaPointMat(2,:) + 1*timeStep; %v %1 is acceleration
   sigmaPointMat(3,:) = sigmaPointMat(3,:);

   sigmaPointMat(11,:) = sigmaPointMat(11,:) + 0.5*1*timeStep*timeStep; %p_y %1 is acceleration

   newSigmaPoints = sigmaPointMat;

end

function [measurementSigmaPoints]= measurementFunc(sigmaPointMat, prevSigmaPointMat, timeStep)

measurementSigPts = zeros(9, 25);
measurementSigPts(1:3, :) = (sigmaPointMat(1:3,:) - prevSigmaPointMat(1:3,:))./timeStep;
    %The above line converts linear velocities to linear accelerations
measurementSigPts(4:6, :) = sigmaPointMat(4:6, :); %rotational velocities in world frame
measurementSigPts(7:9, :) =sigmaPointMat(10:12, :); %Positions in world frame
%A measurement sigma point is 9x1, in order, a_x, a_y, a_z, p, q, r, p_x,
%p_y, p_z

measurementSigmaPoints = measurementSigPts; %output
end

function [measurementMeanWeights, measurementCovWeights]= computeMeasurementWeights(dim, a, b, k)
n = dim;
lambda = (a^2)*(n+k) - n;

w0_m = lambda/(n+lambda);
w0_c = w0_m + 1 - a^2 + b;

wi_m = 1/(2*(n+lambda));
wi_c = wi_m;

meanWeightFill = zeros(2*dim + 1, 1);
covWeightFill = zeros(2*dim + 1,1);

meanWeightFill(1) = w0_m;
meanWeightFill(2:end) = wi_m;
covWeightFill(1) = w0_c;
covWeightFill(2:end) = wi_c;

measurementMeanWeights = meanWeightFill;
measurementCovWeights = covWeightFill;

end

function[updatedMean, updatedCov]= measurementUnscentedTransform(meanWeights, covWeights, measSigmaPointMat, measDim, R)
%computes the mean and covariance of the sigma points in the "predict" step
%of the algorithm, taking into account process covariance Q
updatedMean = measSigmaPointMat*meanWeights;

emptyCov = zeros(measDim,measDim);

    for i = 1:1:(2*measDim+1)
        sigmaPt = measSigmaPointMat(:, i);
        block1 = sigmaPt - updatedMean;
        block2 = transpose(block1);
        weight = covWeights(i);

        unweightedCov = block1*block2;
        weightedCov = weight*unweightedCov;
        emptyCov = emptyCov + weightedCov;
    end

updatedCov = emptyCov + R;
end

function[data]= generateSensorData(timeStep, iteration)
    %Generate a 9x1 vector data with
    %a_x, a_y, a_z, p, q, r, p_x, p_y, p_z
    a_x = 0;
    a_y = (1);
    a_z = 0;
    p = 0;
    q = 0;
    r = 0;
    p_x = 0;
    p_y = 0.5*(a_y)*(timeStep*iteration)^2;
    p_z = 0;

    dataNotNoisy = [a_x; a_y; a_z; p; q; r; p_x; p_y; p_z];
    
    accelNoise = normrnd(0,1);
    rotNoise = normrnd(0,0.5);
    posNoise = normrnd(0, 3);

    noise = zeros(9,1);
    noise(1:3) = accelNoise;
    noise(4:6) = rotNoise;
    noise(7:9) = posNoise;

    data = dataNotNoisy + noise;


end

function[P_xz]= computeCrossCov(dim, covWeights, measSigPts, measMean, sigPts, sigMean)

emptyMat = zeros(12, 9);

for i = 1:1:2*dim+1
    block_1 = sigPts - sigMean;
    block_2 = measSigPts - measMean;
    block_2 = transpose(block_2);
    block_main = covWeights(i)*(block_1*block_2);
    emptyMat = emptyMat + block_main;
end

P_xz = emptyMat;

end