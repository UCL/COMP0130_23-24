% This example demonstrates how to use the graph to estimate the state of a
% vehicle which is driving around. The vehicle's position is being
% estimated using an ideal range-bearing sensor which measures the range
% and bearing to zero or more landmarks in the environment. A landmark
% is visible if it's within a given distance of the vehicle. In addition,
% the bearing must be below a threshold

import g2o.core.*;
import odometry.*;

% Some parameters
numberOfTimeSteps = 4000;

% Number of landmarks and layout
numberOfLandmarks = 200;
extent = 300;

% Nominal odometry
odometry=[1 0 pi/180]';

% Measurement information
maxRange = 30;
maxTheta = 60 * pi / 180;
R = diag([1 pi/180].^2);

% Odometry covariance
Q = diag([0.1 0.05 pi/180].^2);

% Work out the information matrices
omegaR = inv(R);
omegaQ = inv(Q);

% Ground truth array
trueX = zeros(3, numberOfTimeSteps);

% Position observations
Z = cell(1, numberOfTimeSteps);

% Observation matrix
H = [1 0 0;0 1 0];

% Populate the landmark locations
landmarks = (rand([2 numberOfLandmarks]) - 0.5) * extent;

% Extract the observations from the first timestep
Z{1} = hFunRB(trueX(:,1), landmarks, maxRange, maxTheta, R);

% Now predict the subsequent steps
for k = 2 : numberOfTimeSteps
    priorX = trueX(:, k-1);
    v = sqrtm(Q) * randn(3, 1);
    M = [cos(priorX(3)) -sin(priorX(3));
        sin(priorX(3)) cos(priorX(3))];
    predictedX = priorX;
    predictedX(1:2) = predictedX(1:2) + M * (odometry(1:2) + v(1:2));
    predictedX(3) = g2o.stuff.normalize_theta(predictedX(3) + odometry(3) + v(3));
    trueX(:, k) = predictedX;
    Z{k} = hFunRB(trueX(:,k), landmarks, maxRange, maxTheta, R);
end

% Create the graph
graph = SparseOptimizer();
algorithm = LevenbergMarquardtOptimizationAlgorithm();
%algorithm = MatlabNLOptimizationAlgorithm();
%algorithm = GaussNewtonOptimizationAlgorithm();
graph.setAlgorithm(algorithm);

% This array contains the set of vertices for the target state over time
vertices = cell(numberOfTimeSteps, 1);

% This is the prior we compute for the graph. Unlike the true value above
% (which uses the real odometry), the prior here is computed using the
% nominal odometry information
X0 = zeros(3, numberOfTimeSteps);

% Now create the vertices and edges

for k = 1 : numberOfTimeSteps
    
    % Create the object state vertex
    vertices{k} = VehicleStateVertex();
    
    % Added the vertex to the graph.
    graph.addVertex(vertices{k});
    
    % If this isn't the first vertex, predict the nominal value and add an
    % edge
    if (k > 1)
        % Create the edge
        processModelEdge = VehicleKinematicsEdge();
        processModelEdge.setVertex(1, vertices{k-1});
        processModelEdge.setVertex(2, vertices{k});
        processModelEdge.setMeasurement(odometry);
        processModelEdge.setInformation(omegaQ);
        graph.addEdge(processModelEdge);
        
        % Predict the state for the initial condition
        priorX = X0(:, k-1);
        M = [cos(priorX(3)) -sin(priorX(3));
            sin(priorX(3)) cos(priorX(3))];
        predictedX = priorX;
        predictedX(1:2) = predictedX(1:2) + M * odometry(1:2);
        predictedX(3) = g2o.stuff.normalize_theta(predictedX(3) + odometry(3));
        X0(:, k) = predictedX;
    end
            
    % Set the initial estimate.
    vertices{k}.setEstimate(X0(:,k));
    
    % Now process the observations (if any are available)
    for m = 1 : length(Z{k}.landmarkIDs)
        e = LandmarkRangeBearingEdge(landmarks(:, Z{k}.landmarkIDs(m)));
    
        % Link it so that it connects to the vertex we want to estimate
        e.setVertex(1, vertices{k});
    
        % Set the measurement value and the measurement covariance
        e.setMeasurement(Z{k}.z(:, m));
        e.setInformation(omegaR);
    
        % Add the edge to the graph
        graph.addEdge(e);
    end
end

% Graph construction complete

% Initialise the optimization. This is done here because it's a bit
% expensive and if we cache it, we can run multiple optimizations without
% having to build everything from scratch
graph.initializeOptimization();

% Create some output as we go
x = zeros(3, numberOfTimeSteps);

% First copy the state values - these are the prior we set the graph to
for k = 1 : numberOfTimeSteps
    x(:, k) = vertices{k}.estimate();
end

% Plot the prior and the truth
figure(1)
clf

% Plot. Note that we capture the line handle. This is overkill in this
% case, but it's a useful habit to get into for labelling graphs.
gH(1)=plot(x(1, :), x(2,:));
hold on
gH(2)=plot(trueX(1, :), trueX(2, :));
plot(landmarks(1,:), landmarks(2, :), '+')
drawnow

% Optimize the graph
tic
graph.optimize(40)
toc

% Extract the optimized state estimate and plot
for k = 1 : numberOfTimeSteps
    x(:, k) = vertices{k}.estimate();
end
gH(3)=plot(x(1, :), x(2, :), 'LineWidth', 2);

%gH(4)=plot(z(1,:),z(2,:));

% Generate the legend
legend(gH, {'Prior', 'Truth', 'Optimized'});
title([num2str(graph.chi2())])
drawnow

