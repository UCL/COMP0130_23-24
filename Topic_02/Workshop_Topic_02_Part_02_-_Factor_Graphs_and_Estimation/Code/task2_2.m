% Dynamic object example. This shows how to build a graph to estimate an
% object which has both position and velocity

import g2o.core.*;
import two_d_tracking.*;

% Some parameters
numberOfTimeSteps = 1000;
dT = 1;
sigmaR = diag([10 1*pi/180]).^2;
sigmaQ = 0.01;

% Some sensor positions
sensorPose=[50 50 0]';

% Work out the state transition equations
F0=[1 dT; 0 1];
Q0=[dT^3/3 dT^2/2;dT^2/2 dT] * sigmaQ;

F = [F0 zeros(2); zeros(2) F0];
Q = [Q0 zeros(2); zeros(2) Q0];

% Work out the information matrices
omegaR = inv(sigmaR);
omegaQ = inv(Q);

% Ground truth array
trueX = zeros(4, numberOfTimeSteps);
z = zeros(2, numberOfTimeSteps);

% First timestep
trueX(2, 1) = 0.1;
trueX(4, 1) = -0.1;

dXY = trueX([1 3], 1) - sensorPose(1:2);
z(:, 1) = [norm(dXY); atan2(dXY(2), dXY(1)) - sensorPose(3)];

% Now predict the subsequent steps
for k = 2 : numberOfTimeSteps
    trueX(:, k) = F * trueX(:, k - 1) + sqrtm(Q) * randn(4, 1);
    dXY = trueX([1 3], k) - sensorPose(1:2);
    z(:, k) = [norm(dXY); atan2(dXY(2), dXY(1)) - sensorPose(3)];
end

z = z + sqrtm(sigmaR) * randn(2, numberOfTimeSteps);
z(2,:) = g2o.stuff.normalize_thetas(z(2,:));

% Create the graph
graph = SparseOptimizer();
algorithm = LevenbergMarquardtOptimizationAlgorithm();
graph.setAlgorithm(algorithm);

% This array contains the set of vertices for the target state over time
v = cell(numberOfTimeSteps, 1);

% Now create the vertices and edges

xy = zeros(2, numberOfTimeSteps);
vxvy = zeros(2, numberOfTimeSteps);

for n = 1 : numberOfTimeSteps
    
    % Create the first object state vertex
    v{n} = ObjectStateVertex();

    % Set the initial estimate.
    xy(:, n) = z(1,n) .* [cos(z(2,n) + sensorPose(3));sin(z(2,n) + sensorPose(3))]+sensorPose(1:2);
    if (n > 2)
        vxvy(:,n) = (xy(:,n)-xy(:,n-1)) / dT;
    end
    
    v{n}.setEstimate([xy(1, n) 0 xy(2, n) 0]');
        
    % Added the vertex to the graph.
    graph.addVertex(v{n});
    
    % If this isn't the first vertex, add the dynamics
    if (n > 1)
        processModelEdge = ObjectProcessModelEdge();
        processModelEdge.setVertex(1, v{n-1});
        processModelEdge.setVertex(2, v{n});
        processModelEdge.setMeasurement([0;0;0;0]);
        processModelEdge.setF(F);
        processModelEdge.setInformation(omegaQ);
        graph.addEdge(processModelEdge);
    end
    
    % Create the measurement edge
    e = ObjectPolarMeasurementEdge();
    e.setSensorPose(sensorPose);
    
    % Link it so that it connects to the vertex we want to estimate
    e.setVertex(1, v{n});
    
    % Set the measurement value and the measurement covariance
    e.setMeasurement(z(:,n));
    e.setInformation(omegaR);
    
    % Add the edge to the graph; the graph now knows we have these edges
    % which need to be added
    graph.addEdge(e);
    
end

% Graph construction complete

% Initialise the optimization. This is done here because it's a bit
% expensive and if we cache it, we can do it multiple times
graph.initializeOptimization();

% Create some output as we go
x0 = zeros(4, numberOfTimeSteps);

% First copy the state values - these are the prior we set the graph to
for n = 1 : numberOfTimeSteps
    x0(:, n) = v{n}.estimate();
end

% Plot the prior and the truth
figure(1)
clf

% Plot. Note that we capture the line handle. This is overkill in this
% case, but it's a useful habit to get into for labelling graphs.
gH(1)=plot(x0(1, :), x0(3,:), '-*');
hold on
gH(2)=plot(trueX(1, :), trueX(3, :), '-+');

% Optimize the graph
tic
graph.optimize(10)
toc

% Extract the optimized state estimate and plot
x = zeros(4, numberOfTimeSteps);
for n = 1 : numberOfTimeSteps
    x(:, n) = v{n}.estimate();
end
gH(3)=plot(x(1, :), x(3, :), 'LineWidth', 2);

zXY = z(1,:) .* [cos(z(2,:) + sensorPose(3));sin(z(2,:) + sensorPose(3))];

gH(4)=plot(zXY(1,:) + sensorPose(1), zXY(2,:) + sensorPose(2), '-o');

% Generate the legend
legend(gH, {'Prior', 'Truth', 'Optimized', 'Observation'});
title([num2str(graph.chi2())])
drawnow

