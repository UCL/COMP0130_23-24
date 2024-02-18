% This script tests the vehicle model process edge to make sure both the
% error function and the Jacobian are correct. The error should be zero.
% The Jacobian is approximated by a numerical forward difference scheme.

import g2o.core.*;
import odometry.*;

% The basic odometry we'll use
odometry=[1 0 pi/180]';

% Number of steps
numberOfTimeSteps = 4;

% Odometry covariance
Q = diag([0.1 0.05 pi/180].^2);

% Work out the information matrices
omegaQ = inv(Q);

% Initial robot pose
X0 = zeros(3, numberOfTimeSteps);

% Create the poses
for k = 2 : numberOfTimeSteps
    priorX = X0(:, k-1);
    M = [cos(priorX(3)) -sin(priorX(3));
        sin(priorX(3)) cos(priorX(3))];
    predictedX = priorX;
    predictedX(1:2) = predictedX(1:2) + M * odometry(1:2);
    predictedX(3) = g2o.stuff.normalize_theta(predictedX(3) + odometry(3));
    X0(:, k) = predictedX;
end

% Vertcies used for checking

% This array contains the set of vertices for the target state over time
vertices = cell(numberOfTimeSteps, 1);

% Predict the state for the initial condition
for k = 1 : numberOfTimeSteps
    
    % Create the object state vertex
    vertices{k} = VehicleStateVertex();
    
    vertices{k}.setEstimate(X0(:,k));
    
    if (k == 1)
        continue
    end
    
    processModelEdge = VehicleProcessModelEdge();
    processModelEdge.setVertex(1, vertices{k-1});
    processModelEdge.setVertex(2, vertices{k});
    processModelEdge.setMeasurement(odometry);
    processModelEdge.setInformation(omegaQ);
        
    processModelEdge.computeError();
    processModelEdge.linearizeOplus();
    J = processModelEdge.jacobianOplus();
    
    % Numerical solutions
    e0 = processModelEdge.error();
    
    % Work out 
    Jn = zeros(3);    
    for t = 1 : 3
        X = X0(:,k-1);
        X(t) = X(t) + 1e-6;
        vertices{k-1}.setEstimate(X);
        processModelEdge.computeError();
        Jn(:,t) = processModelEdge.error() - e0;
    end
    
    disp('J{1}:')
    J{1}
    Jn / 1e-6
    
    vertices{k-1}.setEstimate(X0(:,k-1));
    
    Jn = zeros(3);
    
    for t = 1 : 3
        X = X0(:,k);
        X(t) = X(t) + 1e-6;
        vertices{k}.setEstimate(X);
        processModelEdge.computeError();
        Jn(:,t) = processModelEdge.error() - e0;
    end
    disp('J{2}:')
    J{2}    
    Jn / 1e-6
    
    vertices{k}.setEstimate(X0(:,k));


    pause
    
end

