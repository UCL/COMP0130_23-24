% Test of the vehicle odometry equations

numberOfSteps = 100;

x = zeros(3, numberOfSteps);

% Step-by-step translation

measurement=[1; 0.5; pi/180];


for k = 2 : numberOfSteps
    priorX = x(:, k-1);
    M = [cos(priorX(3)) -sin(priorX(3));
        sin(priorX(3)) cos(priorX(3))];
    predictedX = priorX;
    predictedX(1:2) = predictedX(1:2) + M * measurement(1:2);
    predictedX(3) = predictedX(3) + measurement(3);
    
    dMdTheta = [-sin(priorX(3)) -cos(priorX(3));
                cos(priorX(3)) -sin(priorX(3))];
    J = eye(3);
    J(1:2, 3) = dMdTheta * measurement(1:2);    
    x(:, k) = predictedX;
end

figure(1)
clf
hold on

plot(x(1,:),x(2,:), '-o')