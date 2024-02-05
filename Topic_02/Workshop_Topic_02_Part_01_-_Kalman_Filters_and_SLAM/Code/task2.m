% This script supports task2, which is to implement the landmark
% initialization step. The main thing here is that we disable the GPS and
% enable the landmark sensor.

% The simulator configuration and graphical output configurations. See
% task0.m for a description
simulatorConfiguration = dotbot.SimulatorConfiguration();
simulatorConfiguration.enableGPS = false;
simulatorConfiguration.enableLandmarkSensor = true;
simulatorConfiguration.maximumStepNumber = 5000;

graphicsConfiguration = dotbot.GraphicalOutputConfiguration();

% Create the simulator - this simulates the movement of dotbot over time.
simulator = dotbot.DotBotSimulator(simulatorConfiguration);

% Create the localization system. This actually does the legwork of doing
% SLAM
slamSystem = dotbot.DotBotSLAMSystem(simulatorConfiguration);

% Create the graphics output object which manages the output figure. Note
% that, unlike task0, we have to add the system here as well.
graphicalOutput = dotbot.GraphicalOutput(graphicsConfiguration, simulator, slamSystem);

% Start up
simulator.start();
graphicalOutput.start();

% Keep running in the mainloop until the simulator says we are done
while (simulator.keepRunning() == true)
    simulator.step();
    events = simulator.events();
    slamSystem.processEvents(events);
    graphicalOutput.update();
end

% This shows how to plot the estimation error and +/ 2 sigma bounds

% Now extract the estimate history from the estimation algorithm
[TEstimator, X, PX] = slamSystem.platformEstimateHistory();

[TSimulator, XTrueHistory] = simulator.platformHistory();

% Plot out state information
sigmaErrorBounds = minislam.graphics.FigureManager.getFigure('Task 2 Estimation Error Results');
clf

stateLabels = {'$x$','$\dot{x}$','$y$','$\dot{y}$'};

for f = 1 : 4
    subplot(4,1,f)
    sigmaBound = 2 * sqrt(PX(f, :));
    plot(TEstimator, -sigmaBound, 'r--')
    hold on
    plot(TEstimator, sigmaBound, 'r--')
    stateError = X(f, :) - XTrueHistory(f, :);
    plot(TEstimator, stateError);
    xlabel('Time (s)')
    if (rem(f, 2) == 0)
        ylabel('Velocity $(ms^{-1})$', 'Interpreter','latex')
    else
        ylabel('Position $(ms)$', 'Interpreter','latex')
    end
    title(stateLabels{f}, 'Interpreter','latex')
end



