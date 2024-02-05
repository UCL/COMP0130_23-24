% This script supports task0, which is the simulator running on its own.
% You don't have to do anything else other than check it runs. However,
% you might find it interesting to play with the different configuration
% settings to see what they all do.

% The simulator configuration. This specifies how the simulation
% environment will work, such as noise levels and how long the simulation
% will run for.
simulatorConfiguration = dotbot.SimulatorConfiguration();

% The graphics output configuration. This controls how much and how often
% graphics should be shown, together with things such as how large various
% icons should be.
graphicsConfiguration = dotbot.GraphicalOutputConfiguration();

% Create the simulator - this generates the environment, simulates the
% movement of the dotbot and the sensors over time.
simulator = dotbot.DotBotSimulator(simulatorConfiguration);

% Create the graphics output object which manages the output figure.
graphicalOutput = dotbot.GraphicalOutput(graphicsConfiguration, simulator);

% Start up
simulator.start();
graphicalOutput.start();

% Keep running in the mainloop until the simulator says we are done
while (simulator.keepRunning() == true)
    simulator.step();
    graphicalOutput.update();
end

% Get the time stamps and ground truth dotbot states over time
[T, XTrueHistory] = simulator.platformHistory();

% Plot out state information
simulationResultsFigure = minislam.graphics.FigureManager.getFigure('Simulation Results');

plot(T, XTrueHistory)
legend({'$x$','$\dot{x}$','$y$','$\dot{y}$'},'Interpreter','latex')
xlabel('Time (s)')
ylabel('State quantity ($m$ or $ms^{-1}$)','Interpreter','latex')