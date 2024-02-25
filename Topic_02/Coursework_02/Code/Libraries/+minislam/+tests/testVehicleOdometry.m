% This script shows how to run the vehicle odomoetry on its own in a
% recursive manner.

% The localization system. This base class only understands vehicle control
% input and GPS data, and will throw an error if you try to give it
% anything else.
localizationSystem = slam.VehicleLocalizationSystem();

% This keeps tabs on the current time
currentTime = 0;

% Add a vehicle odometry input, once every 10s.


tic
for k = 0 : 10
    
    % Create the event which tells the system what's coming in next
    vehicleControlInput = event_types.VehicleControlInputEvent(currentTime, [1 0], [0.1;0.1*pi/180]);
    
    % Bump the current time.
    currentTime = currentTime + 2;    
    
    % Tell the system to process this event. This will create and update
    % the graph, but will not
    localizationSystem.processEvent(vehicleControlInput);
    
    % Optimize. This shows that it's possible to run the optimizer over and
    % over again, using the results from one run as the initial conditions
    % for the next. Note this actually doesn't do anything interesting here
    % because we've added no measurements.
    localizationSystem.optimize();
end

% Now turn a corner. Note that the time steps get shorter because we can
% vary the length of the timestep

for k = 11 : 30
    vehicleControlInput = event_types.VehicleControlInputEvent(currentTime, [1 pi/180], [0.1;0.1*pi/180]);
    
    currentTime = currentTime + 1;
    
    localizationSystem.processEvent(vehicleControlInput);
end

% Optimize down here. Again, this shows that we can vary when we choose to
% do this
localizationSystem.optimize();

% Now drive a bit further, but turning sharply other way
for k = 31 : 120
    vehicleControlInput = event_types.VehicleControlInputEvent(currentTime, [1 -6*pi/180], [0.1;0.1*pi/180]);
    
    currentTime = currentTime + 1;
    
    localizationSystem.processEvent(vehicleControlInput);
end

% Optimize again. This time we want the results.
[results, marginals] = localizationSystem.optimize();


% Plot the results
figure(1)
clf
gtsam.plot3DTrajectory(results, [], marginals);
axis equal

figure(2)
gtsam.plot3DPoints(results, [], marginals);
