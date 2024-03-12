function results = mainLoop(eventGenerator, localizationSystems)

% This function runs the main loop for the MiniSLAM system. It sets up the
% event generator and the localization systems. It sets up the graphics. It
% runs the main loop and collects results.

% Set up the graphics for output
graphicalOutput = minislam.graphics.GraphicalOutput();
graphicalOutput.initialize(eventGenerator, localizationSystems);

% Helper to make passing arguments easier
if (iscell(localizationSystems) == false)
    localizationSystems = {localizationSystems};
end

% Get the number of localization systems
numLocalizationSystems = length(localizationSystems);

% Allocate the results structure
results = cell(numLocalizationSystems, 1);
for l = 1 : numLocalizationSystems
    results{l} = minislam.Results();
end

% Start the event generator
eventGenerator.start();

storeCount = 1;

% Loop until we terminate
while (eventGenerator.keepRunning() == true)
    
    % Get the step number
    storeCount = eventGenerator.stepCount() + 1;
    
    % Print out
    if (rem(storeCount, 100) == 0)
        disp(num2str(storeCount))
    end
    
    % Get the events and generate the events
    events = eventGenerator.events();
    
    % Log the ground truth
    groundTruthState = eventGenerator.groundTruth(false);
    
    % Iterate over all the localization systems and process
    for l = 1 : numLocalizationSystems
        
        % Handle the event for each localization system
        localizationSystem = localizationSystems{l};    
        localizationSystem.processEvents(events);
        runOptimizer = localizationSystem.recommendOptimization();
    
        % If requested, run the optimizer and log the time required
        if (runOptimizer == true)
            tic
            chi2 = localizationSystem.optimize();
            results{l}.optimizationTimes(storeCount) = toc;
            results{l}.chi2Time = cat(1, results{l}.chi2Time, ...
                eventGenerator.time());
            results{l}.chi2History = cat(1, results{l}.chi2History, chi2);
        else
            results{l}.optimizationTimes(storeCount) = NaN;
        end
        
        % Store ground truth in each results structure
        results{l}.vehicleTrueStateTime(storeCount) = eventGenerator.time();
        results{l}.vehicleTrueStateHistory(:, storeCount) = groundTruthState.xTrue;
    end
    
    % Draw the graphics. Since this can be fairly slow, we currently only
    % run it when the optimizer is run
    if (runOptimizer == true)%
        graphicalOutput.update();
    end
    
    eventGenerator.step();
end

% Handle the end of the run. If the graph can be optimized,

for l = 1 : numLocalizationSystems
    % Get the localization system
    localizationSystem = localizationSystems{l};  
    % If recommend optimization was true, we optimized the graph before the
    % while loop ended and so there was nothing to do. Therfore, if
    % recomment Optimization() is false, the localization system might
    % contain additional vertices and edges which need optimizing. These
    % final results are written into the position given by storeCount.
    if (localizationSystem.recommendOptimization() == false)
        tic
        chi2 = localizationSystem.optimize(20);
        results{l}.optimizationTimes(storeCount) = toc;
        results{l}.chi2Time = cat(1, results{l}.chi2Time, ...
            eventGenerator.time());
        results{l}.chi2History = cat(1, results{l}.chi2History, chi2);
    end
    [T, X, P] = localizationSystem.platformEstimateHistory();
    results{l}.vehicleStateTime = T;
    results{l}.vehicleStateHistory = X;
    results{l}.vehicleCovarianceHistory = P;
end

% Make sure that the graphics output is updated to show the final state
graphicalOutput.update();

end