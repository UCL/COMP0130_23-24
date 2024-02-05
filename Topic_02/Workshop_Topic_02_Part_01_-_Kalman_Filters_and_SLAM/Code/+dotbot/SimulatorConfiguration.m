classdef SimulatorConfiguration < handle
   
    % This class just contains a set of parameters
    
    properties(Access = public)
        
        % Time step length
        DT = 0.1;
        
        % Damping parameters for the process model
        alpha = 0.1;
        beta = 0.05;
        
        % Continuous time velocity perturbation term
        sigmaQ = 1;
        
        % If set to a finite value, the simulator artificially terminates
        % at this step number
        maximumStepNumber = 1000;
        
        % If set to true, noise is added to the sensor measurements to
        % simulate observation noise. Set to false if you want to skip
        % this. Setting to false is very helpful for debugging, because
        % with no noise injected, the errors should be zero.
        perturbWithNoise = true;
        
        % Flag to show if GPS available
        enableGPS = true;        
        
        % The period between GPS measurements
        gpsMeasurementPeriod = 1;
        
        % GPS noise measurement
        RGPS = 9 * eye(2);
        
        % Flag to show if the laser is available
        enableLandmarkSensor = true;
        
        % The period between laser measurements
        landmarkSensorMeasurementPeriod = 0.5;
        
        % Detection range - same for all landmarks
        landmarkSensorDetectionRange = 10;
        
        % Noise on each measurement
        RLandMarkSensor = diag([2 2].^2);
        
        % Map specification
        
        % Number of landmarks
        numberOfLandmarks = 40;

        % Extent over which they will be distributed [-extent, extent]
        % in both x and y
        extent = 30;
    end
    
end