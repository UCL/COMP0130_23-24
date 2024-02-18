classdef GraphicalOutputConfiguration < handle
    
    % This class provides a set of parameters for controlling the graphics
   
    properties(Access = public)
        
        % Set to false to disable graphics output
        showGraphics = true;
        
        % How frequently should we update?
        updateFrequency = 5;
        pauseTime = 0.01;
    
        % All sigma covariance ellipses of this value are drawn
        sigmaEllipse = 2;
    
        % Dot bot vehicle radius
        dotBotRadius = 1;
        
        % GPS measurement
        showGPSMeasurement = true;
        showGPSCovarianceEllipse = true;
        gpsMeasurementCrossSize = 8;
        
        % Extent over which the graphics are drawn (same in x and y)
        extent = 30;
    end    
end