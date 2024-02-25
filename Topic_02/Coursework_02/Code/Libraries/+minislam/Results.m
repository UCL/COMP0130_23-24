classdef Results < handle
    
    % This structure stores potentially useful information.
    
    properties(Access = public)
    
        % For each time step, time required to run the optimization. Is NaN
        % when no optimization was run.
        optimizationTimes;
        
        % The chi2 history. Both it and the time are stored at the end of
        % each optimization run.
        chi2Time;
        chi2History;
        
        % For each time step, the ground truth of the vehicle
        vehicleTrueStateTime;
        vehicleTrueStateHistory;
        
        % For each time step, the predicted pose
        vehicleStateTime;        
        vehicleStateHistory;
        vehicleCovarianceHistory;
        
    end
    
end