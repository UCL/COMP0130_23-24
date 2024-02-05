% This class stores an event from the scnario generator. The known
% even types are:
%
% InitialConditionEvent:
% This is a one-off event sent at the start. The data is
% the initial state (position and velocity) of the platform.
%
% GPSObservationEvent:
% This specifies a GPS measurement of vehicle (x,y). The covariance
% is R^GPS_k.
%
% HeartbeatEvent:
% This is sent on a regular basis. It contains no data, but forces the
% SLAM system to predict and update.
%
% LandmarkObservationEvent:
% This specifies the range, attitude and elevation measurements of the
% landmarks. The event also contains the IDs of the detected landmarks.
% The observations are in an 3xn array, where n is the number of landmarks
% observed at the current timestep

classdef Event < handle

    % Enumeration of observation types
    properties(Access = public, Constant = true)
        INITIAL_CONDITION = 0;
        GPS = 1;
        LANDMARK = 2;
        HEARTBEAT = 3;
    end
    
    properties(Access = public)
        
        % The time of the event.
        time; 
        
        % The type of the event. Must be one of the enums listed above.
        type;
        
        % The event data
        data;
        
        % The noise on the event data
        covariance;
    end
    
    methods(Access = public)
        function this = Event(time, type, data, covariance)
            
            % Copy over the common values
            this.time = time;
            this.type = type;
            this.data = data;
            
            % If the noise is a vector, assume that it encodes a diagonal covariance
            % matrix. Therfore, reshape into a matrix.
            if ((size(covariance, 1) == 1) || (size(covariance, 2) == 1))
                covariance = diag(covariance);
            end            
            this.covariance = covariance;
        end
    end
    
end
