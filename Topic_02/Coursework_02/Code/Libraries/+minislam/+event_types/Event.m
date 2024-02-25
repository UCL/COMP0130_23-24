% This class stores an event from the scenario generator.

classdef Event < handle

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
