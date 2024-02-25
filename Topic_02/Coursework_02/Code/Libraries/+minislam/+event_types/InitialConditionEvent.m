% This class wraps an initial condition for the dotbot platform only.

classdef InitialConditionEvent < minislam.event_types.Event
    
    methods(Access = public)
        
        function this = InitialConditionEvent(time, data, covariance)
            this = this@minislam.event_types.Event(time, minislam.event_types.EventTypes.INITIAL_CONDITION, data, covariance);
        end
        
    end
end
