% This class wraps up a "compass" measurement. We use quotes because the
% measurements are assumed to be corrupted by zero-mean, Gaussian noise
% sources. As you have seen in the first part of this module, the actual
% noises in a compass are much more complicated, with stuff like soft
% and hard iron biases

classdef CompassObservationEvent < minislam.event_types.Event
    
    properties(Access = protected)
        theta;
    end
    
    methods(Access = public)
        
        function this = CompassObservationEvent(time, data, covariance, thetaOffset)
            this = this@minislam.event_types.Event(time, minislam.event_types.EventTypes.COMPASS, data, covariance);
            this.theta = thetaOffset;
        end
        
        function theta = thetaOffset(this)
            theta = this.theta;
        end        
    end
end
