% This class wraps up a "GPS" measurement. We use quotes because the
% measurements are assumed to be corrupted by zero-mean, Gaussian noise
% sources. As you have seen in the first part of this module, the actual
% noises in a GNSS are much more complicated.

classdef GPSObservationEvent < minislam.event_types.Event
    
    methods(Access = public)
        
        function this = GPSObservationEvent(time, data, covariance)
            this = this@minislam.event_types.Event(time, minislam.event_types.Event.GPS, data, covariance);
        end
        
    end
end
