% This type of event wraps up the vehicle control inputs. These consist of
% linear speed and angular velocity. The covariance specifies the additive
% process noise term.

classdef VehicleOdometryEvent < minislam.event_types.Event   
    
    methods(Access = public)        
        
        function this = VehicleOdometryEvent(time, data, covariance)
            this = this@minislam.event_types.Event(time, minislam.event_types.EventTypes.VEHICLE_ODOMETRY, data, covariance);
        end
        
    end
end
    
