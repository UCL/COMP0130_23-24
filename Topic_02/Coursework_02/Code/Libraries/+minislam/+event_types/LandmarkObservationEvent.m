% This class wraps up all the measurements to all the landmarks visible in
% a single timestep. Each measurement consists of range, azimuth and
% elevation. Multiple measurements can be taken at the same timestep. If
% there are n measurements, the data field is a 3xn matrix, where the ith
% column is the range, azimuth and elevation to the ith landmark found in
% the observation. The vector landmarkIds is n dimensional. The ith entry
% of this landmark is the Id of the landmark observed.
%
% Because the covariance is the same for all landmarks, the covariance
% field contains the standard covariance matrix which should be applied to
% all observations.

classdef LandmarkObservationEvent < minislam.event_types.Event
    
    properties(Access = public)        
        % The IDs of the landmark observations
        landmarkIds;
    end

    methods(Access = public)
        
        function this = LandmarkObservationEvent(time, data, covariances, landmarkIds)
            this = this@minislam.event_types.Event(time, minislam.event_types.EventTypes.LANDMARK, data, covariances);
            this.landmarkIds = landmarkIds;
        end
    end
end
