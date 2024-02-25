% This enumerated type stores the type of event.
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

classdef EventTypes

    % Enumeration of observation types
    enumeration
        INITIAL_CONDITION,
        GPS,
        LANDMARK,
        HEARTBEAT,
        COMPASS,
        VEHICLE_ODOMETRY
    end
end