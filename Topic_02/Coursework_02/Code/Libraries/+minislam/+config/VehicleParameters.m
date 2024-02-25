classdef VehicleParameters < handle
   
    % This class contains the parameters of the vehicle. This includes
    % performand thresholds and noise levels.
    
    properties(Access = public)
        
        % Vehicle wheelbase. See https://en.wikipedia.org/wiki/Wheelbase.
        % Average wheelbase value from
        % Chassis Handbook: Fundamentals, Driving Dynamics, Components, Mechatronics, Perspectives
        B = 2.5;
        
        % Steer angle controls; made up
        maxDiffDeltaRate = 20*pi/180;
        maxDelta = 40 * pi / 180;
        
        % Speed controls; made up
        maxSpeed = 10;
        minSpeed = 1;
        maxAcceleration = 0.5;
        
        % Covariance of measuring the wheel speed and steer angle; made up
        Qd = diag([0.1 pi/180]).^2;
    end
    
end