% This class uses a slightly simpler model for the vehicle kinematics used
% in the lectures. This is the more standard built in type for estimate.
%
% The model assumes that the vehicle speed is specified in the vehicle
% frame and is then projected into the world frame. Specifically,
%
% M = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0;0 0 1];
%
% The process model has the form:
%
% x = x + M * [vx;vy;theta]
%
% where vx, vy and vtheta are the velocities.
%
% The error model 
% eTheta = 

classdef VehicleKinematicsEdge < g2o.core.BaseBinaryEdge
    
    properties(Access = protected)

    end
    
    methods(Access = public)
        function this = VehicleKinematicsEdge()
            this = this@g2o.core.BaseBinaryEdge(3);            
        end
       
        function computeError(this)
            error('Implement this');
        end
        
        % Compute the Jacobians
        function linearizeOplus(this)
            error('Implement this');
        end
    end    
end