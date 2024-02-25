% This class implements the range-bearing measurement edge.
%
% The measurement model is z(k+1)=h[x(k+1)]+w(k+1)
%
% The measurements are r(k+1) and beta(k+1) and are given as follows.
% The sensor is at (sx, sy, stheta).
%
% dx = x(k+1) - sx; dy = y(k+1) - sy
%
% r(k+1) = sqrt(dx^2+dy^2)
% beta(k+1) = atan2(dy, dx) - stheta
%
% The error model is given by
%
% w(k+1)=z(k+1) - h[x(k+1)]
%
% The Jacobians are adapted from https://github.com/petercorke/robotics-toolbox-matlab/blob/master/RangeBearingSensor.m

classdef ObjectPolarMeasurementEdge < g2o.core.BaseUnaryEdge
   
    properties(Access = protected)
        % The x,y and theta of the sensor
        sensorPose;
    end
    
    methods(Access = public)
    
        function this = ObjectPolarMeasurementEdge()
            this = this@g2o.core.BaseUnaryEdge(2);
            this.sensorPose = zeros(3, 1);
        end
        
        function setSensorPose(this, sensorPose)
            this.sensorPose = sensorPose;
        end
        
        function computeError(this)
            dXY = this.edgeVertices{1}.x([1 3]) - this.sensorPose(1:2);
            this.errorZ(1) = this.z(1) - norm(dXY);
            this.errorZ(2) = g2o.stuff.normalize_theta(this.z(2) - atan2(dXY(2), dXY(1)) + this.sensorPose(3));
        end
        
        function linearizeOplus(this)
            
            dXY = this.edgeVertices{1}.x([1 3]) - this.sensorPose(1:2);
            r = norm(dXY);
            
            % Work out Jacobians
            this.J{1} = [-dXY(1)/r 0 -dXY(2)/r 0;
                dXY(2)/r^2 0 -dXY(1)/r^2 0];
        end        
    end
end