% This edge encodes a range bearing measurement.

% Jacobian from https://github.com/petercorke/robotics-toolbox-matlab/blob/master/RangeBearingSensor.m

classdef LandmarkRangeBearingEdge < g2o.core.BaseUnaryEdge
    
    properties(Access = protected)
        % Coordinates of the landmark
        landmark;
    end
    
    methods(Access = public)
    
        function this = LandmarkRangeBearingEdge(landmark)
            this = this@g2o.core.BaseUnaryEdge(2);
            this.landmark = landmark;
        end
        
        function computeError(this)
            x = this.edgeVertices{1}.estimate();
            dx = this.landmark(1:2) - x(1:2);
            
            this.errorZ(1) = norm(dx) - this.z(1);
            this.errorZ(2) = g2o.stuff.normalize_theta(atan2(dx(2), dx(1)) - x(3) - this.z(2));
        end
        
        function linearizeOplus(this)
            
            x = this.edgeVertices{1}.estimate();
            dx = this.landmark(1:2) - x(1:2);
            r = norm(dx);
            
            this.J{1} = ...
                [-dx(1)/r -dx(2)/r 0;
                dx(2)/r^2 -dx(1)/r^2 -1];
            %this.J{2} = - this.J{1}(1:2, 1:2);
        end        
    end
end