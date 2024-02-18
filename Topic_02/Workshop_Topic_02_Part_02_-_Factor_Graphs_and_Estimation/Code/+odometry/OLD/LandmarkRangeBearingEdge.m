% This edge encodes a range bearing measurement.

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
            error('Implement this');
        end
        
        function linearizeOplus(this)
            error('Implement this');
        end        
    end
end