classdef CompassMeasurementEdge < g2o.core.BaseUnaryEdge
   
    % Q1c:
    % This implementation contains a bug. Identify the problem
    % and fix it as per the question.

    properties(Access = protected)
        
        compassAngularOffset;
        
    end
    
    methods(Access = public)
    
        function this = CompassMeasurementEdge(compassAngularOffset)
            this = this@g2o.core.BaseUnaryEdge(1);
            this.compassAngularOffset = compassAngularOffset;
        end
        
        function computeError(this)
            x = this.edgeVertices{1}.estimate();
            this.errorZ = x(3) + this.compassAngularOffset - this.z;
        end
        
        function linearizeOplus(this)
            this.J{1} = [0 0 1];
        end        
    end
end