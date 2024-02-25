% This class implements the linear position measurement edge edge.
%
% The measurement model is z(k+1)=h[x(k+1)]+w(k+1)
%
% The error is w(k+1)=z(k+1) - h[x(k+1)]


classdef ObjectMeasurementEdge < g2o.core.BaseUnaryEdge
   
    methods(Access = public)
    
        function this = ObjectMeasurementEdge()
            this = this@g2o.core.BaseUnaryEdge(2);
        end
        
        function computeError(this)
            this.errorZ = this.z - this.edgeVertices{1}.x([1 3]);
        end
        
        function linearizeOplus(this)
            this.J{1} = [-1 0 0 0;
                0 0 -1 0];
        end        
    end
end