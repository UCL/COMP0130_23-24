% This edge describes the observation model
%
% z = x + w
%
% Specifically, the error is given by
%
% w = z -x
%
% Note the sign of the returned Jacobian.
%

classdef ObjectMeasurementEdge < g2o.core.BaseUnaryEdge
   
    methods(Access = public)
    
        function this = ObjectMeasurementEdge()
            this = this@g2o.core.BaseUnaryEdge(1);
        end
        
        function computeError(this)
            this.errorZ = this.z - this.edgeVertices{1}.x;
        end
        
        function linearizeOplus(this)
            this.J{1} = - 1;
        end        
    end
end