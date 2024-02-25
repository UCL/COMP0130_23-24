classdef InitialPriorEdge < g2o.core.BaseUnaryEdge
   
    % This edge stores the initial conditions
    
    methods(Access = public)
    
        function this = InitialPriorEdge()
            this = this@g2o.core.BaseUnaryEdge(3);
        end
        
        function computeError(this)
            this.errorZ = this.edgeVertices{1}.x - this.z;
            this.errorZ(3) = g2o.stuff.normalize_theta(this.errorZ(3));
        end
        
        function linearizeOplus(this)
            this.J{1} = eye(3);
        end        
    end
end