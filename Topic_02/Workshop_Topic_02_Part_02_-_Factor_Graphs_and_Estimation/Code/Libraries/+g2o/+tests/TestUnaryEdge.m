classdef TestEdge < g2o.BaseUnaryEdge
    
    methods(Access = public)
   
        function this = TestEdge()
            this = this@g2o.BaseUnaryEdge(2);
        end
        
    end
    
end