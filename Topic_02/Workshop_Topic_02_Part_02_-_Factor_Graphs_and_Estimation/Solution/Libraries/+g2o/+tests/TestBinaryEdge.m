classdef TestBinaryEdge < g2o.core.BaseBinaryEdge
    
    methods(Access = public)
   
        function this = TestBinaryEdge()
            this = this@g2o.core.BaseBinaryEdge(2);
        end
        
        function linearizeOplus(~)
        end
        
        function computeError(~)
        end
        
    end
end