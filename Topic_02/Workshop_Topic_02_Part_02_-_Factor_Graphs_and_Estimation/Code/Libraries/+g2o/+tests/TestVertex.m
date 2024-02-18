classdef TestVertex < g2o.BaseVertex

    methods(Access = public)
        
        function this = TestVertex()
            this = this@g2o.BaseVertex(2);
        end
        
    end
    
end
