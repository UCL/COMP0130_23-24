classdef ObjectMeasurementEdge < g2o.core.BaseUnaryEdge
   
    methods(Access = public)
    
        function this = ObjectMeasurementEdge()
            this = this@g2o.core.BaseUnaryEdge(2);
        end
        
        function computeError(this)
            error('Complete this for task 2.');
        end
        
        function linearizeOplus(this)
            error('Complete this for task 2.');
        end        
    end
end