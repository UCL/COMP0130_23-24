classdef GPSMeasurementEdge < g2o.core.BaseUnaryEdge
   
    methods(Access = public)
    
        function this = GPSMeasurementEdge()
            this = this@g2o.core.BaseUnaryEdge(2);
        end
        
        function computeError(this)
            error('Implement this');
        end
        
        function linearizeOplus(this)
            error('Implement this');
        end        
    end
end