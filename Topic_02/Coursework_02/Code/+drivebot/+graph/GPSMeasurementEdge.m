classdef GPSMeasurementEdge < g2o.core.BaseUnaryEdge
   
    properties(Access = protected)
        
        xyOffset;
        
    end
    
    methods(Access = public)
    
        function this = GPSMeasurementEdge(xyOffset)
            this = this@g2o.core.BaseUnaryEdge(2);
            this.xyOffset = xyOffset;
        end
        
        function computeError(this)

	    % Q1d:
        % Implement the code
        warning('gpsmeasurementedge:computeerror:unimplemented', ...
                'Implement the rest of this method for Q1d.');
        end
        
        function linearizeOplus(this)

	    % Q1d:
        % Implement the code
        warning('gpsmeasurementedge:lineareizeoplus:unimplemented', ...
                'Implement the rest of this method for Q1d.');
        end
    end
end
