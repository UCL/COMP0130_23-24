classdef ObjectPolarMeasurementEdge < g2o.core.BaseUnaryEdge
   
    properties(Access = protected)
        % The x,y and theta of the sensor
        sensorPose;
    end
    
    methods(Access = public)
    
        function this = ObjectPolarMeasurementEdge()
            this = this@g2o.core.BaseUnaryEdge(2);
            this.sensorPose = zeros(3, 1);
        end
        
        function setSensorPose(this, sensorPose)
            this.sensorPose = sensorPose;
        end
        
        function computeError(this)
            error('Complete this for task 2.');
        end
        
        function linearizeOplus(this)
            error('Complete this for task 2.');
        end        
    end
end