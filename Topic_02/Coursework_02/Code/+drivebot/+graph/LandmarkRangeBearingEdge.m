% This edge encodes a 3D range bearing measurement.
%
% The measurement is in spherical polar coordinates

% Jacobian from https://github.com/petercorke/robotics-toolbox-matlab/blob/master/RangeBearingSensor.m

classdef LandmarkRangeBearingEdge < g2o.core.BaseBinaryEdge
    
    methods(Access = public)
    
        function this = LandmarkRangeBearingEdge()
            this = this@g2o.core.BaseBinaryEdge(2);
        end
        
        function initialize(this)
            % Q2b:
            % Complete implementation
            warning('landmarkrangebearingedge:initialize:unimplemented', ...
                'Implement the rest of this method for Q1b.');
        end
        
        function computeError(this)

            % Q2b:
            % Complete implementation
            warning('landmarkrangebearingedge:computeerror:unimplemented', ...
                'Implement the rest of this method for Q1b.');

        end
        
        function linearizeOplus(this)
            % Q2b:
            % Complete implementation
            warning('landmarkrangebearingedge:linearizeoplus:unimplemented', ...
                'Implement the rest of this method for Q1b.');
        end        
    end
end