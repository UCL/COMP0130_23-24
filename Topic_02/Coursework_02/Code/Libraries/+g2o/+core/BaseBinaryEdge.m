% This class specializes a BaseEdge to the case that it plugs into two
% vertices.

classdef BaseBinaryEdge < g2o.core.BaseEdge

    methods(Access = protected)
   
        function this = BaseBinaryEdge(measurementDimension)
            this = this@g2o.core.BaseEdge(2, measurementDimension);
        end
    end
end
