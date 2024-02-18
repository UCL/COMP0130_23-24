% The object state vertex:
% x(1) - x
% x(2) - dot_x
% x(3) - y
% x(4) - dot_y

classdef ObjectStateVertex < g2o.core.BaseVertex
   
    methods(Access = public)
        function this = ObjectStateVertex()
            this=this@g2o.core.BaseVertex(4);
        end
    end
    
end