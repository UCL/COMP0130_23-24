% The object state vertex for the static example. In this case, the state
% is just the object position x.
%
% Note the state here is linear, and so we can use the default oplus update
% method.

classdef ObjectStateVertex < g2o.core.BaseVertex
   
    methods(Access = public)
        function this = ObjectStateVertex()
            this=this@g2o.core.BaseVertex(1);
        end
    end
    
end