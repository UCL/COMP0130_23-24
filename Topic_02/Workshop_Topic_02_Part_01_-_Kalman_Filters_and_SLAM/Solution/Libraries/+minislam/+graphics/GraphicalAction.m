% This class defines an action which gets fired after the graphics have
% been drawn

classdef GraphicalAction < handle
   
    methods(Access = public, Abstract = true)
        
        invoke(this);
        
    end
    
end