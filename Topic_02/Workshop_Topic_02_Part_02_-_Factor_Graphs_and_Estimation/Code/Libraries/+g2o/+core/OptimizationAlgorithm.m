classdef OptimizationAlgorithm < handle
   
    properties(Access = protected)
       
        % The graph that we connect to
        optimizableGraph;
        
    end
    
    methods(Access = protected)
       
        function this = OptimizationAlgorithm()
            
            
        end
        
    end
    
    methods(Access = public)
        
        function init(this)
        end
        
        function [X, numberOfIterations] = solve(this, X0, maximumNumberOfIterations)
            X=[];
            numberOfIterations = -1;
        end
        
    end
    
    
    methods(Access = public, Sealed)

        function optimizableGraph = graph(this)
            optimizableGraph = this.optimizableGraph;
        end
        
    end
    
    methods(Access =  {?g2o.core.SparseOptimizer})
        
        function setGraph(this, optimizableGraph)
            % Check of the correct type
            assert(isa(optimizableGraph, 'g2o.core.OptimizableGraph') == true, ...
                'g2o:optimizationalgorithm:graphwrongtype', ...
                [ 'The graph should be of class g2o.OptimizableGraph' ...
                'the provided graph is of class %s'], class(optimizableGraph));
            
            this.optimizableGraph = optimizableGraph;   
        end
        
    end
    
end