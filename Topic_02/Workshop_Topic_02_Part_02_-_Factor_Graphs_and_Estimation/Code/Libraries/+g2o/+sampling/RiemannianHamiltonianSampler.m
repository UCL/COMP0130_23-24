classdef RiemannianHamiltonianSampler < g2o.sampling.HamiltonianSampler
   
    % This applies the RHMC sampler to a graph. The main assumption is that
    % the information matrices on each edge do not depend on the state.
    
    properties(Access = protected)
        
        % The graph we are sampling from
        graph;
        
    end
    
    methods(Access = public)
       
        function this = RiemannianHamiltonianSampler(graph)
           
            % Check the graph is of the right kind
            assert(isa(graph, 'g2o.core.SparseOptimizer'), 'graphbasedsampler:graphbasedsampler:wrongclass', ...
                'The object should be of class g2o.core.SparseOptimizer; the object class is %s', ...
                class(graph));
            
            % Run base constructor
            this = this@g2o.sampling.HamiltonianSampler();
            
            this.graph = graph;
            
        end
        
    end
    
    methods(Access = protected)
       
        % Compute the likelihood from the graph
        function V = computeV(this)
            this.graph.assignXToVertices(this.theta);
            V = 0.5 * this.graph.chi2();
        end
        
        % Compute the mass matrix. This is identical to the Hessian.
        function computeM(this)
            this.graph.assignXToVertices(this.theta);
            [~, this.M] =  this.graph.computeHessian();
        end
        
        function dHDTheta = computeDHDTheta(this)
            this.graph.assignXToVertices(this.theta);
            [dHDTheta, ~] = this.graph.computeHessian();
        end        
    end
end