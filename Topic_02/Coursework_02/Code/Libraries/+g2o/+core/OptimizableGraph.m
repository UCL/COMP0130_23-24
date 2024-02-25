% This class provides an interface ontop of the basic hypergraph to support
% setting up and optimizing the graph. It does not, however, actually
% implement any optimization algorithm. This is an internal class and
% shouldn't be instantiated directly.

classdef OptimizableGraph < g2o.core.HyperGraph
    
    properties(Access = protected)
        
        % Flag shows if initialization is required
        initializationRequired;
        
        % The state vector
        X;
        
        % The non-fixed vertices
        numNonFixedVertices;
        nonFixedVertices;       
    end
    
    methods(Access = public)

        function this = OptimizableGraph()
            
            this = this@g2o.core.HyperGraph();
            
            this.initializationRequired = false;
            
        end
        
        function this = addVertex(this, vertex)
            
            % Call the method in the base class
            addVertex@g2o.core.HyperGraph(this, vertex);
            
            % Flag initialization required
            this.initializationRequired = true;
            
        end
 
        function this = removeVertex(this, vertex)
            
            % Call the method in the base class
            removeVertex@g2o.core.HyperGraph(this, vertex);
            
            % Flag initialization required
            this.initializationRequired = true;
            
        end
        
        function this = addEdge(this, edge)
            
            % Call the method in the base class
            addEdge@g2o.core.HyperGraph(this, edge);
            
            % Flag initialization required
            this.initializationRequired = true;
            
        end
 
        function this = removeEdge(this, edge)
            
            % Call the method in the base class
            removeEdge@g2o.core.HyperGraph(this, edge);
            
            % Flag initialization required
            this.initializationRequired = true;
            
        end

        function this = initializeOptimization(this, validateGraph)
            if ((nargin == 2) && (validateGraph == true))
                this.validateGraph();
            end
            this.buildStructure();
            this.computeInitialErrors();
            this.initializationRequired = false;
        end
        
        function numIterations = optimize(this, maximumNumberOfIterations)
            
            assert(this.initializationRequired == false, ...
                'g2o:optimizablegraph:initializationrequired', ...
                'The graph structure has changed; call initializeOptimization before optimizing');
            
            % Default number of iterations
            if (nargin == 1)
                maximumNumberOfIterations = 50;
            end
            
            numIterations = this.runOptimization(maximumNumberOfIterations);
        end
    
        % Get the chi2 value for the overall graph. This is the cost.
        % If specified, also return a list of each chi2 value for each edge
        % separately.
        function [chi2Sum, chi2List] = chi2(this)
            
            chi2Sum = 0;

            edgesToCheck = values(this.edgesMap);
            numberOfEdges = length(edgesToCheck);

            if (nargout == 2)
                chi2List = zeros(1, numberOfEdges);
            end
            
            for e = 1 : length(edgesToCheck)
                edgesToCheck{e}.computeError();
                chi2Value = edgesToCheck{e}.chi2();
                chi2Sum = chi2Sum + chi2Value;
                if (nargout == 2)
                    chi2List(e) = chi2Value;
                end                
            end            
        end
    end
    
    methods(Access = protected)
       
        % Make sure that all the vertices and edges are set up correctly
        function validateGraph(this)
            
            % Make sure the progress bar is reset
            textprogressbar();

            % The sets to check
            verticesToCheck = values(this.verticesMap);
            edgesToCheck = values(this.edgesMap);
            
            % First clear the valid state on all vertices and edges
            for v = 1 : length(verticesToCheck)
                verticesToCheck{v}.clearValidated();
            end
            for e = 1 : length(edgesToCheck)
                edgesToCheck{e}.clearValidated();
            end
            
            % Check all the vertices are valid
            textprogressbar('validating vertices: ');
            for v = 1 : length(verticesToCheck)
                textprogressbar(100*v/length(verticesToCheck));
                verticesToCheck{v}.validate();
            end
            textprogressbar(' completed');

            % Check all the edges are valid
            textprogressbar('validating edges   : ');
            for e = 1 : length(edgesToCheck)
                textprogressbar(100*e/length(edgesToCheck));
                edgesToCheck{e}.validate();
            end
            textprogressbar(' completed');
        end
        
        % Internally process the structure and pre-cache results where
        % possible.
        function buildStructure(this)
            
            % First accumulate all the states into a big vector
            currentIdx = 0;
            
            this.X = [];
            
            % Go through each vertex. Identify the non-fixed ones. Store
            % them in the big X matrix and assign the index in the big X
            % map
            verticesToCheck = values(this.verticesMap);
            this.nonFixedVertices = cell(length(verticesToCheck), 1);
            this.numNonFixedVertices = 0;
            for v = 1 : length(verticesToCheck)
                vertex = verticesToCheck{v};
                if (vertex.conditioned == true)
                    vertex.iX = [];
                    continue
                end
                vertexDim = vertex.dimension();
                idx = currentIdx + (1:vertexDim);
                currentIdx = idx(end);
                vertex.iX = idx;
                this.X(idx, 1) = vertex.estimate();
                this.numNonFixedVertices = this.numNonFixedVertices + 1;
                this.nonFixedVertices{this.numNonFixedVertices} = vertex;
            end
        end
        
        function computeInitialErrors(this)
            edgesToCheck = values(this.edgesMap);            
            for e = 1 : length(edgesToCheck)
                edgesToCheck{e}.computeError();
            end
        end
    end
    
    methods(Access = protected, Abstract)
        numberOfIterations = runOptimization(this, maximumNumberOfIterations);
    end
    
end