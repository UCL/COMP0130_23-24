% An edge is used to establish a probabilistic relationship on one or more
% vertices. Specifically, the idea is that it's possible create an error
% function of the form
%
% e = h({v}, z)
%
% where {v} is the set of vertices attached to this edge, and z is a
% "measurement" (which is actually any input vector used with computing the
% error).
%
% This object is not instantianted directly. Rather, you should create a
% subclass. 
%
% To create a subclass you need to:
% 1. Specify the number of vertices (length({v})
% 2. Specify the dimension of e
% 3. Provide a function to compute the error
% 4. Provide a function which computes the Jacobians of the error functio
% with respect to the different vertices {v}.
%
% Two helper classes - BaseUnaryEdge and BaseBinaryEdge - are provided
% which have compatibility with g2o's class hierarchy.

classdef BaseEdge < g2o.core.HyperGraphElement

    properties(Access = protected)
        
        
        % The measurement associated with this edge
        z;
        
        % The information associated with this edge
        Omega;
        
        % Dimensions of the measurement
        dimZ;
        
        % The number of vertices associated with this edge
        numVertices;
        
        % The error based on the current vertex state estimates
        errorZ;
        
        % The Jacobians
        J;
        
    end
    
    properties(Access = {?g2o.core.HyperGraphElement,?g2o.core.OptimizableGraph})
        % The vertices associated with this edge
        edgeVertices;
    end
    
    methods(Access = protected)
        
        % Construct the new edge. The number of vertices and the dimensions
        % of the measurement must be specified.
        function this = BaseEdge(numVertices, measurementDimension)
            this = this@g2o.core.HyperGraphElement();
            
            assert(nargin > 1, 'g2o:baseedge:baseedge:insufficientarguments', ...
                'The number of vertices and dimensions are mandatory');
            
            % Check the number of vertices is good
           assert(numVertices > 0, 'g2o:baseedge:baseedge:invalidnumvertices', ...
                'The number of vertices must be a non-negative integer; the value is %d', numVertices);
            
            this.numVertices = numVertices;
            
            this.edgeVertices = cell(1, numVertices);
            
            % Check the dimensions are okay and store
            assert(measurementDimension > 0, 'g2o:baseedge:baseedge:invalidmeasurementdimension', ...
                'The measurement dimension must be a non-negative integer; the value is %d', measurementDimension);

            % Check the dimensions are okay and store
            this.dimZ = measurementDimension;
            
            % Allocate an initial values
            this.z = NaN(this.dimZ, 1);
            this.Omega = NaN(this.dimZ, this.dimZ);
            this.errorZ = NaN(this.dimZ, 1);
            
            this.setId(g2o.core.BaseEdge.allocateId());
            
            % Preallocate space for the Jacobians for each vertex
            this.J = cell(1, numVertices);
        end
    end
    
    methods(Access = public)
                
        % Set the value of Omega for this edge.
        function setInformation(this, newOmega)
            % Needs to be a 2D array
            newOmegaNDims = ndims(newOmega);
            assert(newOmegaNDims == 2, ...
                'g2o:baseedge:setinformation:informationwrongdimension', ...
                'The information matrix must be a two dimensional array; ndims=%d', ...
                newOmegaNDims);
            
            % Get the vector sizes
            rows = size(newOmega, 1);
            cols = size(newOmega, 2);

            % Check the size
            assert((rows == this.dimZ) && (cols == this.dimZ), ...
                'g2o:baseedge:setinformation:informationwrongdimension', ...
                ['The information matrix should be a square ' ...
                ' matrix of dimension %d; the matrix has dimensions (%d, %d)'], ...
                this.dimZ, rows, cols);
            
            % Check not NaN
            assert(any(any(isnan(newOmega))) == false, ...
                'g2o:baseedge:setinformation:informationhasnans', ...
                'The information matrix contains NaNs');
            
            % Check the matrix matrix is positive semidefinite
            assert(all(eig(newOmega) > eps), 'g2o:baseedge:setinformation:informationnotpsd', ...
                'The information matrix is not positive semidefinite');
            
            this.Omega = newOmega;
        end

        
    end
    
    methods(Access = public, Sealed = true)
        
        % Set the vertex for this edge. Different edges have different
        % numbers of vertices. The vertices are indexed by the vertex
        % number. Note that the meaning of each vertex depends on how you
        % define the error function.
        function this = setVertex(this, vertexNumber, vertex)
            
            % Cast to the right type
            vertexNumber = uint32(vertexNumber);
            
            % Check the vertex is of the right kind
            assert(isa(vertex, 'g2o.core.BaseVertex'), 'g2o:baseedge:addedge:wrongclass', ...
                'The object should inherit from base.Vertex; the object class is %s', ...
                class(vertex));
            
            % Check the vertex number if valid
            assert((vertexNumber >=1) && (vertexNumber <= this.numVertices), ...
                'g2o:baseedge:addedge:invalidvertexnumber', ...
                'The vertex number must be between 0 and %d; the value is %d', this.numVertices, ...
                vertexNumber);
            
            % Assign
            this.edgeVertices{vertexNumber} = vertex;
            
            % (Re)allocate the storage of the Jacobian for this vertex
            this.J{vertexNumber} = zeros(vertex.dimension(), this.dimZ);
            
            % Register the edge with the vertex
            vertex.addEdge(this);
        end
        
        % Remove a vertex from this edge.
        function removeVertex(this, vertex)
            
            % Check the class is correct
           assert(isa(vertex, 'g2o.core.BaseVertex'), 'g2o:baseedge:removeedge:wrongclass', ...
                'The object should inherit from base.Vertex; the object class is %s', ...
                class(vertex));
 
            % Go through all the registered vertices. If we found it,
            % remove it
            foundVertex = false;
            for v = 1 : this.numVertices
                if ((isempty(this.edgeVertices{v}) == false) && ...
                        (this.edgeVertices{v}.id() == vertex.id()))
                    this.edgeVertices{v}.removeEdge(this);
                    this.edgeVertices{v} = [];
                    foundVertex = true;
                    break;
                end
            end
            
            % The vertex wasn't found
            assert(foundVertex == true, 'g2o:baseedge:removeedge:wrongclass', ...
                'No vertex with ID %d is registered with edge with ID %d', vertex.id(), this.elementId);
        end
        
        % Number of vertices which this edge should connect to
        function numVertices = numberOfVertices(this)
            numVertices = this.numVertices;
        end
        
        % The number of vertices which have not been assigned to the edge
        % yet. A valid edge has all of its vertices assigned.
        function numUndefinedVertices = numberOfUndefinedVertices(this)
            numUndefinedVertices = sum(cellfun('isempty',this.edgeVertices));
        end
        
        % Get an individual vertex
        function v = vertex(this, vertexNumber)
            
            assert((vertexNumber > 0) && (vertexNumber <= this.numVertices), ...
                'g2o:baseedge:illegalvertexid', 'The vertexNumber %d should be between 1 and %d', ...
                vertexNumber, this.numVertices);
            v = this.edgeVertices{vertexNumber};            
        end
        
        % Get a cell array of the list of vertices
        function vertices = vertices(this)
            vertices = this.edgeVertices;
        end
        
        % Dimension of the measurement.
        function dimension = dimension(this)
            dimension = this.dimZ;
        end
        
        % Get the error.
        function errZ = error(this)
            errZ = this.errorZ;
        end
        
        % Get the Jacobians for the most recent error calculation. This is
        % a cell array. The ith element in the array is the Jacobian of the
        % error with respect to the ith vertex.
        function J = jacobianOplus(this)
            J = this.J;
        end
        
        % Return the "chi2" value. This is equal to e'*Omega*e, and is the
        % contribution of this edge to the overall cost term we are trying
        % to minimize.
        function chi2 = chi2(this)
            chi2 = this.errorZ' * (this.Omega * this.errorZ);
            assert(isnan(chi2) == false, 'g2o:baseedge:chi2:chi2isnan', ...
                'The chi2 value is NaN.');
        end
        
        % This method computes the contributions to the Hessian
        % (information matrix) and information update vector. Basically it
        % computes the individual elements of (20) and (21) of the g2o
        % documentation in the appendix.
        function [H, b] = computeHB(this)
            
            % Compute the Jacobians
            this.linearizeOplus();

            % Compute the error
            this.computeError();
            
            H = cell(this.numVertices, this.numVertices);
            b = cell(1, this.numVertices);
            
            % Work out the contribution from each vertex. Note
            % this is symmetric, so we only populate the upper triangle.
            % We also cache common terms
            for i = 1 : this.numVertices         
                b{i} = this.J{i}' * (this.Omega * this.errorZ);
                JWi = this.J{i}' * this.Omega;
                H{i, i} = JWi * this.J{i};
                for j = i + 1 : this.numVertices
                    H{i, j} = JWi * this.J{j};
                end
            end
        end
        
        % Set the measurement value.
        function setMeasurement(this, newZ)
            
            % Needs to be a 2D array
            newZNDims = ndims(newZ);
            assert(newZNDims == 2, ...
                'g2o:baseedge:setmeasurement:measurementwrongdimension', ...
                'The measurement vector must be a column vector; ndims=%d', ...
                newZNDims);
            
            % Get the vector sizes
            rows = size(newZ, 1);
            cols = size(newZ, 2);

            % Check number of rows
            assert(cols == 1, ...
                'g2o:baseedge:setmeasurement:measurementwrongdimension', ...
                'The measurement vector must be a column vector; columns=%d', ...
                cols);
            
            % Check number of columns
            assert(rows == this.dimension, ...
                'g2o:baseedge:setmeasurement:measurementwrongdimension', ...
                'The measurement dimension is wrong; required=%d, actual=%d', ...
                this.dimZ, rows);
            
            % Check not NaN
            assert(any(isnan(newZ)) == false, ...
                'g2o:baseedge:setmeasurement:measurementhasnans', ...
                'The measurement contains NaNs');
            this.z = newZ;

        end
        
        % Return the measurement value.
        function z = measurement(this)
           z = this.z; 
        end
        
        % Return the value of omega for this edge.
        function Omega = information(this)
            Omega = this.Omega;
        end
        
        % Make sure the edge is set up properly: the measurement is
        % defined, the information is defined (and is positive
        % semidefinite) and all the vertices have been assigned.
        function validate(this)
            
            if (this.validated == true)
                return;
            end

            % If any edges are undefined, then generate a warning and
            % return
            assert(this.numberOfUndefinedVertices() == 0, ...
                'g2o:baseedge:validate:undefinedvertices', ...
                'this.numberOfUndefinedVertices()=%d for the edge with id %d', ...
                this.numberOfUndefinedVertices(), this.id);
            
            % Check the measurement is not nan
            assert(any(isnan(this.z)) == false, ...
                'g2o:baseedge:validate:measurementhasnans', ...
                'The measurement contains NaNs');
            
            assert(any(any(isnan(this.Omega))) == false, ...
                'g2o:baseedge:validate:informationhasnans', ...
                'The information matrix contains NaNs');

            assert(all(eig(this.Omega) > eps), 'g2o:baseedge:validate:informationnotpsd', ...
                'The information matrix is not positive semidefinite');
            
            % Check all the vertices and make sure they are registered.
            for v = 1 : this.numVertices
                assert(this.edgeVertices{v}.owningGraph == this.owningGraph, ...
                    'g2o:baseedge:validate:vertcesnotregistered', ...
                    'vertex with ID %d is not registered with a graph', ...
                    this.edgeVertices{v}.elementId);
            end
            
            this.validated = true;
        end    
        
    end
    
    methods(Access = public, Abstract)
        
        % This method takes the current estimates from the edge's vertices
        % and uses them to assign a value to this.errorZ.
        computeError(this);
        
        % This method computes the cell array of Jacobians which is
        % returned by jacobianOPlus.
        linearizeOplus(this);        
    end

    
     methods(Access = protected, Static)
       
        % This private method ensures that each vertex has a unique ID.
        % This is used for map storage / search internally.
        function id = allocateId()
            persistent idCount;
            
            if (isempty(idCount))
                idCount = 0;
            else
                idCount = idCount + 1;
                
            end
            id = idCount;
            
        end
        
    end
end

