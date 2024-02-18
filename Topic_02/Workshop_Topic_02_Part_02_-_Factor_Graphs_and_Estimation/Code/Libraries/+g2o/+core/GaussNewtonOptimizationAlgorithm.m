classdef GaussNewtonOptimizationAlgorithm < g2o.core.OptimizationAlgorithm
    
    properties (Access = private)
        tol = 1e-5
    end
    
    methods(Access = public)
        
        function this = GaussNewtonOptimizationAlgorithm(tol)
            this = this@g2o.core.OptimizationAlgorithm();
            if nargin > 0
                this.tol = tol;
            end
        end
        
        function [X, numberOfIterations] = solve(this, X0, maximumNumberOfIterations)
            
            tic;
            
            % Set the graph to the initial condition. Might be redundant
            this.optimizableGraph.assignXToVertices(X0);
            
            % Get the cost of the initial solution, and check it's okay
            R0 = this.optimizableGraph.chi2();
            
            assert(isnan(R0) == false, ...
                'g2o:core:lsqroptimizationalgorith:chi2isnan', ...
                'The chi2 value from the optimizable graph is NaN.');
            
            % Set the current best solution
            X = X0;
            
            % Set the number of iterations
            numberOfIterations = 0;
            
            % We have to check that R0 isn't too small otherwise 
            
            while ((numberOfIterations < maximumNumberOfIterations))
                numberOfIterations = numberOfIterations + 1;
                
                fprintf('Iteration = %03d; Residual = %6.3f; Time = %3.3f\n', ...
                    numberOfIterations, R0, toc); 
                
                % Compute the problem
                [H,b] = this.optimizableGraph.computeHB(X);
                
                % Work out the update step
                dX = H\b;
                
                % Check not NaN
                assert(any(isnan(dX)) == false, ...
                    'g2o:core:gaussnewtonoptimizationalgorith:dXisnan', ...
                    'The step direction vector contains NaNs');
                
                % The scale of the step length
                alpha = -1;
                
                % First take a step and compute what the new cost is
                XadX = this.optimizableGraph.assignXToVerticesWithPerturbation(X, alpha * dX);
                R1 = this.optimizableGraph.chi2();
                
                assert(isnan(R1) == false, ...
                    'g2o:core:gaussnewtonoptimizationalgorith:chi2isnan', ...
                    'The chi2 value from the optimizable graph is NaN.');
                
                % If the error increased, then reduce the step length and
                % try again
                while ((R1 > R0) && (abs(alpha) > 1e-10))
                    alpha = alpha * 0.5;
                    XadX = this.optimizableGraph.assignXToVerticesWithPerturbation(X, alpha * dX);
                    R1 = this.optimizableGraph.chi2();
                end
                
                X = XadX;

                % If this condition fails, this captures the cases that
                % either (a) the cost didn't change very much or (b) the
                % cost actually went up
                if ((R0-R1) < this.tol)
                    break;
                end
                
                % Store the new best cost and solution
                R0 = R1;
            end
        end
    end
    
    methods(Access = protected)
        
    end
end