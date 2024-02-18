classdef LevenbergMarquardtOptimizationAlgorithm < g2o.core.OptimizationAlgorithm
    
    % Implement the LM optimization algorithm from the pseudocode in the
    % Figure 2. of the paper "SBA: A Software Package for Generic Sparse Bundle Adjustment"
    % by Lourakis et al.
    % https://www.researchgate.net/publication/220492985_SBA_A_Software_Package_for_Generic_Sparse_Bundle_Adjustment
    
    properties(Access = protected)
        
        % Some paramters
        tau;
        
        % Various error bounds
        e1;
        e2;
        e3;
        e4;
    end
    
    methods(Access = public)
        
        function this = LevenbergMarquardtOptimizationAlgorithm()
            this = this@g2o.core.OptimizationAlgorithm();
            
            % Set up values using the numbers from the Lourakis paper
            this.tau = 1e-3;
            this.e1 = 1e-12;
            this.e2 = 1e-12;
            this.e3 = 1e-12;
            this.e4 = 0;
        end
        
        function [X, k] = solve(this, X0, kmax)
            
            k = 1;
            nu = 2;
            tic
            
            n = length(X0);
            X = X0;

            % Set the graph to the initial condition. Might be redundant
            this.optimizableGraph.assignXToVertices(X);
            
            % Get the cost of the initial solution, and check it's okay
            R0 = this.optimizableGraph.chi2();

            % Compute the stats of the problem; just used to work out the
            % starting stepsize, H=J'*J, b=J'*ep
            [H, b] = this.optimizableGraph.computeHB(X);
            
            % If the magnitude of b is too small, return
            if (norm(b, 'inf') < this.e1)
                return
            end
            
            stop = false;
            
            % Compute the initial step size
            mu = this.computeInitialMu(H);            
            
            while ((stop == false) && (k <= kmax))
                fprintf('Iteration = %03d; Residual = %6.3f; Time = %3.3f\n', k, R0, toc);                

                rho = -Inf;
                
                while ((rho < 0) && (stop == false))

                    % Compute the damped step
                    dX = (H+mu*speye(n))\b;

                    % Termination condition
                    if (norm(dX) < this.e2 * (norm(X) + this.e2))
                        stop = true;
                        break;
                    end
               
                    % Work out the state and the cost with this step size
                    XdX = this.optimizableGraph.assignXToVerticesWithPerturbation(X, -dX);
                    R1 = this.optimizableGraph.chi2();
                
                    % Work out the gain ratio
                    rho = (R0 - R1) / (dX'*(mu*dX+b));

                    % If rho is positive, the step was accepted
                    if (rho > 0)
                        stop = ((sqrt(R0) - sqrt(R1)) < (this.e4 * sqrt(R0)));
                        
                        % Accept the new state
                        X = XdX;
                        R0 = R1;
                       
                        % Compute the information matrices; note that we do this
                        % each time with X. This accounts for the effects of both
                        % updated and rejected steps.
                        [H,b] = this.optimizableGraph.computeHB(X);

                        stop = stop || (norm(b, 'inf') < this.e1);
                        
                        % Rescale the step
                        mu = mu * max(1/3,1-(2*rho-1)^3);
                        nu = 2;
                    else
                        mu = mu * nu;
                        nu = nu * 2;
                    end
                end
                stop = stop | (sqrt(R0) < this.e3);
                k = k + 1;
            end
        end
    end
        
    methods (Access = private)
        
        function mu0 = computeInitialMu(this, H)
            mu0 = this.tau * max(diag(H));
        end

    end
        
        
end