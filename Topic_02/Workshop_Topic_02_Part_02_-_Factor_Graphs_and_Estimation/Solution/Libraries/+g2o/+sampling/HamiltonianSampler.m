classdef HamiltonianSampler < handle
           
    % This is an attempt to reimplement Richard's code in a more object
    % oriented way to help me understand the structure and generalise a bit
    % for some other experiments.
    
    properties(Access = protected)

        % Step size
        epsilon;
        
        % Number of Hamilton steps
        nSteps;
        
        % Enable Metropolis rejection test
        useRejectionTest;
        
        % The observation and observation covariance
        y;
        R;
        
        % The mass matrix
        M;
        
        % State and momentum and dimensions
        theta;
        p;
        ndims;

        % Number of steps used in leapfrog integrator
        newtonSteps;        
    end

    methods(Access = public)
        
        function this = HamiltonianSampler()
            this.nSteps = 35;
            this.epsilon = 0.01;
            this.useRejectionTest = true;
        end
        
        function setParameters(this, nSteps, epsilon)
            this.nSteps = nSteps;
            this.epsilon = epsilon;
        end
        
        % Sample the state starting from theta0 for an observation y
        % If p0 is provided, use it at the initial momentum. Otherwise,
        % it is sampled.
        
        function [theta, acceptMove, HTrajectory, reverseTrajectory] = sample(this, y, theta0, p0)
            
            % Store the state and the observation
            this.y = y;
            this.theta = theta0;
            this.ndims = length(this.theta);
            
            % The history of the Hamiltonians, used for debugging
            storeHTrajectory = (nargout > 2);
            if (storeHTrajectory == true)
                HTrajectory = NaN(1, this.nSteps + 1);
            end
            
            % Measure to detect sample reversal - simple test to suggest
            % samples "folding back on themselves" so the possibility of
            % the NUTS sampler
            storeReverse = (nargout > 3);
            if (storeReverse == true)
                reverseTrajectory = NaN(2, this.nSteps + 1);
            end
            
            % Choose the initial mass matrix
            this.computeM();
            
            % Choose initial value of the momentum if it's not provided
            if (nargin == 4)
                this.p = p0;
            else
                this.p = this.sampleP();
            end
            
            % Work out the initial Hamiltonian. This is needed for the accept
            % / reject step at the end
            H0 = this.computeH();
            
            % Store the initial Hamiltonian if required
            if (storeHTrajectory == true)
                HTrajectory(1) = H0;
            end
            
            % Store the initial reverse value if required
            if (storeHTrajectory == true)
                reverseTrajectory(:, 1) = 0;
            end
            
            % Use the leapfrog integrator
            keepRunning = true;
            s = 1;
            
            while ((s <= this.nSteps) && (keepRunning == true))
                % Do the leapfrog integration using (23a)-(23c)
                this.p = this.p - 0.5 * this.epsilon * this.computeDHDTheta();
                this.theta = this.theta + this.epsilon * this.computeDHDP();
                this.p = this.p - 0.5 * this.epsilon * this.computeDHDTheta();

                % Update the mass matrix; this is needed for computeDHDP.
                % It is also used in the accept / reject step so it isn't
                % wasted.
                this.computeM();
                
                % Bump the step count
                s = s + 1;
    
                % If storing the Hamiltonian, compute the value and save it
                if (storeHTrajectory == true)
                    HTrajectory(s) = this.computeH();
                end
                
                % If storing the reverse measure, compute the value and
                % save it
                if (storeReverse == true)
                    reverseTrajectory(1, s) = dot(this.theta - theta0, this.p);
                    reverseTrajectory(2, s) = reverseTrajectory(1, s)/(norm(this.theta - theta0)*norm(this.p));
                 end
                
                % Test for early termination goes here
                keepRunning = true;
            end
            
            % Work out the final Hamiltonian
            H = this.computeH();
            
            % Work out the energy
            deltaH = H0 - H;
            
            % Do the sample rejection step
            %acceptMove = (this.useRejectionTest == false) || (log(rand(1)) < deltaH);
            acceptMove = (this.useRejectionTest == false) || (rand(1) < exp(deltaH));
            
            if (acceptMove == true)
                theta = this.theta;
            else
                theta = theta0;
            end
        end
    end
    
    methods(Access = protected)
        
        % Sample the initial momentum
        function p0 = sampleP(this)
            p0 = chol(this.M)' * randn(this.ndims, 1);
        end
    
        % Compute the Hamiltonian
        function H = computeH(this)
            H = this.computeV() + this.computeT();
        end
        
        % Compute the potential energy. Assuming a Gaussian, we use the
        % function to convert theta to an observation, and then compute the
        % deltas.
        function V = computeV(this)
            nu = this.computeNu();
            V = 0.5 * nu' * (this.R \ nu);
        end
        
        % Compute the kinetic energy. Assume this is Gaussian distributed
        % and is zero-mean with covariance equal to the mass matrix.
        function T = computeT(this)
            T = 0.5 * this.p' * (this.M \ this.p);
        end
        
        % Compute the Hamiltonian with respect to momentum
        function dHdP = computeDHDP(this)
            dHdP = this.M \ this.p;
        end
        
        % The mass matrix; the default is that it's constant
        function computeM(this)
            this.M = eye(this.ndims);
        end
        
    end
    
    methods(Access = protected, Abstract)
        
        dHDTheta = computeDHDTheta(this);
    end    
end