% This class will contain your implementation of your 2D SLAM system.

classdef DotBotSLAMSystem < minislam.slam.SLAMSystem
    
    properties(Access = public, Constant)
        % Platform state dimension
        NP = 4;
        
        % Landmark dimension
        NL = 2;
    end
    
    properties(Access = protected)
        
        % Kalman filter mean and covariance
        x;
        P;
        
        % Store of the mean and covariance values
        timeStore;
        xStore;
        PStore;
        
        % Map stores landmark ID with the indices in the state vector
        landmarkIDStateVectorMap;
        
        %% ADD FOR TASK 1
        % You may need to add code here to suppor the prediction step.
        %% END ADD FOR TASK 1
    end
    
    methods(Access = public)
    
        function this = DotBotSLAMSystem(configuration)
            this = this@minislam.slam.SLAMSystem(configuration);
            
            % Set the storage up for stuff like the history
            this.xStore = NaN(dotbot.DotBotSLAMSystem.NP, 1);
            this.PStore = NaN(dotbot.DotBotSLAMSystem.NP, 1);
            this.x = NaN(dotbot.DotBotSLAMSystem.NP, 1);
            this.P = NaN(dotbot.DotBotSLAMSystem.NP, dotbot.DotBotSLAMSystem.NP);
            
            %% ADD FOR TASK 1
            % You may need to add code here to suppor the prediction step.
            %% END ADD FOR TASK 1
            
            % Create the map which links ID type to the index array
            this.landmarkIDStateVectorMap = containers.Map('KeyType', 'int64', 'ValueType', 'any');
            
        end
        
        % Return the current platform estimate
        function [x,P] = platformEstimate(this)
            x = this.x(1:dotbot.DotBotSLAMSystem.NP);
            P = this.P(1:dotbot.DotBotSLAMSystem.NP, 1:dotbot.DotBotSLAMSystem.NP);
        end
        
        function [T, X, PX] = platformEstimateHistory(this)
            T = this.timeStore;
            X = this.xStore;
            PX = this.PStore;
        end
        
        function [x, P, landmarkIds] = landmarkEstimates(this)
            landmarkIds = keys(this.landmarkIDStateVectorMap);
            
            numberOfLandmarks = length(landmarkIds);
           
            x = NaN(dotbot.DotBotSLAMSystem.NL, numberOfLandmarks);
            P = NaN(dotbot.DotBotSLAMSystem.NL, dotbot.DotBotSLAMSystem.NL, numberOfLandmarks);
            
            for l = 1 : numberOfLandmarks
                landmarkId = landmarkIds(l);
                idx = this.landmarkIDStateVectorMap(landmarkId{1});
                x(:, l) = this.x(idx);
                P(:, :, l) = this.P(idx, idx);
            end
        end
    end
       
    methods(Access = protected)
        
        % Set the intial conditions
        function handleInitialConditionEvent(this, event)
            this.x = event.data;
            this.P = event.covariance;
        end
        
        % Handle the case that no prediction happens
        function handleNoPrediction(this)
            % Nothing to do
        end
        
        % Handle everything needed to predict to the current state
        function handlePredictToTime(this, deltaToNextTime, nextTime)
            
            %% ADD TASK 1; UPDATE FOR TASK 2
            error('Implement to complete task 1')
            % You will need to implement your code to do the Kalman filter
            % prediction here. The predicted mean should be in this.x, the
            % predicted covariance this.P.
            %% END ADD TASK 1; UPDATE FOR TASK 2
        end
 
        % Handle a GPS measurement
        function handleGPSObservationEvent(this, event)

            %% ADD TASK 1; UPDATE FOR TASK 5
            error('Implement to complete task 1')
            % You will need to add code here to update the GPS. If you
            % experiment with adding it back in as task 5, you will need to
            % modify it.
            %% END ADD TASK 1; UPDATE FOR TASK 5                       
        end
            
        % Handle a set of measurements of landmarks
        function handleLandmarkObservationEvent(this, event)
            
           % Iterate over all the landmark measurements
            for l = 1 : length(event.landmarkIds)
                
                landmarkId = event.landmarkIds(l);
                 
                % If the landmark did not exist, augment it to the state
                % and return
                if (isKey(this.landmarkIDStateVectorMap, landmarkId) == false)

                    % ADD FOR TASK 2
                    error('Implement to complete task 2')
                    % You will need to add your code here to register the
                    % new landmark ID and initialize the state estimate
                    % (mean and covariance) using the SLAM augmentation
                    % step. The updated mean will be in this.x, the
                    % covariance in this.P
                    % END ADD FOR TASK 2

                else
                    % ADD FOR TASK 3, MAYBE TASK 4
   
                    % You will need to add your code here to update a
                    % previously observed landmark with an observation. The
                    % updated state must be in this.x and this.P.
                    
                    % You might want to modify the code here as well if you
                    % are corrupting the covariance as part of Task 4.
                end
            end
        
        end
                
        function storeStepResults(this)
          % Store the estimate for the future
            this.timeStore(:, this.stepNumber) = this.currentTime;
            this.xStore(:, this.stepNumber) = this.x(1:dotbot.DotBotSLAMSystem.NP);
            this.PStore(:, this.stepNumber) = diag(this.P(1:dotbot.DotBotSLAMSystem.NP, 1:dotbot.DotBotSLAMSystem.NP));
            
            % ADD ADDITIONAL LOGGING HERE
            % If you want to do any extra analysis, such as monitoring
            % cross correlations, the code here is the place to do it.
            % END ADDITIONAL LOGGING HERE
        end
     
    end    
end