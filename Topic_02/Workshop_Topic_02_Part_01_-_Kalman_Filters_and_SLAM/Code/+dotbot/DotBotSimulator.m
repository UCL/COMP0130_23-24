classdef DotBotSimulator < minislam.event_generators.EventGenerator
    
        properties(Access = protected)
            
        % The current simulation time
        currentTime;
        
        % The state of the platform (x, x_dot, y, y_dot)
        x;
        
        % The store of the ground truth dotbot state
        xTrueStore;
        
        % The time of the ground truth states
        timeStore;
        
        % The set of landmarks
        landmarks;
        
        % Index to the waypoint
        waypointIndex;
        
        % Flag indicates we're done
        carryOnRunning;
        
        % Time of next GPS event
        nextGPSTime;
        
        % Time of next range-bearing sensor event
        nextLandmarkSensorTime;
        
        % Debug. Set to 0 to disable noise.
        noiseScale = 1;
        
        % Flag to show if starting
        initialConditionsSent;
        
        % The continuous time system equations
        Fc;
        Qc;
        lastDT;
        
        % Discrete time system equations
        dT;
        Fd;
        Qd;
    end
    
    methods(Access = public)
        
        function this = DotBotSimulator(configuration)
            
            this = this@minislam.event_generators.EventGenerator(configuration);
            
            if (configuration.perturbWithNoise == true)
                this.noiseScale = 1;
            else
                this.noiseScale = 0;
            end
        end
        
        % Start the simulator
        function start(this)
           
            conf = this.configuration;
            
            % Set time to zero
            this.currentTime = 0;
            
            % Set up the continuous time process model
            this.Fc=[0 1 0 0;
                -conf.alpha -conf.beta 0 0;
                0 0 0 1;
                0 0 -conf.alpha -conf.beta];

            this.Qc = [0 0 0 0;
                       0 1 0 0;
                       0 0 0 0;
                       0 0 0 1] * conf.sigmaQ;
                   
            this.lastDT = NaN;
            
            % Set up the landmarks; right now these are allocated randomly
            this.landmarks = 2 * conf.extent * (rand(2, conf.numberOfLandmarks) - 0.5);
            
            % Start at the origin with zero speed
            this.x = [0, 0, 0, 0]';
            
            % Clear the event queue
            this.mostRecentEvents = minislam.event_generators.OrderedEventQueue();
            
            % Set the sensor start times. If a sensor is disabled, make
            % this inf to push it off forever.
            if (conf.enableGPS == true)
               this.nextGPSTime = this.currentTime + rand * this.configuration.gpsMeasurementPeriod;
            else
                this.nextGPSTime = inf;
            end
            
            if (conf.enableLandmarkSensor == true)
                this.nextLandmarkSensorTime = this.currentTime + rand * this.configuration.landmarkSensorMeasurementPeriod;
            else
                this.nextLandmarkSensorTime = inf;
            end
            
            % Preallocate space to store information
            this.xTrueStore = NaN(4, conf.maximumStepNumber);
            this.timeStore = NaN(1, conf.maximumStepNumber);
            
            this.xTrueStore(:, 1) = this.x;
            this.timeStore(1) = this.currentTime;
            
            this.stepNumber = 0;
            
            this.carryOnRunning = true;
            
        end
        
        % Return whether the simulator has finished
        function carryOn =  keepRunning(this)
            carryOn = ((this.carryOnRunning) && (this.stepNumber < this.configuration.maximumStepNumber));
        end
        
        function T = time(this)
            T = this.currentTime;
        end
        
        function x = currentState(this)
            x = this.x;
        end
        
        function [T, X] = platformHistory(this)
            T = this.timeStore;
            X = this.xTrueStore;
        end
        
        
        % Step the simulator and return events
        function step(this)

            % Bump the step number
            this.stepNumber = this.stepNumber + 1;
            
            % If we are just doing the initial conditions, set up and
            % return
            if (this.stepNumber == 1)
                initialConditionEvent = minislam.event_types.InitialConditionEvent(this.currentTime, ...
                        this.x, zeros(4));
                this.mostRecentEvents.insert(initialConditionEvent);
                return
            end

            % Clear any pending events
            this.mostRecentEvents.clear();
            
            % Figure out when the next sensor timestep will be
            [nextTime, idx] = min([this.currentTime + this.configuration.DT, this.nextGPSTime, ...
                this.nextLandmarkSensorTime]);
            dT = nextTime - this.currentTime;
           
            % If dT is too small, just skip the prediction this time
            if (dT > 1e-3)
                % First generate the discrete time model if we need to
                if (this.lastDT ~= dT)
                    [this.Fd, this.Qd] = minislam.utils.continuousToDiscrete(this.Fc, this.Qc, dT);
                    this.lastDT = dT;
                end

                % Update the platform state using the discrete time process
                % model
                this.x = this.Fd * this.x + sqrtm(this.Qd) * randn(4, 1);
            end

            this.currentTime = nextTime;
            
            % Generate the event based on type
            switch(idx)
                
                case 1
                    event = this.simulateHeartbeatEvents();
                    
                case 2
                    event = this.simulateGPSEvents();
                
                case 3
                    event = this.simulateLandmarkDetectorEvents();
            end
            
            this.mostRecentEvents.insert(event);
            
            this.xTrueStore(:, this.stepNumber) = this.x;
            this.timeStore(this.stepNumber) = this.currentTime;

        end
        
        % Get the ground truth state of the vehicle; not available in the
        % real world (alas)
        function groundTruthState = groundTruth(this, getFullStateInformation)
            
            if (nargin == 1)
                getFullStateInformation = true;
            end
            
            groundTruthState = minislam.event_generators.simulation.SimulatorState();
            
            % Required information
            groundTruthState.currentTime = this.currentTime;
            groundTruthState.xTrue = this.x;
            
            % Optional information
            if (getFullStateInformation == true)
                groundTruthState.mTrue = this.landmarks;
            end
        end
    end
            
    
    methods(Access = protected)

                
        function heartbeatEvents = simulateHeartbeatEvents(this)
            heartbeatEvents = {minislam.event_types.HeartbeatEvent(this.currentTime)};
        end
        
        function gpsEvents = simulateGPSEvents(this)
            
            if (this.configuration.enableGPS == false)
                gpsEvents = {};
                return
            end
                
            this.nextGPSTime = this.nextGPSTime + this.configuration.gpsMeasurementPeriod;

            gpsMeasurement = [this.x(1); this.x(3)] + this.noiseScale * sqrtm(this.configuration.RGPS) * randn(2, 1);
            
            gpsEvents = {minislam.event_types.GPSObservationEvent(this.currentTime, gpsMeasurement, ...
                this.configuration.RGPS)};
        end
            
        function laserEvents = simulateLandmarkDetectorEvents(this)
            
            laserEvents = {};
            if (this.configuration.enableLandmarkSensor== false)
                return
            end
            
            this.nextLandmarkSensorTime = this.nextLandmarkSensorTime + ...
                this.configuration.landmarkSensorMeasurementPeriod;
            
            % Find the landmarks which are in range
            
            % Work out the relative distance to all the robots
            dX = this.landmarks;
            dX(1, :) = dX(1, :) - this.x(1);
            dX(2, :) = dX(2, :) - this.x(3);
            
            % Squared range to each landmark
            R2 = sum(dX.^2,1);
            R = sqrt(R2);
            
            ids = find(R <= this.configuration.landmarkSensorDetectionRange);
            
            % If nothing to see, return
            if (isempty(ids))
                return
            end
            
            numLandmarks = length(ids);
            
            % Create observations; take the delta values and add the noise
            dXn = dX(:, ids) + this.noiseScale * sqrtm(this.configuration.RLandMarkSensor) * randn(2, numLandmarks);          
            % Package into a single event
            laserEvents = {minislam.event_types.LandmarkObservationEvent(this.currentTime, ...
                dXn, this.configuration.RLandMarkSensor, ids)};
        end

    end
end