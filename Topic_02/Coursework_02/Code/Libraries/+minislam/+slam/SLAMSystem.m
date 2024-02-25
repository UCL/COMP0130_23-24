% This class implements an event-based estimation system for building up a
% minimal, ideal SLAM system. The system is event-based and responds to a
% sequence of events which are time stamped and served in order.

classdef SLAMSystem < handle
        
    properties(Access = protected)
        
        % Configuration information
        configuration;
        
        % Step number - how many times have we iterated?
        stepNumber;
        
        % The last time an event was processed.
        currentTime;
                
        % Flag to show if debugging is enabled
        debug;
        
        % Flag to show if the system has been initialized or not
        initialized;
        
        % The vehicle control inputs. These are wheel speed and steer
        % angle. These are assumed to be held constant until the next
        % control input event comes along.
        u;
        uCov;
        
        % Flag to run the detailed graph validation checks
        validateGraphOnInitialization;
        
        % The graph used for performing estimation.
        graph;
        
        % The optimization algorithm
        optimizationAlgorithm;

    end
       
    methods(Access = public)
              
        % Get the underlying g2o graph
        function graph = optimizer(this)
            graph = this.graph;
        end

        % Set the flag to enable / disable validation. This spots errors in
        % graph construction, but repeated calls can slow things down by
        % 20% or more.
        function setValidateGraph(this, validateGraphOnInitialization)
            this.validateGraphOnInitialization = validateGraphOnInitialization;
        end
        
        function validateGraphOnInitialization = validateGraph(this)
            validateGraphOnInitialization = this.validateGraphOnInitialization;
        end

        % Process a cell array which is a sequence of events. Each event is
        % processed in the order it appears in the array.
        function processEvents(this, eventQueue)

            % Increment the step number
            this.stepNumber = this.stepNumber + 1;
           
            events = eventQueue.events();
            
            % Process all the events
            for e = 1 : length(events)                
                event = events{e};

                % First check it's of the right class.
                assert(isa(event, 'minislam.event_types.Event'), ...
                    'slamsystem:processevents:wrongobjecttype', ...
                    'The object type is %s', class(event));
                
                % Now do all the actual work
                this.processEvent(event);
            end
            
            % Handle any post-event activities
            this.storeStepResults();
            
        end
        
        % Process the individual event. If the time of the current event is
        % not the same as the current time in the filter, first take steps
        % to predict the filter state to the current time.
        function processEvent(this, event)

            % If initialized, predict if the timestep length is
            % sufficiently long
            if (this.initialized == true)
                dT = event.time() - this.currentTime;

                % Nothing to do if it's really close to the last
                if (abs(dT) < 1e-3)
                    this.handleNoPrediction();
                else
                    this.handlePredictToTime(event.time(), dT);        
                    this.currentTime = event.time();
                end
            end
            
            % odometry and initialization events.
            switch(event.type)
                
                case minislam.event_types.EventTypes.INITIAL_CONDITION
                    assert(this.initialized == false)
                    this.handleInitialConditionEvent(event);
                    this.initialized = true;
                    
                case minislam.event_types.EventTypes.GPS
                    if (this.initialized == true)
                        this.handleGPSObservationEvent(event);
                    end
                    
                case minislam.event_types.EventTypes.COMPASS
                    if (this.initialized == true)
                        this.handleCompassObservationEvent(event);
                    end

                case minislam.event_types.EventTypes.LANDMARK
                    if (this.initialized == true)
                        this.handleLandmarkObservationEvent(event);
                    end
                    
                case minislam.event_types.EventTypes.HEARTBEAT
                    this.handleHeartbeatEvent(event);

                case minislam.event_types.EventTypes.VEHICLE_ODOMETRY
                    this.handleVehicleOdometryEvent(event);

                otherwise
                    error('Unknown event type %s', event.type)     
            end
        end
        
        % This method runs the graph optimiser and the outputs of the
        % optimisation are used as the initial conditions for the next
        % round of the graph. This makes it possible to iterative compute
        % solutions as more information becomes available over time.
        function chi2 = optimize(this, maximumNumberOfOptimizationSteps)

            this.graph.initializeOptimization(this.validateGraphOnInitialization);
            
            if (nargin > 1)
                this.graph.optimize(maximumNumberOfOptimizationSteps);
            else
                this.graph.optimize();
            end
            
            % Compute the chi2 value if required
            if (nargout > 0)
                chi2 = this.graph.chi2();
            end
        end

    end
    
    methods(Access = public, Abstract)
        
        % Recommend if an optimization is required
        recommendation = recommendOptimization(this);

        % Get the mean and covariance of the estimate at the current time.
        % This is used for output purposes.
        [x, P] = platformEstimate(this);
        
        % Get the mean and covariance history of the robot across the whole
        % run. This is used for analysis purposes.
        [T, X, PX] = platformEstimateHistory(this);
        
        % Get the current landmarks estimates.
        [x, P, landmarkIds] = landmarkEstimates(this);
                
    end
    
    methods(Access = protected)
        
        % Create the localization system and start it up.
        function this = SLAMSystem(configuration)
            
            % Save configuration information
            this.configuration = configuration;
            
            % Create the graph and the optimization algorithm
            this.graph = g2o.core.SparseOptimizer();
            algorithm = g2o.core.LevenbergMarquardtOptimizationAlgorithm();
            this.graph.setAlgorithm(algorithm);
            
            % Do detailed checking by default
            this.validateGraphOnInitialization = true;
            
            % Set the start time to 0.
            this.currentTime = 0;
            
            % Set the current step number to zero
            this.stepNumber = 0;
            
            % Set that we need to initialize
            this.initialized = false;
        end
        
        % Handle a new odometry event. We simply change the vehicle
        % odometry to the new value.
        function handleVehicleOdometryEvent(this, event)
            this.u = event.data;
            this.uCov = event.covariance;
        end

    end
    
    methods(Access = protected, Abstract)
       
        % Handle the initial conditions
        handleInitialConditionEvent(this, event);
        
        % Handle when there is no prediction between events
        handleNoPrediction(this);
        
        % Handle everything needed to predict to the current state. Note we
        % pass in both the time step length and the absolute next time just
        % in case that's needed.
        handlePredictToTime(this, deltaToNextTime, nextTime);
 
        % Handle a GPS measurement
        handleGPSObservationEvent(this, event);
        
        % Handle a compass measurement
        handleCompassObservationEvent(this, event);
            
        % Handle a set of measurements of landmarks
        handleLandmarkObservationEvent(this, event);
        
        % Handle the heartbeat event
        handleHeartbeatEvent(this, event);

        % Store results
        storeStepResults(this);
        
    end
end
