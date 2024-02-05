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
    end
       
    methods(Access = public)
        
        % Create the localization system and start it up.
        function this = SLAMSystem(configuration)
            
            % Save configuration information
            this.configuration = configuration;
            
            % Set the start time to 0.
            this.currentTime = 0;
            
            % Set the current step number to zero
            this.stepNumber = 0;
            
            this.initialized = false;
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
                assert(isa(event, 'minislam.event_types.Event'));
                
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
                    this.handlePredictToTime(dT, event.time());        
                    this.currentTime = event.time();
                end
            end
            
            % Handle the event on the basis of its type. Note that if we
            % are not initialized yet, we can only handle the
            % odometry and initialization events.
            switch(event.type)
                
                case minislam.event_types.Event.INITIAL_CONDITION
                    assert(this.initialized == false)
                    this.handleInitialConditionEvent(event);
                    this.initialized = true;
                    
                case minislam.event_types.Event.GPS
                    if (this.initialized == true)
                        this.handleGPSObservationEvent(event);
                    end

                case minislam.event_types.Event.LANDMARK
                    if (this.initialized == true)
                        this.handleLandmarkObservationEvent(event);
                    end
                    
                case minislam.event_types.Event.HEARTBEAT
                    % Nothing
                    
                otherwise
                    error('Unknown observation type %d', event.type)     
            end
        end
    end
    
    methods(Access = public, Abstract)
        
        % Get the mean and covariance of the estimate at the current time.
        % This is used for output purposes.
        [x, P] = platformEstimate(this);
        
        % Get the mean and covariance history of the robot across the whole
        % run. This is used for analysis purposes.
        [T, X, PX] = platformEstimateHistory(this);
        
        % Get the current landmarks estimates.
        [x, P, landmarkIds] = landmarkEstimates(this);
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
            
        % Handle a set of measurements of landmarks
        handleLandmarkObservationEvent(this, event);
        
        % Store step-by-step results
        storeStepResults(this);
        
    end
end
