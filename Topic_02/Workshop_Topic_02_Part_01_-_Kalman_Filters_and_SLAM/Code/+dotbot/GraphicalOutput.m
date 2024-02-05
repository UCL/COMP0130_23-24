classdef GraphicalOutput < handle
    
    % This class generates real-time graphics output in matlab. The key
    % thing for getting speedy graphics is to use handles and to change
    % values rather than draw things over and over again from scratch.
    % Unfortunately, the code ends up being pretty cumbersome, however.
    
    properties(Access = protected)
        
        % The configuration
        configuration;
        
        % The event generator and localization system, which are poked to
        % produce various kinds of outpus
        eventGenerator;
        numLocalizationSystems;
        localizationSystems;
        
        % Figure handle
        figureHandle
        
        % Base geometry for the robot
        dotBotGeometry;
        
        % True vehicle triangle handle
        trueVehicleTriangle;
        trueVehicleX;
        
        % Landmarks
        trueLandmarks;
        
        % GPS measurement
        gpsMeasurementCross;
        gpsCovarianceEllipse;
        
        % Laser measurements
        laserMeasurementLines;
                
        % Estimates
        xEst;  
        PEst;
        estimatedLandmarks;
        PEstimatedLandmarks;
        
        % Display timestep
        displayTimestep;
        
    end
    
    
    methods(Access = public)
        
        function this = GraphicalOutput(configuration, eventGenerator, localizationSystems)
            
            % Store the configuration
            this.configuration = configuration;
            
            % Store the event generator (simulator) and any localization
            % systems
            this.eventGenerator = eventGenerator;
            
            % Sort out the localization systems
            if (nargin == 2)
                this.localizationSystems = {};
            else            
                if (iscell(localizationSystems) == false)
                    this.localizationSystems = {localizationSystems};
                else
                    this.localizationSystems = localizationSystems;
                end
            end
            this.numLocalizationSystems = length(this.localizationSystems);
        end
        
        function start(this)

            % Nothing to do if graphics are disabled
            if (this.configuration.showGraphics == false)
                return
            end
                       
            % Get the ground truth state
            groundTruthState = this.eventGenerator.groundTruth(true);
            
            % Do the initialization
            this.figureHandle = minislam.graphics.FigureManager.getFigure('Simulator Output');
            clf;
            hold on
            axis equal
            
            % The axis will enclose the extent plus a little bit
            minCoord = -1.05 * this.configuration.extent;
            maxCoord = 1.05 * this.configuration.extent;
            
            axis([minCoord maxCoord minCoord maxCoord])
            xlabel('x coordinate')
            xlabel('y coordinate')
            title('dotbot scenario')
            
            % Draw a rectangle surrounding it all
            rectangle('Position',[minCoord minCoord maxCoord*2 maxCoord*2]);
            
            % Landmark graphics
            this.trueLandmarks = plot(groundTruthState.mTrue(1, :), ...
                groundTruthState.mTrue(2, :), 'k*', 'MarkerSize', 8);
            
            % Create the circle associated with the ground truth robot
            % position
            r = this.configuration.dotBotRadius;
            d = r*2;
            px = 1;
            py = 1;
            this.dotBotGeometry = rectangle('Position',[px py d d],'Curvature',[1,1], 'FaceColor', 'r');
            %this.dotBotGeometry = plot(NaN, NaN, 'o', 'Markersize', 10);
            
            % Create the observation associated with the GPS sensor
            this.gpsMeasurementCross = plot(NaN, NaN, 'm+', 'MarkerSize', 8);
            this.gpsCovarianceEllipse = plot(NaN, NaN, 'm');
            
            % Create the lines for the laser measurements
            this.laserMeasurementLines = plot(NaN, NaN, 'b', 'LineWidth', 2);
            
            % Allocate the landmark and vehicle plot estimates
            colours = [
                         0         0    1.0000
                        1.0000         0         0
                         0    1.0000         0
                         0         0    0.1724
                    1.0000    0.1034    0.7241
                    1.0000    0.8276         0
                         0    0.3448         0
                    0.5172    0.5172    1.0000
                    0.6207    0.3103    0.2759
                         0    1.0000    0.7586];
     
            for l = 1 : this.numLocalizationSystems           
                this.estimatedLandmarks{l} = plot(NaN, NaN, '+', 'MarkerSize', 4, 'Color', colours(l,:));            
                this.PEstimatedLandmarks{l} = plot(NaN, NaN, '-', 'LineWidth', 2, 'Color', colours(l,:));
            
                % Estimate
                this.xEst{l} = plot(NaN, NaN, 'g*', 'MarkerSize', 4);
                this.PEst{l} = plot(NaN, NaN, 'g');            
            end
            drawnow
            
            this.displayTimestep = 0;
            
        end
        
        function frameStart(~)
        end
        
        function frameEnd(~)
            drawnow
        end
        
        function update(this)

            % Nothing to do if graphics are disabled
            if (this.configuration.showGraphics == false)
                return
            end
            
            this.displayTimestep = this.displayTimestep + 1;
            
            if (rem(this.displayTimestep, this.configuration.updateFrequency) > 1)
                return
            end
            
            this.updateFromSimulator();
            this.updateFromLocalizationSystems();
            
            drawnow;
            pause(this.configuration.pauseTime)
            
        end
    end
    
    methods(Access = private)
        
        % This method updates all the graphics associated with the
        % simulator, including the ground truth dotbot position.
        function updateFromSimulator(this)
            
            % Update the ground truth position of the dot bot
            groundTruthState = this.eventGenerator.groundTruth(false);
            this.trueVehicleX = groundTruthState.xTrue;
                       
            r = this.configuration.dotBotRadius;
            d = r*2;
            px = groundTruthState.xTrue(1) - r;
            py = groundTruthState.xTrue(3) - r;
            set(this.dotBotGeometry, 'Position', [px py d d]);
            
            % Reset all observations and draw
            %set(this.gpsMeasurementCross, 'XData', NaN, 'YData', NaN);
            %set(this.laserMeasurementLines, 'XData', NaN, 'YData', NaN);
            
            % Now iterate
            orderedEvents = this.eventGenerator.events();
            events = orderedEvents.events();
            
            for k = 1 : length(events)                
                event = events{k};

                % First check it's of the right class.
                assert(isa(event, 'minislam.event_types.Event'));
                
                % Now do all the actual work
                switch(event.type)

                    case minislam.event_types.Event.GPS
                        set(this.gpsMeasurementCross, 'XData', event.data(1), 'YData', event.data(2));
                        pts = this.covarianceEllipsePoints(event.data, event.covariance);
                        set(this.gpsCovarianceEllipse, 'XData', pts(1, :), 'YData', pts(2, :));

                    case minislam.event_types.Event.LANDMARK
                        numLandmarks = length(event.landmarkIds);

                        lX = NaN(1, 3 * numLandmarks - 1);
                        lY = NaN(1, 3 * numLandmarks - 1);

                        for l = 1 : numLandmarks
                            lX(3*l-2:3*l-1) = this.trueVehicleX(1) + ...
                                [0 event.data(1, l)];
                            lY(3*l-2:3*l-1) = this.trueVehicleX(3) + ...
                                [0 event.data(2, l)];
                        end
                        set(this.laserMeasurementLines, 'XData', lX, 'YData', lY);
                end            
            end
        end
        
        function updateFromLocalizationSystems(this)
            
            for l = 1 : this.numLocalizationSystems
            
                % Update the vehicle
                [x, P] = this.localizationSystems{l}.platformEstimate();
                set(this.xEst{l}, 'XData', x(1), 'YData', x(3));
                covPts = this.covarianceEllipsePoints([x(1);x(3)], P([1 3], [1 3]));
                set(this.PEst{l}, 'XData', covPts(1, :), 'YData', covPts(2, :));

                % Now update the landmarks
                [lx, P] = this.localizationSystems{l}.landmarkEstimates();
                set(this.estimatedLandmarks{l}, 'XData', lx(1, :), 'YData', lx(2, :));
                               
                covPtsX = [];
                covPtsY = [];
                for p =  1 : size(P, 3)
                    ptsC = this.covarianceEllipsePoints(lx(:, p), P(:, :, p), 3);
                    covPtsX = cat(2, covPtsX, [ptsC(1, :) NaN]);
                    covPtsY = cat(2, covPtsY, [ptsC(2, :) NaN]);  
                end
                set(this.PEstimatedLandmarks{l}, 'XData', covPtsX, 'YData', covPtsY);               
            end
        end        

        
        function ptsC = covarianceEllipsePoints(this, x, P, sigmaValue)
            if (nargin < 4)
                sigmaValue = this.configuration.sigmaEllipse;
            end

            % If the state is NaN, not initialized yet
            if (any(isnan(x)) == true)
                ptsC = NaN(2, 1);
                return
           end
            
            theta = [0 : 15 : 360] * pi / 180;
            ptsC = sigmaValue * sqrtm(P(1:2, 1:2)+1e-12*eye(2)) * [cos(theta);sin(theta)];
            
            %translate
            ptsC(1, :) = ptsC(1, :) + x(1);
            ptsC(2, :) = ptsC(2, :) + x(2);
        end    
    end
    

end