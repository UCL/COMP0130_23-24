classdef GraphicalOutput < handle
    
    % This class generates real-time graphics output in matlab. The key
    % thing for getting speedy graphics is to use handles and to change
    % values rather than draw things over and over again from scratch.
    % Unfortunately, the code ends up being pretty cumbersome, however.
    
    properties(Access = protected)
        
        % The parameters
        parameters;
        
        % The event generator and localization system, which are poked to
        % produce various kinds of outpus
        eventGenerator;
        numLocalizationSystems;
        localizationSystems;
        
        % Figure handle
        figureHandle
        
        % Base geometry for the robot
        trueVehicleGeometry;
        
        % True vehicle triangle handle
        trueVehicleTriangle;
        trueVehicleX;
        
        % Waypoints and routes
        waypointMarkers;
        waypointRoutes;
        
        % Landmarks
        trueLandmarks;
        
        % GPS measurement
        gpsMeasurementCross;
        
        % Laser measurements
        laserMeasurementLines;
        
        
        % Estimates
        xEst;  
        PEst;
        estimatedLandmarks;
        PEstimatedLandmarks;
        
    end
    
    
    methods(Access = public)
        
        function this = GraphicalOutput()
            % Nasty hack for vehicle size
            this.parameters.B = 2.5;
        end
        
        function initialize(this, eventGenerator, localizationSystems)
            
            % Store
            this.eventGenerator = eventGenerator;
            
            if (iscell(localizationSystems) == false)
                this.localizationSystems = {localizationSystems};
            else
                this.localizationSystems = localizationSystems;
            end
            
            this.numLocalizationSystems = length(this.localizationSystems);
            
            this.open();
            
            % Get the ground truth state
            groundTruthState = eventGenerator.groundTruth(true);
            
            % Plot the waypoints markers
             set(this.waypointMarkers, 'XData', groundTruthState.waypoints(1, :), ...
                'YData', groundTruthState.waypoints(2, :));
            
            % For the route, prepend the robot current position
            routeXData = [groundTruthState.xTrue(1) groundTruthState.waypoints(1, :)];
            routeYData = [groundTruthState.xTrue(2) groundTruthState.waypoints(2, :)];            
            set(this.waypointRoutes, 'XData', routeXData, 'YData', routeYData);
            
            % Plot the landmarks
            set(this.trueLandmarks, 'XData', groundTruthState.mTrue(1, :), ...
                'YData', groundTruthState.mTrue(2, :));
            
            % Figure out the axes - should be big enough to encompass all
            % the landmarks and waypoints
            minX = min([groundTruthState.waypoints(1, :) groundTruthState.mTrue(1, :)]);
            maxX = max([groundTruthState.waypoints(1, :) groundTruthState.mTrue(1, :)]);
            minY = min([groundTruthState.waypoints(2, :) groundTruthState.mTrue(2, :)]);
            maxY = max([groundTruthState.waypoints(2, :) groundTruthState.mTrue(2, :)]);
            
            % Bounding box size
            bbWidth = 1.2 * max(maxX - minX, 10);
            bbHeight = 1.2 * max(maxY - minY, 10);
            
            % Centre
            midX = 0.5*(minX + maxX);
            midY = 0.5*(minY + maxY);
            
            axis([midX-0.5*bbWidth midX+0.5*bbWidth midY-0.5*bbHeight midY+0.5*bbHeight])
            
            drawnow
        end
        
        function open(this)
            
            % Do the initialization
            this.figureHandle = minislam.graphics.FigureManager.getFigure('Simulator Output');
            clf;
            hold on
            axis equal
            
            % Landmark graphics
            this.trueLandmarks = plot(NaN, NaN, 'k+', 'MarkerSize', 8);

            % Create the triangle associated with the ground truth robot
            this.trueVehicleGeometry = 1.1*[0 -this.parameters.B -this.parameters.B; 0 -1.2 1.2];
            this.trueVehicleTriangle = patch(this.trueVehicleGeometry(1,:), ...
                this.trueVehicleGeometry(2,:), 'b', 'FaceAlpha', 0.5);
            
            % Create the observation associated with the GPS sensor
            this.gpsMeasurementCross = plot(NaN, NaN, 'k+', 'MarkerSize', 8);
            
            % Create the lines for the laser measurements
            this.laserMeasurementLines = plot(NaN, NaN, 'r');
            
            % Create the graphics associated with the waypoints and routes
            % (if available).
            this.waypointMarkers = plot(NaN, NaN, 'm+', 'MarkerSize', 4);
            this.waypointRoutes = plot(NaN, NaN, 'm-', 'LineWidth', 2);
            
            % Allocate the landmark and vehicle plot estimates
            colours = distinguishable_colors(this.numLocalizationSystems);

            for l = 1 : this.numLocalizationSystems           
                this.estimatedLandmarks{l} = plot(NaN, NaN, '+', 'MarkerSize', 4, 'Color', colours(l,:));            
                this.PEstimatedLandmarks{l} = plot(NaN, NaN, '-', 'LineWidth', 2, 'Color', colours(l,:));
            
                % Estimate
                this.xEst{l} = plot(NaN, NaN, '*', 'MarkerSize', 4, 'Color', colours(l,:));
                this.PEst{l} = plot(NaN, NaN, 'Color', colours(l,:));            
            end
            
            % Set the axis
            %axis(10 * [-10 80 -10 80])
            axis([-186.2662   32.9164 -111.5191  107.6636])
            %axis([-10 800 -10 800])
        end
        
        function frameStart(~)
        end
        
        function frameEnd(~)
            drawnow
        end
        
        function update(this)
            this.updateFromGroundTruth();
            this.updateFromLocalizationSystems();
            
            drawnow;
        end
        
        function updateFromGroundTruth(this)
            groundTruthState = this.eventGenerator.groundTruth(false);           
            this.updateGroundTruthRobot(groundTruthState.xTrue);            
        end
        
        function updateFromLocalizationSystems(this)
            
            for l = 1 : this.numLocalizationSystems
            
                % Update the vehicle
                [x, P] = this.localizationSystems{l}.platformEstimate();
                set(this.xEst{l}, 'XData', x(1), 'YData', x(2));
                covPts = this.getCovarianceEllipsePoints([x(1);x(2)], P, 3);
                set(this.PEst{l}, 'XData', covPts(1, :), 'YData', covPts(2, :));

                % Now update the landmarks
                [lx, P] = this.localizationSystems{l}.landmarkEstimates();
                set(this.estimatedLandmarks{l}, 'XData', lx(1, :), 'YData', lx(2, :));

                covPtsX = [];
                covPtsY = [];
                for p =  1 : size(P, 3)
                    ptsC = minislam.graphics.GraphicalOutput.getCovarianceEllipsePoints(lx(:, p), P(:, :, p), 3);
                    covPtsX = cat(2, covPtsX, [ptsC(1, :) NaN]);
                    covPtsY = cat(2, covPtsY, [ptsC(2, :) NaN]);  
                end
                set(this.PEstimatedLandmarks{l}, 'XData', covPtsX, 'YData', covPtsY);
            
            end
            
            % Reset all observations and draw
            set(this.gpsMeasurementCross, 'XData', NaN, 'YData', NaN);
            set(this.laserMeasurementLines, 'XData', NaN, 'YData', NaN);
            
            % Now iterate
            orderedEvents = this.eventGenerator.events();
            events = orderedEvents.events();
            
            for k = 1 : length(events)                
                event = events{k};

                % First check it's of the right class.
                assert(isa(event, 'minislam.event_types.Event'));
                
                % Now do all the actual work
                this.drawEvent(event);
            end
        end        
    end
    
    methods(Access = private)
        
        function drawEvent(this, event)
            
            switch(event.type)

                case minislam.event_types.EventTypes.VEHICLE_ODOMETRY
                        % Nothing to do

                    case minislam.event_types.EventTypes.GPS
                        set(this.gpsMeasurementCross, 'XData', event.data(1), 'YData', event.data(2));

                    case minislam.event_types.EventTypes.LANDMARK
                        numLandmarks = length(event.landmarkIds);
                        
                        lX = NaN(1, 3 * numLandmarks - 1);
                        lY = NaN(1, 3 * numLandmarks - 1);
                        
                        for l = 1 : numLandmarks
                            lX(3*l-2:3*l-1) = this.trueVehicleX(1) + ...
                                [0 event.data(1, l) * cos(event.data(2, l)+this.trueVehicleX(3))];
                            lY(3*l-2:3*l-1) = this.trueVehicleX(2) + ...
                                [0 event.data(1, l) * sin(event.data(2, l)+this.trueVehicleX(3))];
                        end
                        set(this.laserMeasurementLines, 'XData', lX, 'YData', lY);
            end            
        end    
        
        function updateGroundTruthRobot(this, x)
            this.trueVehicleX = x;
            ptsG = this.transformToGlobal(this.trueVehicleGeometry, x);
            set(this.trueVehicleTriangle, 'XData', ptsG(1,:), 'YData', ptsG(2,:));
        end

        function updateGPSMeasurement(this, z)
            set(this.gpsMeasurementCross, 'XData', z(1), 'YData', z(2));
        end
    end
    
    methods(Access = private, Static = true)
        
        function ptsG = transformToGlobal(ptsL, x)
            rot = [cos(x(3)) -sin(x(3)); sin(x(3)) cos(x(3))];
            ptsG(1:2,:) = rot*ptsL(1:2,:);

            % translate
            ptsG(1,:) = ptsG(1,:) + x(1);
            ptsG(2,:) = ptsG(2,:) + x(2);
        end
        
        function ptsC = getCovarianceEllipsePoints(x, P, sigmaValue)
            theta = [0 : 15 : 360] * pi / 180;
            ptsC = sigmaValue * sqrtm(P(1:2, 1:2)+1e-12*eye(2)) * [cos(theta);sin(theta)];
            
            %translate
            ptsC(1, :) = ptsC(1, :) + x(1);
            ptsC(2, :) = ptsC(2, :) + x(2);
        end
    end    
end