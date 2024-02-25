classdef SimulatorConfiguration < handle
   
    % This class contains a set of parameters. Given that these are shared,
    % you should NOT edit these values directly. 
    
    properties(Access = public)
        
        % Time step length
        DT = 0.1;
        
        % If set to a finite value, the terminator artificially terminates
        % at this step number. You might find this useful for debugging.
        maximumStepNumber = inf;
        
        % Vehicle wheelbase. See https://en.wikipedia.org/wiki/Wheelbase.
        % Average wheelbase value from
        % Chassis Handbook: Fundamentals, Driving Dynamics, Components, Mechatronics, Perspectives
        B = 2.5;
        
        % Flag to show if noises should be added. If you set this parameter
        % to false, the simulator generates measurements. This can be very
        % useful for debugging. However, unless specified otherwise, any
        % submitted results for any questions or subparts of questions
        % must have this value set to true.
        perturbWithNoise = true;
        
        % The period between sensor measurements is not entirely
        % deterministic. The dither time refers to a small amount of time
        % which is added to each measurement. You can set this to zero
        % to help debugging, but all submitted results must have this
        % set to true.
        sensorDitherTime = 1e-2;
        
        % Steer angle controls for the vehicle. Do not change.
        maxDiffDeltaRate = 20*pi/180;
        maxDelta = 40 * pi / 180;
        
        % Speed controls for the vehicle. Do not change.
        maxSpeed = 10;
        minSpeed = 1;
        maxAcceleration = 0.5;

        % The flags below set the default configurations for different
        % sensors. Different launcher scripts will override these values to
        % produce different behaviours.

        % Flag to show if odometry is available.
        enableOdometry = true;
        
        % Control input measurement noise
        ROdometry = diag([0.2 0.1 pi/180]).^2;
       
        % Flag to show if the compass is available
        enableCompass = false;
        
        % The period between compass measurements
        compassMeasurementPeriod = 0.1;
        
        % Noise on each measurement
        RCompass = (5*pi/180)^2;
        
        % Angle offset of the compass (in the platform frame)
        compassAngularOffset = 0;

        % Flag to show if GPS available
        enableGPS = false;        
        
        % The period between GPS measurements
        gpsMeasurementPeriod = 1;
        
        % GPS noise measurement
        RGPS = 1 * eye(2);
        
        % Position offset of the GPS (in platform frame)
        gpsPositionOffset = [10 -20]';
        
        % Flag to show if the laser is available
        enableLaser = false;
        
        % The period between laser measurements
        laserMeasurementPeriod = 0.1;
        
        % Detection range - same for all landmarks
        laserDetectionRange = 10;
        
        % Noise on each measurement
        RLaser = diag([0.1 pi/180]).^2;

        
    end
end