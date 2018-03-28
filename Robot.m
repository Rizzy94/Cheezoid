classdef Robot < handle
    %ROBOT class for controlling NXT mindstorm robot.
    
    properties
        % Default BotSim variables some are likely not needed.
        pos;    %position of the robot
        ang;    %angle of the robot (radians)
        dir;    %angle of the robot (stored as 2d vector)
        map;    %map coordinates with a copy of the first coordiantes at the end
        mapLines;   %The map stored as a list of lines (for easy line interection)
        inpolygonMapformatX; %The map stored as a polygon for the insidepoly function
        inpolygonMapformatY; %The map stored as a polygon for the insidepoly function
        scanConfig;     %stores how the robot performs a scan (number of points, angle between points)
        scanLines;      %the scan configuration stored as 2d lines
        sl;
        sensorNoise;     %how much noise to add to a scan. Constant noise model. Error standard deviation in cm
        motionNoise;     %how much noise when moving. Proportional noise model. cm error stdDev per unit length in cm/cm
        turningNoise;    %how much noise when turning. Porportional noise model. Radian stdDev error per radian rad/rad
        adminKey;       %the key to lock off certain features
        PosHistory;  %stores history of where the robot has been
        MoveCount;
        
        %New Variables
        SensorAngle;
        m1Cal; %Calibration for motor 1
        m2Cal; %Calibration for motor 2
        m3Cal; %Calibration for motor 3
    end
    
    methods
        
    end
    
end
