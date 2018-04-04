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
        wireTwist = -1;
        
        %New Variables
        h; %the robot handle
        SensorAngle;
        m1Cal; %Calibration for motor 1
        m2Cal; %Calibration for motor 2
        m3Cal; %Calibration for motor 3
    end
    
    methods
        %set up robot, called on initiating class
        function nxt = Robot()
            COM_CloseNXT all;  
            %COM_CloseNXT('all');
            h = COM_OpenNXT();      %look for USB devices
            COM_SetDefaultNXT(h);   %sets default handle
            OpenUltrasonic(SENSOR_1); %opens ultrasound connection
        end
        
        %safely exit NXT
        function close(nxt)
           pause(.5)
           COM_CloseNXT all;
        end
        
        %beep boop, takes frequency and time in ms
        function beep(nxt, hz, ms)
            NXT_PlayTone(hz, ms);
        end
        
        %forward scan
        function dist = scan(nxt, num)
            dist = zeros(num, 1);
            for i = 1:num
                dist(i) = GetUltrasonic(SENSOR_1);
                pause(0.1);
            end
        end
        
        %rotating scan
        function scan = rotScan(nxt, numScans, turnPow)
            turnPow = turnPow * nxt.wireTwist;
            scan = zeros(numScans, 2);
            mC = NXTMotor('C', 'Power', turnPow, 'TachoLimit', 360);
            %OpenUltrasonic(SENSOR_1);
            mC.SpeedRegulation = false;
            mC.SmoothStart = false;
            mC.ResetPosition(); 
            pause(.5);
            datC = mC.ReadFromNXT();
            posC = datC.Position;
            mC.SendToNXT();
            scan(1,1) = GetUltrasonic(SENSOR_1);
            scan(1,2) = posC;
            scanCount = 1;

            while ((datC.IsRunning == 1) || (scanCount == 1) )
                if (abs(posC)) > scanCount*(360/numScans)
                    scanCount = scanCount +  1;
                    scan(min(scanCount, numScans),1) = GetUltrasonic(SENSOR_1);
                    scan(min(scanCount, numScans),2) = posC;
                end
                datC = mC.ReadFromNXT();
                posC = datC.Position;
            end

            mC.WaitFor();
            nxt.wireTwist = - nxt.wireTwist;
        end
    end
    
end

