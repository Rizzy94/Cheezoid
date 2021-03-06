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
        
        %Callibration Variables
        cmPerDeg = 0.0361;
        degPerDeg = 0.2067;
        motionNoise = 0.9083;
        turningNoise = 12.2474;
        
        %power variables
        pUltra = 70;    %ultra scanner power
        pTurn = 70;     %turning power
        pMove = 70;     %moving power
    end
    
    methods
        %set up robot, called on initiating class
        function nxt = Robot()
            COM_CloseNXT all;  
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
        function scan = rotScan(nxt, numScans)
            turnPow = nxt.pUltra * nxt.wireTwist;
            scan = zeros(numScans, 1);
            mC = NXTMotor('C', 'Power', -turnPow, 'TachoLimit', 360);
            mC.SpeedRegulation = false;
            mC.SmoothStart = false;
            mC.ResetPosition(); 
            pause(.5); %why is this pause here, can it be shorter?
            datC = mC.ReadFromNXT();
            posC = datC.Position;
            mC.SendToNXT();
            scan(1) = GetUltrasonic(SENSOR_1);
            scanCount = 1;

            while ((datC.IsRunning == 1) || (scanCount == 1) )
                if (abs(posC)) > scanCount*(360/numScans)
                    scanCount = scanCount +  1;
                    scan(min(scanCount, numScans)) = GetUltrasonic(SENSOR_1);
                end
                datC = mC.ReadFromNXT();
                posC = datC.Position;
            end

%             if sign(nxt.wireTwist)<0
%                 scan(2:end) = flip(scan(2:end)); 
%             end
            nxt.wireTwist = - nxt.wireTwist;
            
            mC.WaitFor();
            %switch the 2nd to end scan array, if counter-rotating
            mC = NXTMotor('C', 'Power', turnPow, 'TachoLimit', 360);
            mC.SpeedRegulation = false;
            mC.SmoothStart = false;
            mC.SendToNXT();
            mC.WaitFor();
            nxt.wireTwist = - nxt.wireTwist;
            
        end
        
        %turn
        function turn(nxt, angle)
            %convert radian input to degrees
            angle = rad2deg(angle);
            turnPow = nxt.pTurn * sign(angle);
            angle = abs(angle);
            mA = NXTMotor('A', 'Power', turnPow, 'TachoLimit', round(angle/nxt.degPerDeg));
            mB = NXTMotor('B', 'Power', -turnPow, 'TachoLimit', round(angle/nxt.degPerDeg));
            mB.SpeedRegulation = false;
            mB.SmoothStart = true;
            mA.SpeedRegulation = false;
            mA.SmoothStart = true;

            mA.SendToNXT();
            mB.SendToNXT();
            mA.WaitFor();
            mB.WaitFor();
            mA.Stop('brake') 
            mB.Stop('brake') 

        end
        
        %move
        function move(nxt, howFar)
            %allow for backwards movements
            movePow = nxt.pMove * sign(howFar);
            howFar = abs(howFar);
            %run the motors
            mAB = NXTMotor('AB', 'Power', movePow, 'TachoLimit', round(howFar/nxt.cmPerDeg));
            mAB.SpeedRegulation = false;
            mAB.SmoothStart = true;
            mAB.SendToNXT();
            mAB.WaitFor();
            mAB.Stop('brake') 
        end
        
        %callibrate TODO: store this data in a callibration .txt/.csv/.mat
        function callibrate(nxt)
            
        end
  
    end
    
end

