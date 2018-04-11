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
        
        %pos of the ultrascanner
        posC;
        datC;
        scanOffset = 20;
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
            scan = zeros(numScans,1);
            initPos = nxt.sensorAngle();
            turnPow = nxt.pUltra * (-sign(initPos));
            
            mC = NXTMotor('C', 'Power', turnPow, 'TachoLimit', 400); %make this so it moves to a set position
            mC.SpeedRegulation = false;
            mC.SmoothStart = false;
            pause(.5); %why is this pause here, can it be shorter?

            mC.SendToNXT();
            scanCount = 0;
            if sign(initPos)<0 %Clockwise Scan
                while nxt.datC.IsRunning || scanCount<numScans-1
                     nxt.sensorAngle();
                     if nxt.posC >= scanCount*(360/numScans) && nxt.posC < 360
                        scanCount = scanCount + 1;
                        scan(scanCount) = GetUltrasonic(SENSOR_1);
%                         scan(scanCount, 2) = nxt.posC;

                     end

                end
            end
            
            if sign(initPos)>0 %Anti clockwise Scan
                while nxt.datC.IsRunning || scanCount<numScans-1
                     nxt.sensorAngle();
                     if nxt.posC <= 360 - scanCount*(360/numScans) && nxt.posC > 0
                        scanCount = scanCount + 1;
                        scan(scanCount) = GetUltrasonic(SENSOR_1);
%                         scan(scanCount, 2) = nxt.posC;

                     end
                scan(2:end) = flip(scan(2:end));
                end
                
            end
       
            
            mC.WaitFor();
            %switch the 2nd to end scan array, if counter-rotating
%             if sign(nxt.wireTwist)<0
%                  
%             end
% 
%             nxt.wireTwist = - nxt.wireTwist;
        end
        
        
        function setUpScanner(nxt)
            turnPow = nxt.pUltra * -sign(nxt.scanOffset);
            mC = NXTMotor('C', 'Power', turnPow, 'TachoLimit', abs(nxt.scanOffset));
            mC.SpeedRegulation = false;
            mC.SmoothStart = false;
            mC.ResetPosition();
            mC.SendToNXT();
            mC.WaitFor();
            nxt.datC = mC.ReadFromNXT();
            nxt.posC = nxt.datC.Position;
            pause(0.4)
        end
            
        function angle = sensorAngle(nxt)
            mC = NXTMotor('C');
            nxt.datC = mC.ReadFromNXT();
            nxt.posC = nxt.datC.Position;
            angle = nxt.posC;            
        end
        

        
        %turn
        function turn(nxt, angle)
            %convert 
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

