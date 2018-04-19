classdef Robot < handle
    %ROBOT class for controlling NXT mindstorm robot.
    
    properties
        % Default BotSim variables some are likely not needed.
        pos;    %position of the robot
        ang;    %angle of the robot (radians)
        dir;    %angle of the robot (stored as 2d vector)
        map = [0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105]; %default map;    %map coordinates with a copy of the first coordiantes at the end
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
%         numScans = 30; %is this a desireable change? will need to change
%         functions too for this to make sense.
        
        %Callibration Variables
        cmPerDeg; %forward distance
        degPerDeg; %turning variable
        motionNoise;
        turningNoise;
        m1Cal = 1.0; %Calibration for motor 1 Left motor
        m2Cal = 1.0; %Calibration for motor 2 Right motor
        m3Cal = 1.0; %Calibration for motor 3 NOT NEEDED
        
        %power variables
        pUltra = 20;    %ultra scanner power
        pTurn = 100;     %turning power
        pMove = 100;     %moving power
        
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
            nxt.loadCallibration()
            OpenUltrasonic(SENSOR_1);
            nxt.setUpScanner(1)
        end
        
        %load callibration variables from 'callibration.mat'
        function loadCallibration(nxt)
            calVars = load('callibration', 'cmPerDeg' , 'degPerDeg', 'motionNoise', 'turningNoise');
            nxt.cmPerDeg = calVars.cmPerDeg;
            nxt.degPerDeg = calVars.degPerDeg;
            nxt.motionNoise = calVars.motionNoise;
            nxt.turningNoise = calVars.turningNoise;
        end
        
        %safely exit NXT
        function close(nxt)
           if nxt.posC > 0 %if the wire is twisted
                nxt.rotScan(20); %untwist the wire
           end
           pause(.5)
           nxt.setUpScanner(0)
           COM_CloseNXT all;
        end
        
        %beep boop, takes frequency and time in ms
        function beep(~, hz, ms)
            NXT_PlayTone(hz, ms);
        end
        
        %forward scan
        function dist = scan(~, num)
            dist = zeros(num, 1);
            for i = 1:num
                dist(i) = GetUltrasonic(SENSOR_1);
                pause(0.1);
            end
        end
        
        %rotating scan
        function scan = rotScan(nxt, numScans) 
            scan = zeros(numScans, 1);
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
                     end        
                end
                scan(2:end) = flip(scan(2:end));
            end
            fudgeFactor = -15;
            if sign(initPos)>0 %Anti clockwise Scan
                while nxt.datC.IsRunning || scanCount<numScans-1
                     nxt.sensorAngle();
                     if nxt.posC <= 360 + fudgeFactor - scanCount*(360/numScans) && nxt.posC > 0+fudgeFactor
                        scanCount = scanCount + 1;
                        scan(scanCount) = GetUltrasonic(SENSOR_1);
                     end          
%                 scan(2:end) = flip(scan(2:end));
                end
                
            end
       
            
            mC.WaitFor();

        end
        
        
        function setUpScanner(nxt, initial)
            if initial == 1
                turnPow = nxt.pUltra * -sign(nxt.scanOffset);
            else
                turnPow = nxt.pUltra * sign(nxt.scanOffset);
            end
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
            
            % ADD FOLLOWING LINE DUE TO CHANGES MADE IN REDESIGN
            howFar = -howFar;
            
            movePow = nxt.pMove * sign(howFar);
            howFar = abs(howFar);
            %run the motors
            mA = NXTMotor('A', 'Power', movePow, 'TachoLimit', round(howFar/nxt.cmPerDeg)*nxt.m1Cal);
            mB = NXTMotor('B', 'Power', movePow, 'TachoLimit', round(howFar/nxt.cmPerDeg)*nxt.m2Cal);
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
        
        %callibrate TODO: store this data in a callibration .txt/.csv/.mat
        function callibrate(nxt, numTrials)
            wheelDegsMove = 400; %test movemement tacho
            wheelDegsTurn = 200; %test turn tacho
            ready = 'NoGo';

            while strcmp(ready,'GO') == 0
                ready = input(['Type GO when you are ready to measure robot forward movement' newline], 's');
            end
            
            pause(.5)
            mAB = NXTMotor('AB', 'Power', nxt.pMove, 'TachoLimit', wheelDegsMove);
            mAB.SpeedRegulation = false;
            mAB.SmoothStart = true;

            dists = zeros(numTrials, 1);

            for i = 1:numTrials
                mAB.SendToNXT();
                mAB.WaitFor();
                mAB.Stop('brake') 
                dists(i) = input(['How many cm did the robot move?' newline]);
            end
            ready = 'NoGo';

            while strcmp(ready,'GO') == 0
                ready = input(['Type GO when you are ready to observe straight forward movement' newline], 's');
            end

            %callibrate left right
            straight = 0;
            while ~straight
               nxt.move(10)
               userInput = input(['Did the robot move left/right/straight?' newline], 's');
             
               if strcmp(userInput, 'straight') == 1
                   straight = 1;
               elseif strcmp(userInput, 'left') == 1
                   nxt.m1Cal = nxt.m1Cal + 0.02;
                   nxt.m2Cal = nxt.m2Cal - 0.02;
               elseif strcmp(userInput, 'right') == 1
                   nxt.m1Cal = nxt.m1Cal - 0.02;
                   nxt.m2Cal = nxt.m2Cal + 0.02;
               end
                
            end
            
            %end left right 
            
            nxt.cmPerDeg = mean(dists) / wheelDegsMove;
            nxt.motionNoise = std(dists);

            sprintf('This robot moves %d cm per 1 degree of wheel rotation.' , nxt.cmPerDeg)
            disp(newline)
            sprintf('This robot has approximately %d movementNoise.' , nxt.motionNoise)
            disp(newline)

            %% Angular Calibration

            ready = 'NoGo';

            while strcmp(ready,'GO') == 0
                ready = input(['Type GO when you are ready to measure robot turning movement' newline], 's');
            end
            %set up commands for turn
            mA = NXTMotor('A', 'Power', nxt.pTurn, 'TachoLimit', wheelDegsTurn);
            mB = NXTMotor('B', 'Power', -nxt.pTurn, 'TachoLimit', wheelDegsTurn);
            mA.SpeedRegulation = false;
            mB.SpeedRegulation = false;
            mA.SmoothStart = true;
            mB.SmoothStart = true;

            angs = zeros(numTrials, 1);
            %turn
            for i = 1:numTrials
                mA.SendToNXT();
                mB.SendToNXT();
                mA.WaitFor();
                mB.WaitFor();
                mA.Stop('brake') 
                mB.Stop('brake')
                angs(i) = input(['How much did it turn in degrees?' newline]);
            end

            nxt.degPerDeg = mean(angs) / wheelDegsTurn;
            nxt.turningNoise = std(angs);

            sprintf('This robot turns %d degs per 1 degree of wheel rotations.' , nxt.degPerDeg)
            disp(newline)
            sprintf('This robot has approximately %d turningNoise.' , nxt.turningNoise)
            disp(newline)
            disp('Variables to save: ')
            sprintf('cmPerDeg: %d' , nxt.cmPerDeg)
            sprintf('motionNoise: %d' , nxt.motionNoise)
            sprintf('degPerDeg: %d' , nxt.degPerDeg)
            sprintf('turningNoise: %d' , nxt.turningNoise)
            sprintf('m1cal: %d' , nxt.m1Cal)
            sprintf('m2cal: %d' , nxt.m2Cal)
            userInput = input('Do you want to save to callibration.mat? (yes/no) ==> ', 's');
            if strcmp(userInput,'yes')
                save('callibration.mat', 'nxt.cmPerDeg' , 'nxt.degPerDeg', 'nxt.motionNoise', 'nxt.turningNoise', 'nxt.m1Cal', 'nxt.m2Cal');                
            else
                warning('Input not equal to "yes". Callibration variables will not be saved')
            end
            nxt.loadCallibration(); %load the new OR old callibration variables

        end
  
    end
    
end

