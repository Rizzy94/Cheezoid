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
        pUltra = 40;    %ultra scanner power
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
            nxt.loadCalibration()
            OpenUltrasonic(SENSOR_1);
            nxt.setUpScanner(1)
        end
        
        %load callibration variables from 'callibration.mat'
        function loadCalibration(nxt)
            calVars = load('calibration', 'cmPerDeg' , 'degPerDeg', 'motionNoise', 'turningNoise', 'm1Cal', 'm2Cal');
            nxt.cmPerDeg = calVars.cmPerDeg;
            nxt.degPerDeg = calVars.degPerDeg;
            nxt.motionNoise = calVars.motionNoise;
            nxt.turningNoise = calVars.turningNoise;
            nxt.m1Cal = calVars.m1Cal;
            nxt.m2Cal = calVars.m2Cal;
        end
        
        function saveCalibration(nxt)
            cmPerDeg = nxt.cmPerDeg;
            degPerDeg = nxt.degPerDeg;
            motionNoise = nxt.motionNoise;
            turningNoise = nxt.turningNoise;
            m1Cal = nxt.m1Cal;
            m2Cal = nxt.m2Cal;            
            save('calibration.mat', 'cmPerDeg' , 'degPerDeg', 'motionNoise', 'turningNoise', 'm1Cal', 'm2Cal');
        end
        
        %safely exit NXT
        function close(nxt)
           if nxt.posC > 0 %if the wire is twisted
                nxt.rotScan(20); %untwist the wire
           end
           pause(.5)
           nxt.setUpScanner(0)
           display('Closing robot connection')
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
        
        %rotating scan
        function scan = rotScanStopStart(nxt, numScans) 
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
                        mC.Stop('brake')
                        pause(.2)
                        scan(scanCount) = GetUltrasonic(SENSOR_1);
                        pause(.1)
                        mC = NXTMotor('C', 'Power', turnPow, 'TachoLimit', 400-abs(nxt.posC - initPos)); %make this so it moves to a set position
                        mC.SpeedRegulation = false;
                        mC.SmoothStart = false;
                        pause(.5); %why is this pause here, can it be shorter?
                        mC.SendToNXT();
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
            if howFar == 0; warning('nxt.move passed a 0 move command'); end
            %allow for backwards movements
            movePow = nxt.pMove * -sign(howFar);
            howFar = abs(howFar);
            %run the motors
            mA = NXTMotor('A', 'Power', movePow, 'TachoLimit', round((howFar*nxt.m1Cal)/nxt.cmPerDeg)); 
            mB = NXTMotor('B', 'Power', movePow, 'TachoLimit', round((howFar*nxt.m2Cal)/nxt.cmPerDeg));
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
        
        function userReady(~, obs)
            aString = 'Type GO when you are ready to ';
            go = 0;
            while ~go
               userInput = input([strcat(aString, obs) newline], 's');            
               if strcmp(userInput, 'GO') == 1
                   go = 1;
               else
                   warning('Expected input: GO')
               end                
            end  
        end
        
        function calibrate(nxt)
            %TODO: consider calibrating for noise
            
            %calibrate forward movement, over under correct            
            nxt.userReady('measure forward movement of 22cm')            
            correct = 0;
            while ~correct
               nxt.move(22)
               userInput = input(['Did the robot move over/under/correct?' newline], 's');            
               if strcmp(userInput, 'correct') == 1
                   correct = 1;
               elseif strcmp(userInput, 'over') == 1
                   nxt.cmPerDeg = nxt.cmPerDeg + 0.001;
               elseif strcmp(userInput, 'under') == 1
                   nxt.cmPerDeg = nxt.cmPerDeg - 0.001;
               else 
                   warning('Expected input: over; under; correct')
               end               
            end  

            %calibrate forward movement, left right straight
            nxt.userReady('measure straight movement')
            straight = 0;
            while ~straight
               nxt.move(22)
               userInput = input(['Did the robot move left/right/straight?' newline], 's');            
               if strcmp(userInput, 'straight') == 1
                   straight = 1;
               elseif strcmp(userInput, 'left') == 1
                   nxt.m1Cal = nxt.m1Cal + 0.02;
                   nxt.m2Cal = nxt.m2Cal - 0.02;
               elseif strcmp(userInput, 'right') == 1
                   nxt.m1Cal = nxt.m1Cal - 0.02;
                   nxt.m2Cal = nxt.m2Cal + 0.02;
               else 
                   warning('Expected input: left; right; straight')
               end               
            end           
            
            %calibrate turning over under correct
            nxt.userReady('observe a 90 degree turn')
            correct = 0;
            while ~correct
               nxt.turn(pi/2)
               userInput = input(['Did the robot move over/under/correct?' newline], 's');            
               if strcmp(userInput, 'correct') == 1
                   correct = 1;
               elseif strcmp(userInput, 'over') == 1
                   nxt.degPerDeg = nxt.degPerDeg + 0.01;
               elseif strcmp(userInput, 'under') == 1
                   nxt.degPerDeg = nxt.degPerDeg - 0.01;
               else 
                   warning('Expected input: over; under; correct')
               end               
            end 
            
            %decide whether to save the new calibration variables, then
            %reload from calibration.mat
            userInput = input('Do you want to save to calibration.mat? (yes/no) ==> ', 's');
            if strcmp(userInput,'yes')
                nxt.saveCalibration()                
            else
                warning('Input not equal to "yes". Calibration variables will not be saved')
            end
            nxt.loadCalibration(); %load the new OR old calibration variables

        end
  
    end
    
end

