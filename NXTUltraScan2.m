function [scan] = NXTUltraScan2(numScans, turnPow)
%NXTUltraScan2(72, 20);

% COM_CloseNXT all;        %prepares workspace
% h = COM_OpenNXT();      %look for USB devices
% COM_SetDefaultNXT(h);   %sets default handle
% pause(1)

%% 360 Rotation with Ultrascan

% numScans = 72;
% degsFor360 = 1500;      % Put the number of wheel degrees needed for the robot to turn 360 
% leftOrRight = 'Right';
% turnPow = 30;

scan = zeros(numScans, 1);

mC = NXTMotor('C', 'Power', turnPow, 'TachoLimit', 360);

OpenUltrasonic(SENSOR_1);
mC.SpeedRegulation = false;
mC.SmoothStart = false;

mC.ResetPosition(); 
pause(.5);

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
        
mC.WaitFor();
%% Now we'll turn back around

mC = NXTMotor('C', 'Power', -turnPow, 'TachoLimit', 360);
mC.SpeedRegulation = false;
mC.SmoothStart = false;
mC.SendToNXT();

mC.WaitFor();
    
end
    