function [scan] = NXTUltraScan(numScans, degsFor360, leftOrRight, turnPow)

% COM_CloseNXT all        %prepares workspace
% h = COM_OpenNXT();      %look for USB devices
% COM_SetDefaultNXT(h);   %sets default handle
% pause(1)

%% 360 Rotation with Ultrascan

% numScans = 72;
% degsFor360 = 1500;      % Put the number of wheel degrees needed for the robot to turn 360 
% leftOrRight = 'Right';
% turnPow = 30;

isLeft = (strcmp(leftOrRight,'Left')*2 - 1);
scan = zeros(72, 1);

mA = NXTMotor('A', 'Power', turnPow*isLeft, 'TachoLimit', degsFor360);
mB = NXTMotor('B', 'Power', -turnPow*isLeft, 'TachoLimit', degsFor360);
OpenUltrasonic(SENSOR_1);
mB.SpeedRegulation = false;
mB.SmoothStart = true;
mA.SpeedRegulation = false;
mA.SmoothStart = true;

mB.ResetPosition(); mA.ResetPosition();
pause(.5);

datA = mA.ReadFromNXT();
posA = datA.Position;
datB = mB.ReadFromNXT();
posB = datB.Position;

mA.SendToNXT();
mB.SendToNXT();
scan(1) = GetUltrasonic(SENSOR_1);

scanCount = 1;

while (((max(posA,posB) <= degsFor360) || (min(posA,posB) >= -degsFor360)) && (datA.IsRunning == 1 || datB.IsRunning == 1)) || (scanCount == 1)
    if mean([abs(posA), abs(posB)]) > scanCount*(degsFor360/numScans)
        scanCount = scanCount +  1;
        scan(min(scanCount, numScans)) = GetUltrasonic(SENSOR_1);
    end
    datA = mA.ReadFromNXT();
    posA = datA.Position;
    datB = mB.ReadFromNXT();
    posB = datB.Position;
end
        
mA.WaitFor();
mB.WaitFor();

%% Now we'll turn back around

mA = NXTMotor('A', 'Power', -turnPow*isLeft, 'TachoLimit', degsFor360);
mB = NXTMotor('B', 'Power', turnPow*isLeft, 'TachoLimit', degsFor360);
mB.SpeedRegulation = false;
mB.SmoothStart = true;
mA.SpeedRegulation = false;
mA.SmoothStart = true;

mA.SendToNXT();
mB.SendToNXT();

mA.WaitFor();
mB.WaitFor();
    
end
    


