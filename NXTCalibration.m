function [cmPerDeg, movementNoise] = NXTCalibration(numTrials)

clf
ready = 'NoGo';

wheelDegs = 500;
wheelPow = 50;

while strcmp(ready,'GO') == 0
    ready = input(['Type GO when you are ready to measure robot movement' newline], 's');
end

COM_CloseNXT all        %prepares workspace
h = COM_OpenNXT();      %look for USB devices
COM_SetDefaultNXT(h);   %sets default handle

pause(.5)

mAB = NXTMotor('AB', 'Power', wheelPow, 'TachoLimit', wheelDegs);
%mB = NXTMotor('B', 'Power', wheelPow, 'TachoLimit', wheelDegs);
mAB.SpeedRegulation = false;
%MB.SpeedRegulation = false;
mAB.SmoothStart = true;
%MB.SmoothStart = true;

dists = zeros(numTrials, 1);

for i = 1:numTrials
    
    mAB.SendToNXT();
    %mB.SendToNXT();
    
    mAB.WaitFor();
    
    mAB.Stop('brake') 
    %mB.Stop('brake')

    dists(i) = input(['How many cm did the robot move?' newline]);

end

cmPerDeg = mean(dists) / wheelDegs;
movementNoise = std(dists);

sprintf('This robot moves %d cm per 1 degree of wheel rotation.' , cmPerDeg)
disp(newline)
sprintf('This robot has approximately %d movementNoise.' , movementNoise)
disp(newline)

end

